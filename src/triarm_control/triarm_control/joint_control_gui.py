"""
关节控制GUI节点

- 订阅 /joint_states (来自Isaac Sim) 显示实时值
- 发布 /joint_commands 发送目标角度指令
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

try:
    import tkinter as tk
    from tkinter import ttk
    HAS_TK = True
except ImportError:
    HAS_TK = False

from .joint_names import JOINT_NAMES_LIST, JOINT_LIMITS


class JointControlGUI(Node):
    """关节控制GUI节点"""

    def __init__(self):
        super().__init__('triarm_joint_control_gui')

        # 声明参数
        self.declare_parameter('namespace', '')
        self.declare_parameter('gui_width', 700)
        self.declare_parameter('gui_height', 500)
        self.declare_parameter('joint_velocity', 30.0)
        self.declare_parameter('publish_rate', 50.0)

        # 获取参数
        ns = self.get_parameter('namespace').value
        self.gui_width = self.get_parameter('gui_width').value
        self.gui_height = self.get_parameter('gui_height').value
        self.joint_velocity = self.get_parameter('joint_velocity').value  # 度/秒
        self.publish_rate = self.get_parameter('publish_rate').value  # Hz

        # 话题名称（支持namespace）
        cmd_topic = f'{ns}/joint_command' if ns else '/joint_command'
        state_topic = f'{ns}/joint_states' if ns else '/joint_states'

        # 发布者
        self.publisher = self.create_publisher(JointState, cmd_topic, 10)

        # 订阅当前状态
        self.subscription = self.create_subscription(
            JointState, state_topic, self.state_callback, 10)

        # 状态变量
        self.current_positions = [0.0] * 19  # 当前位置 (弧度)
        self.target_positions = [0.0] * 19   # 目标位置 (弧度)
        self.cmd_positions = [0.0] * 19      # 插值指令位置 (弧度)
        self.is_moving = False               # 是否正在运动
        self.has_received_state = False      # 是否已收到实时数据
        self.is_initialized = False          # 是否已初始化cmd_positions

        # GUI 组件
        self.entries = []          # 输入框
        self.sliders = []          # 滑块
        self.realtime_labels = []  # 实时值标签 (来自Isaac Sim)
        self.cmd_labels = []       # 被控量标签 (发送的指令)

        self._create_gui()

        # 定时发布器
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self._timer_callback)

    def _create_gui(self):
        """创建GUI界面"""
        self.root = tk.Tk()
        self.root.title('三臂机器人关节控制')
        self.root.geometry(f'{self.gui_width}x{self.gui_height}')

        # 创建notebook用于分组
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill='both', expand=True, padx=5, pady=5)

        # 创建各组标签页
        self._create_platform_tab(notebook)
        self._create_arm_tab(notebook, 'A', 1)
        self._create_arm_tab(notebook, 'B', 7)
        self._create_arm_tab(notebook, 'S', 13)

        # 按钮区域
        btn_frame = ttk.Frame(self.root)
        btn_frame.pack(fill='x', padx=5, pady=5)

        ttk.Button(btn_frame, text='执行',
                   command=self.start_motion).pack(side='left', padx=5)
        ttk.Button(btn_frame, text='停止',
                   command=self.stop_motion).pack(side='left', padx=5)
        ttk.Button(btn_frame, text='归零',
                   command=self.reset_all).pack(side='left', padx=5)

        # 状态标签
        self.status_label = ttk.Label(btn_frame, text='就绪', foreground='green')
        self.status_label.pack(side='right', padx=10)

    def _add_header_row(self, parent):
        """添加表头行"""
        ttk.Label(parent, text='关节', width=4, font=('', 9, 'bold')).grid(
            row=0, column=0, padx=2, pady=5)
        ttk.Label(parent, text='实时值', width=8, foreground='blue', font=('', 9, 'bold')).grid(
            row=0, column=1, padx=2, pady=5)
        ttk.Label(parent, text='指令值', width=8, foreground='green', font=('', 9, 'bold')).grid(
            row=0, column=2, padx=2, pady=5)
        ttk.Label(parent, text='滑块控制', width=20, font=('', 9, 'bold')).grid(
            row=0, column=3, padx=2, pady=5)
        ttk.Label(parent, text='目标角度', width=8, font=('', 9, 'bold')).grid(
            row=0, column=4, padx=2, pady=5)

    def _create_platform_tab(self, notebook):
        """创建平台标签页"""
        frame = ttk.Frame(notebook)
        notebook.add(frame, text='平台')

        # 添加表头
        self._add_header_row(frame)

        name = JOINT_NAMES_LIST[0]
        limits = JOINT_LIMITS[name]
        min_deg = math.degrees(limits[0])
        max_deg = math.degrees(limits[1])

        self._add_joint_row(frame, 1, 'D1', min_deg, max_deg, 0)

    def _create_arm_tab(self, notebook, arm_name, start_idx):
        """创建机械臂标签页"""
        frame = ttk.Frame(notebook)
        notebook.add(frame, text=f'机械臂{arm_name}')

        # 添加表头
        self._add_header_row(frame)

        for i in range(6):
            idx = start_idx + i
            name = JOINT_NAMES_LIST[idx]
            limits = JOINT_LIMITS[name]
            min_deg = math.degrees(limits[0])
            max_deg = math.degrees(limits[1])
            label = f'{arm_name}{i+1}'
            self._add_joint_row(frame, i+1, label, min_deg, max_deg, idx)

    def _add_joint_row(self, parent, row, label, min_val, max_val, idx):
        """添加关节控制行"""
        # 关节名称
        ttk.Label(parent, text=label, width=4).grid(
            row=row, column=0, padx=2, pady=2)

        # 实时值显示 (来自Isaac Sim)
        rt_label = ttk.Label(parent, text='--', width=8, foreground='blue')
        rt_label.grid(row=row, column=1, padx=2, pady=2)
        self.realtime_labels.append((idx, rt_label))

        # 被控量显示 (发送的指令)
        cmd_label = ttk.Label(parent, text='--', width=8, foreground='green')
        cmd_label.grid(row=row, column=2, padx=2, pady=2)
        self.cmd_labels.append((idx, cmd_label))

        # 滑块
        var = tk.DoubleVar(value=0.0)
        slider = ttk.Scale(parent, from_=min_val, to=max_val,
                           variable=var, length=200)
        slider.grid(row=row, column=3, padx=2, pady=2)
        self.sliders.append((idx, var, min_val, max_val))

        # 目标角度输入框
        entry = ttk.Entry(parent, width=8)
        entry.insert(0, '0.0')
        entry.grid(row=row, column=4, padx=2, pady=2)
        self.entries.append((idx, entry, min_val, max_val))

        # 滑块和输入框双向同步
        var.trace_add('write', lambda *_, v=var, e=entry: self._sync_entry(v, e))
        entry.bind('<Return>', lambda ev, v=var, e=entry: self._sync_slider(v, e))

    def _sync_entry(self, var, entry):
        """滑块变化时同步到输入框"""
        entry.delete(0, tk.END)
        entry.insert(0, f'{var.get():.1f}')

    def _sync_slider(self, var, entry):
        """输入框回车时同步到滑块"""
        try:
            val = float(entry.get())
            var.set(val)
        except ValueError:
            pass

    def state_callback(self, msg: JointState):
        """更新当前状态显示"""
        if len(msg.position) > 0 and len(msg.name) > 0:
            # 按关节名称匹配更新位置（避免顺序错乱）
            for i, joint_name in enumerate(msg.name):
                if i < len(msg.position) and joint_name in JOINT_NAMES_LIST:
                    idx = JOINT_NAMES_LIST.index(joint_name)
                    self.current_positions[idx] = msg.position[i]
            self.has_received_state = True

            # 更新实时值标签
            for idx, rt_label in self.realtime_labels:
                deg_val = math.degrees(self.current_positions[idx])
                rt_label.config(text=f'{deg_val:.1f}°')

    def start_motion(self):
        """开始运动"""
        # 检查是否已收到实时数据
        if not self.has_received_state:
            self.status_label.config(text='等待实时数据...', foreground='red')
            self.get_logger().warn('尚未收到关节状态数据')
            return

        # 读取输入框中的目标角度
        for idx, entry, min_val, max_val in self.entries:
            try:
                val = float(entry.get())
                val = max(min_val, min(max_val, val))  # 限位
                self.target_positions[idx] = math.radians(val)
            except ValueError:
                pass

        # 只在第一次初始化时从实时位置开始
        if not self.is_initialized:
            self.cmd_positions = list(self.current_positions)
            self.is_initialized = True
            self.get_logger().info(f'初始化位置: {[f"{math.degrees(p):.1f}" for p in self.cmd_positions[:5]]}...')

        self.is_moving = True
        self.status_label.config(text='运动中...', foreground='orange')

    def stop_motion(self):
        """停止运动"""
        self.is_moving = False
        self.status_label.config(text='已停止', foreground='red')

    def reset_all(self):
        """所有输入框归零"""
        for idx, entry, min_val, max_val in self.entries:
            entry.delete(0, tk.END)
            entry.insert(0, '0.0')

    def _timer_callback(self):
        """定时器回调：插值运动并发布指令"""
        if not self.is_moving:
            return

        # 计算每个周期的最大角度变化量 (弧度)
        dt = 1.0 / self.publish_rate
        max_delta = math.radians(self.joint_velocity) * dt

        # 检查是否所有关节都到达目标
        all_reached = True

        for i in range(19):
            diff = self.target_positions[i] - self.cmd_positions[i]
            if abs(diff) > 0.001:  # 约0.06度
                all_reached = False
                # 限制单步变化量
                if abs(diff) > max_delta:
                    diff = max_delta if diff > 0 else -max_delta
                self.cmd_positions[i] += diff

        # 发布指令
        self._publish_command()

        # 检查是否完成
        if all_reached:
            self.is_moving = False
            self.status_label.config(text='完成', foreground='green')

    def _publish_command(self):
        """发布关节指令"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES_LIST
        msg.position = list(self.cmd_positions)
        self.publisher.publish(msg)

        # 更新被控量显示
        for idx, cmd_label in self.cmd_labels:
            deg_val = math.degrees(self.cmd_positions[idx])
            cmd_label.config(text=f'{deg_val:.1f}°')

    def run(self):
        """运行GUI主循环"""
        def ros_spin():
            rclpy.spin_once(self, timeout_sec=0.01)
            self.root.after(10, ros_spin)

        self.root.after(10, ros_spin)
        self.root.mainloop()


def main(args=None):
    if not HAS_TK:
        print('错误: 需要安装tkinter')
        return

    rclpy.init(args=args)
    node = JointControlGUI()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
