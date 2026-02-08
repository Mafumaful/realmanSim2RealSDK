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
        cmd_topic = f'{ns}/joint_commands' if ns else '/joint_commands'
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

        # GUI 组件
        self.entries = []          # 输入框
        self.realtime_labels = []  # 实时值标签

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

    def _create_platform_tab(self, notebook):
        """创建平台标签页"""
        frame = ttk.Frame(notebook)
        notebook.add(frame, text='平台')

        name = JOINT_NAMES_LIST[0]
        limits = JOINT_LIMITS[name]
        min_deg = math.degrees(limits[0])
        max_deg = math.degrees(limits[1])

        self._add_joint_row(frame, 0, 'D1', min_deg, max_deg, 0)

    def _create_arm_tab(self, notebook, arm_name, start_idx):
        """创建机械臂标签页"""
        frame = ttk.Frame(notebook)
        notebook.add(frame, text=f'机械臂{arm_name}')

        for i in range(6):
            idx = start_idx + i
            name = JOINT_NAMES_LIST[idx]
            limits = JOINT_LIMITS[name]
            min_deg = math.degrees(limits[0])
            max_deg = math.degrees(limits[1])
            label = f'{arm_name}{i+1}'
            self._add_joint_row(frame, i, label, min_deg, max_deg, idx)

    def _add_joint_row(self, parent, row, label, min_val, max_val, idx):
        """添加关节控制行"""
        # 关节名称
        ttk.Label(parent, text=label, width=6).grid(
            row=row, column=0, padx=3, pady=3)

        # 实时值显示 (来自Isaac Sim)
        rt_label = ttk.Label(parent, text='--', width=10, foreground='blue')
        rt_label.grid(row=row, column=1, padx=3, pady=3)
        self.realtime_labels.append((idx, rt_label))

        # 目标角度输入框
        entry = ttk.Entry(parent, width=10)
        entry.insert(0, '0.0')
        entry.grid(row=row, column=2, padx=3, pady=3)

        # 限位提示
        ttk.Label(parent, text=f'[{min_val:.1f}, {max_val:.1f}]',
                  width=16, foreground='gray').grid(row=row, column=3, padx=3, pady=3)

        self.entries.append((idx, entry, min_val, max_val))

    def state_callback(self, msg: JointState):
        """更新当前状态显示"""
        if len(msg.position) == 19:
            self.current_positions = list(msg.position)
            # 更新实时值标签
            for idx, rt_label in self.realtime_labels:
                deg_val = math.degrees(self.current_positions[idx])
                rt_label.config(text=f'{deg_val:.1f}°')

    def start_motion(self):
        """开始运动"""
        # 读取输入框中的目标角度
        for idx, entry, min_val, max_val in self.entries:
            try:
                val = float(entry.get())
                val = max(min_val, min(max_val, val))  # 限位
                self.target_positions[idx] = math.radians(val)
            except ValueError:
                pass

        # 初始化指令位置为当前位置
        self.cmd_positions = list(self.current_positions)
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
