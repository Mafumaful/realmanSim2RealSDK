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

        # 获取参数
        ns = self.get_parameter('namespace').value
        self.gui_width = self.get_parameter('gui_width').value
        self.gui_height = self.get_parameter('gui_height').value

        # 话题名称（支持namespace）
        cmd_topic = f'{ns}/joint_commands' if ns else '/joint_commands'
        state_topic = f'{ns}/joint_states' if ns else '/joint_states'

        # 发布者
        self.publisher = self.create_publisher(JointState, cmd_topic, 10)

        # 订阅当前状态
        self.subscription = self.create_subscription(
            JointState, state_topic, self.state_callback, 10)

        self.current_positions = [0.0] * 19
        self.sliders = []
        self.value_labels = []
        self.realtime_labels = []

        self._create_gui()

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

        # 发送按钮
        btn_frame = ttk.Frame(self.root)
        btn_frame.pack(fill='x', padx=5, pady=5)

        ttk.Button(btn_frame, text='发送指令',
                   command=self.send_command).pack(side='left', padx=5)
        ttk.Button(btn_frame, text='归零',
                   command=self.reset_all).pack(side='left', padx=5)

    def _create_platform_tab(self, notebook):
        """创建平台标签页"""
        frame = ttk.Frame(notebook)
        notebook.add(frame, text='平台')

        name = JOINT_NAMES_LIST[0]
        limits = JOINT_LIMITS[name]
        min_deg = math.degrees(limits[0])
        max_deg = math.degrees(limits[1])

        self._add_slider(frame, 0, 'D1', min_deg, max_deg, 0)

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
            self._add_slider(frame, i, label, min_deg, max_deg, idx)

    def _add_slider(self, parent, row, label, min_val, max_val, idx):
        """添加滑块控件"""
        # 关节名称
        ttk.Label(parent, text=label, width=6).grid(
            row=row, column=0, padx=3, pady=3)

        # 实时值显示 (来自Isaac Sim)
        rt_label = ttk.Label(parent, text='--', width=10, foreground='blue')
        rt_label.grid(row=row, column=1, padx=3, pady=3)
        self.realtime_labels.append((idx, rt_label))

        # 滑块
        var = tk.DoubleVar(value=0.0)
        slider = ttk.Scale(parent, from_=min_val, to=max_val,
                           variable=var, length=350)
        slider.grid(row=row, column=2, padx=3, pady=3)

        # 目标值显示
        val_label = ttk.Label(parent, text='0.0°', width=10)
        val_label.grid(row=row, column=3, padx=3, pady=3)

        var.trace_add('write', lambda *_: val_label.config(
            text=f'{var.get():.1f}°'))

        self.sliders.append((idx, var))
        self.value_labels.append(val_label)

    def state_callback(self, msg: JointState):
        """更新当前状态显示"""
        if len(msg.position) == 19:
            self.current_positions = list(msg.position)
            # 更新实时值标签
            for idx, rt_label in self.realtime_labels:
                deg_val = math.degrees(self.current_positions[idx])
                rt_label.config(text=f'{deg_val:.1f}°')

    def send_command(self):
        """发送关节指令"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES_LIST

        positions = [0.0] * 19
        for idx, var in self.sliders:
            positions[idx] = math.radians(var.get())

        msg.position = positions
        self.publisher.publish(msg)
        self.get_logger().info('指令已发送')

    def reset_all(self):
        """所有滑块归零"""
        for idx, var in self.sliders:
            var.set(0.0)

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
