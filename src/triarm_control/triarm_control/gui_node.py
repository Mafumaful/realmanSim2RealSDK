"""
关节控制GUI模块

独立的GUI界面，通过ROS2话题与控制节点通信
支持23关节体系 (19臂关节 + 4夹爪关节)
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

try:
    import tkinter as tk
    from tkinter import ttk
    HAS_TK = True
except ImportError:
    HAS_TK = False

from .joint_names import JOINT_NAMES_LIST, JOINT_LIMITS, TOTAL_JOINT_COUNT


class JointControlGUI(Node):
    """关节控制GUI节点"""

    def __init__(self):
        super().__init__('triarm_gui')

        # 声明参数
        self.declare_parameter('namespace', '')
        self.declare_parameter('gui_width', 700)
        self.declare_parameter('gui_height', 600)

        # 获取参数
        ns = self.get_parameter('namespace').value
        self.gui_width = self.get_parameter('gui_width').value
        self.gui_height = self.get_parameter('gui_height').value

        # 话题名称
        prefix = f'{ns}/' if ns else '/'
        state_topic = f'{prefix}joint_states'
        target_topic = f'{prefix}target_joints'
        cmd_topic = f'{prefix}joint_command'

        # 订阅实时状态
        self.state_sub = self.create_subscription(
            JointState, state_topic, self._state_callback, 10)

        # 订阅指令反馈
        self.cmd_sub = self.create_subscription(
            JointState, cmd_topic, self._cmd_callback, 10)

        # 发布目标位置
        self.target_pub = self.create_publisher(Float64MultiArray, target_topic, 10)

        # 发布夹爪目标 (供 gripper_bridge real模式 和 unified_arm_node sim模式 使用)
        gripper_topic = f'{prefix}gripper_target'
        self.gripper_target_pub = self.create_publisher(Float64MultiArray, gripper_topic, 10)

        # 状态
        self.current_positions = [0.0] * TOTAL_JOINT_COUNT
        self.cmd_positions = [0.0] * TOTAL_JOINT_COUNT
        self.has_state = False

        # GUI组件
        self.entries = []
        self.sliders = []
        self.realtime_labels = []
        self.cmd_labels = []

        self._create_gui()

    def _create_gui(self):
        self.root = tk.Tk()
        self.root.title('三臂机器人关节控制')
        self.root.geometry(f'{self.gui_width}x{self.gui_height}')

        # Notebook分页
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill='both', expand=True, padx=5, pady=5)

        # 创建各组标签页
        self._create_platform_tab(notebook)
        self._create_arm_tab(notebook, 'A', 1)
        self._create_arm_tab(notebook, 'B', 7)
        self._create_arm_tab(notebook, 'S', 13)
        self._create_gripper_tab(notebook)

        # 按钮区域
        self._create_buttons()

    def _create_buttons(self):
        btn_frame = ttk.Frame(self.root)
        btn_frame.pack(fill='x', padx=5, pady=5)

        ttk.Button(btn_frame, text='执行',
                   command=self._send_target).pack(side='left', padx=5)
        ttk.Button(btn_frame, text='归零',
                   command=self._reset_all).pack(side='left', padx=5)

        # 状态标签
        self.status_label = ttk.Label(btn_frame, text='就绪', foreground='green')
        self.status_label.pack(side='right', padx=10)

    def _add_header_row(self, parent):
        headers = [('关节', 6), ('实时值', 8), ('指令值', 8), ('滑块控制', 20), ('目标角度', 8)]
        colors = [None, 'blue', 'green', None, None]
        for col, (text, width) in enumerate(headers):
            lbl = ttk.Label(parent, text=text, width=width, font=('', 9, 'bold'))
            if colors[col]:
                lbl.config(foreground=colors[col])
            lbl.grid(row=0, column=col, padx=2, pady=5)

    def _create_platform_tab(self, notebook):
        frame = ttk.Frame(notebook)
        notebook.add(frame, text='平台')
        self._add_header_row(frame)

        name = JOINT_NAMES_LIST[0]
        limits = JOINT_LIMITS[name]
        min_deg, max_deg = math.degrees(limits[0]), math.degrees(limits[1])
        self._add_joint_row(frame, 1, 'D1', min_deg, max_deg, 0)

    def _create_arm_tab(self, notebook, arm_name, start_idx):
        frame = ttk.Frame(notebook)
        notebook.add(frame, text=f'机械臂{arm_name}')
        self._add_header_row(frame)

        for i in range(6):
            idx = start_idx + i
            name = JOINT_NAMES_LIST[idx]
            limits = JOINT_LIMITS[name]
            min_deg, max_deg = math.degrees(limits[0]), math.degrees(limits[1])
            self._add_joint_row(frame, i+1, f'{arm_name}{i+1}', min_deg, max_deg, idx)

    def _create_gripper_tab(self, notebook):
        """创建夹爪标签页 (4个关节: L1, L11, R1, R11)"""
        frame = ttk.Frame(notebook)
        notebook.add(frame, text='夹爪')
        self._add_header_row(frame)

        gripper_joints = [
            (19, 'L1',  '左指1'),
            (20, 'L11', '左指2'),
            (21, 'R1',  '右指1'),
            (22, 'R11', '右指2'),
        ]
        for row, (idx, label, _desc) in enumerate(gripper_joints, start=1):
            name = JOINT_NAMES_LIST[idx]
            limits = JOINT_LIMITS[name]
            min_deg, max_deg = math.degrees(limits[0]), math.degrees(limits[1])
            self._add_joint_row(frame, row, label, min_deg, max_deg, idx)

        # 快捷按钮
        quick_frame = ttk.LabelFrame(frame, text='快捷操作', padding=5)
        quick_frame.grid(row=6, column=0, columnspan=5, pady=10, padx=5, sticky='ew')

        ttk.Button(quick_frame, text='左夹爪打开',
                   command=lambda: self._set_gripper('left', 0.0)).pack(side='left', padx=5)
        ttk.Button(quick_frame, text='左夹爪闭合',
                   command=lambda: self._set_gripper('left', 1.0)).pack(side='left', padx=5)
        ttk.Button(quick_frame, text='右夹爪打开',
                   command=lambda: self._set_gripper('right', 0.0)).pack(side='left', padx=15)
        ttk.Button(quick_frame, text='右夹爪闭合',
                   command=lambda: self._set_gripper('right', 1.0)).pack(side='left', padx=5)

    def _set_gripper(self, side: str, angle_rad: float):
        """快捷设置夹爪角度并立即发送

        同时发布:
        - /target_joints (23关节, sim模式 controller_node 使用)
        - /gripper_target (4关节, sim: unified_arm_node 合并到 /target_joints; real: gripper_bridge → Modbus)
        """
        angle_deg = math.degrees(angle_rad)
        neg_deg = math.degrees(-angle_rad)

        if side == 'left':
            pairs = [(19, angle_deg), (20, neg_deg)]
        else:
            pairs = [(21, angle_deg), (22, neg_deg)]

        for idx, deg in pairs:
            for s_idx, var, _, _ in self.sliders:
                if s_idx == idx:
                    var.set(deg)
            for e_idx, entry, _, _ in self.entries:
                if e_idx == idx:
                    entry.delete(0, tk.END)
                    entry.insert(0, f'{deg:.1f}')

        # 发布完整23关节目标 (sim: controller_node → Isaac Sim)
        self._send_target()

        # 同时发布 /gripper_target (real: gripper_bridge → Modbus)
        # 从滑块读取另一侧当前值，避免覆盖归零
        gripper_data = [0.0] * 4  # [L1, L11, R1, R11]
        gripper_idx_to_slot = {19: 0, 20: 1, 21: 2, 22: 3}
        for s_idx, var, _, _ in self.sliders:
            if s_idx in gripper_idx_to_slot:
                gripper_data[gripper_idx_to_slot[s_idx]] = math.radians(var.get())
        if side == 'left':
            gripper_data[0] = angle_rad
            gripper_data[1] = -angle_rad
        else:
            gripper_data[2] = angle_rad
            gripper_data[3] = -angle_rad
        gripper_msg = Float64MultiArray()
        gripper_msg.data = gripper_data
        self.gripper_target_pub.publish(gripper_msg)

    def _add_joint_row(self, parent, row, label, min_val, max_val, idx):
        ttk.Label(parent, text=label, width=6).grid(row=row, column=0, padx=2, pady=2)

        # 实时值
        rt_label = ttk.Label(parent, text='--', width=8, foreground='blue')
        rt_label.grid(row=row, column=1, padx=2, pady=2)
        self.realtime_labels.append((idx, rt_label))

        # 指令值
        cmd_label = ttk.Label(parent, text='--', width=8, foreground='green')
        cmd_label.grid(row=row, column=2, padx=2, pady=2)
        self.cmd_labels.append((idx, cmd_label))

        # 滑块
        var = tk.DoubleVar(value=0.0)
        slider = ttk.Scale(parent, from_=min_val, to=max_val, variable=var, length=200)
        slider.grid(row=row, column=3, padx=2, pady=2)
        self.sliders.append((idx, var, min_val, max_val))

        # 输入框
        entry = ttk.Entry(parent, width=8)
        entry.insert(0, '0.0')
        entry.grid(row=row, column=4, padx=2, pady=2)
        self.entries.append((idx, entry, min_val, max_val))

        # 双向同步
        var.trace_add('write', lambda *_, v=var, e=entry: self._sync_entry(v, e))
        entry.bind('<Return>', lambda ev, v=var, e=entry: self._sync_slider(v, e))

    def _sync_entry(self, var, entry):
        entry.delete(0, tk.END)
        entry.insert(0, f'{var.get():.1f}')

    def _sync_slider(self, var, entry):
        try:
            var.set(float(entry.get()))
        except ValueError:
            pass

    def _state_callback(self, msg: JointState):
        if msg.name and msg.position:
            for i, name in enumerate(msg.name):
                if i < len(msg.position) and name in JOINT_NAMES_LIST:
                    idx = JOINT_NAMES_LIST.index(name)
                    self.current_positions[idx] = msg.position[i]
            self.has_state = True
            self._update_realtime_labels()

    def _cmd_callback(self, msg: JointState):
        if msg.name and msg.position:
            for i, name in enumerate(msg.name):
                if i < len(msg.position) and name in JOINT_NAMES_LIST:
                    idx = JOINT_NAMES_LIST.index(name)
                    self.cmd_positions[idx] = msg.position[i]
            self._update_cmd_labels()

    def _update_realtime_labels(self):
        for idx, label in self.realtime_labels:
            label.config(text=f'{math.degrees(self.current_positions[idx]):.1f}°')

    def _update_cmd_labels(self):
        for idx, label in self.cmd_labels:
            label.config(text=f'{math.degrees(self.cmd_positions[idx]):.1f}°')

    def _send_target(self):
        if not self.has_state:
            self.status_label.config(text='等待数据...', foreground='red')
            return

        targets = [0.0] * TOTAL_JOINT_COUNT
        for idx, entry, min_val, max_val in self.entries:
            try:
                val = float(entry.get())
                val = max(min_val, min(max_val, val))
                targets[idx] = val
            except ValueError:
                pass

        msg = Float64MultiArray()
        msg.data = targets
        self.target_pub.publish(msg)
        self.status_label.config(text='已发送', foreground='orange')

    def _reset_all(self):
        for idx, entry, _, _ in self.entries:
            entry.delete(0, tk.END)
            entry.insert(0, '0.0')
        for idx, var, _, _ in self.sliders:
            var.set(0.0)

    def run(self):
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
