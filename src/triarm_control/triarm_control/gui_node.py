"""
关节控制GUI模块

独立的GUI界面，通过ROS2话题与控制节点通信
支持23关节体系 (19臂关节 + 4夹爪关节)
"""

import math
import threading
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String, Bool

try:
    import tkinter as tk
    from tkinter import ttk
    HAS_TK = True
except ImportError:
    HAS_TK = False

from rm_ros_interfaces.msg import Movel, Movejp, Movej
from geometry_msgs.msg import Pose
from .joint_names import JOINT_NAMES_LIST, JOINT_LIMITS, TOTAL_JOINT_COUNT
from .realman_sdk_wrapper import RealManAlgo


class JointControlGUI(Node):
    """关节控制GUI节点"""

    def __init__(self):
        super().__init__('triarm_gui')

        # 声明参数
        self.declare_parameter('namespace', 'robot')
        self.declare_parameter('mode', 'sim')
        self.declare_parameter('gui_width', 700)
        self.declare_parameter('gui_height', 600)

        # per-arm base 参数 (与 unified_arm_node 一致, 用于 RM65Robot IK/FK)
        self.declare_parameter('arm_a.base_position', [0.05457, -0.04863, 0.2273])
        self.declare_parameter('arm_a.base_orientation_deg', [45.0, 90.0, 0.0])
        self.declare_parameter('arm_a.d6_mm', 172.5)
        self.declare_parameter('arm_b.base_position', [-0.04867, -0.05374, 0.2273])
        self.declare_parameter('arm_b.base_orientation_deg', [135.0, 90.0, 0.0])
        self.declare_parameter('arm_b.d6_mm', 172.5)

        # 获取参数
        ns = self.get_parameter('namespace').value
        self.mode = self.get_parameter('mode').value
        self.gui_width = self.get_parameter('gui_width').value
        self.gui_height = self.get_parameter('gui_height').value

        # 从controller节点获取夹爪参数
        self._load_gripper_params_from_controller()

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

        # Real模式：创建rm_driver发布器
        if self.mode == 'real':
            self.movej_pub_a = self.create_publisher(
                Movej, '/arm_a/rm_driver/movej_cmd', 10)
            self.movej_pub_b = self.create_publisher(
                Movej, '/arm_b/rm_driver/movej_cmd', 10)
            self.get_logger().info('Real模式：已创建rm_driver发布器')

        # 发布夹爪目标 (供 gripper_bridge real模式 和 unified_arm_node sim模式 使用)
        gripper_topic = f'{prefix}gripper_target'
        self.gripper_target_pub = self.create_publisher(Float64MultiArray, gripper_topic, 10)

        # 发布控制参数（用于动态修改）
        self.smooth_pub = self.create_publisher(Bool, f'{prefix}enable_smooth', 10)
        self.velocity_mode_pub = self.create_publisher(String, f'{prefix}velocity_mode', 10)

        # 发布夹爪控制指令
        self.gripper_pub = self.create_publisher(String, f'{prefix}gripper_control', 10)

        # 发布 MoveL/MoveJP/MoveP 命令
        self.movel_pubs = {
            'arm_a': self.create_publisher(Movel, 'arm_a/rm_driver/movel_cmd', 10),
            'arm_b': self.create_publisher(Movel, 'arm_b/rm_driver/movel_cmd', 10),
        }
        self.movejp_pubs = {
            'arm_a': self.create_publisher(Movejp, 'arm_a/rm_driver/movej_p_cmd', 10),
            'arm_b': self.create_publisher(Movejp, 'arm_b/rm_driver/movej_p_cmd', 10),
        }
        self.movep_pubs = {
            'arm_a': self.create_publisher(Movel, 'arm_a/rm_driver/movep_cmd', 10),
            'arm_b': self.create_publisher(Movel, 'arm_b/rm_driver/movep_cmd', 10),
        }

        # 订阅运动结果
        for arm in ['arm_a', 'arm_b']:
            self.create_subscription(
                Bool, f'{arm}/rm_driver/movel_result',
                self._make_move_result_cb(arm, 'movel'), 10)
            self.create_subscription(
                Bool, f'{arm}/rm_driver/movej_p_result',
                self._make_move_result_cb(arm, 'movejp'), 10)
            self.create_subscription(
                Bool, f'{arm}/rm_driver/movep_result',
                self._make_move_result_cb(arm, 'movep'), 10)

        # 状态
        self.current_positions = [0.0] * TOTAL_JOINT_COUNT
        self.cmd_positions = [0.0] * TOTAL_JOINT_COUNT
        self.has_state = False
        self._world_pose_cmd_seq = 0
        self._pending_world_pose_cmd = None
        self._world_pose_cmd_lock = threading.Lock()

        # per-arm 算法库 (带各自 base 参数, 与 unified_arm_node 保持一致)
        self._algos = {}
        for arm in ['arm_a', 'arm_b']:
            bp = list(self.get_parameter(f'{arm}.base_position').value)
            bo = list(self.get_parameter(f'{arm}.base_orientation_deg').value)
            d6 = self.get_parameter(f'{arm}.d6_mm').value
            algo = RealManAlgo(base_position=bp, base_orientation_deg=bo, d6_mm=d6)
            algo.initialize()
            self._algos[arm] = algo
        # 默认引用 (向后兼容其他使用 pos2matrix/matrix2pos 的地方)
        self._algo = self._algos['arm_a']

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
        self._create_world_pose_tab(notebook)

        # 按钮区域
        self._create_buttons()

    def _create_buttons(self):
        btn_frame = ttk.Frame(self.root)
        btn_frame.pack(fill='x', padx=5, pady=5)

        # 左侧：执行和归零按钮
        ttk.Button(btn_frame, text='执行',
                   command=self._send_target).pack(side='left', padx=5)
        ttk.Button(btn_frame, text='归零',
                   command=self._reset_all).pack(side='left', padx=5)

        # 中间：平滑处理勾选框
        self.smooth_var = tk.BooleanVar(value=True)
        smooth_check = ttk.Checkbutton(
            btn_frame, text='平滑处理',
            variable=self.smooth_var,
            command=self._on_smooth_changed
        )
        smooth_check.pack(side='left', padx=15)

        # 速度模式选择
        ttk.Label(btn_frame, text='速度:').pack(side='left', padx=(15, 5))
        self.velocity_mode = tk.StringVar(value='normal')
        velocity_combo = ttk.Combobox(
            btn_frame,
            textvariable=self.velocity_mode,
            values=['slow', 'normal', 'fast'],
            state='readonly',
            width=8
        )
        velocity_combo.pack(side='left', padx=5)
        velocity_combo.bind('<<ComboboxSelected>>', self._on_velocity_changed)

        # 右侧：状态标签
        self.status_label = ttk.Label(btn_frame, text='就绪', foreground='green')
        self.status_label.pack(side='right', padx=10)

        # 夹爪控制区域
        self._create_gripper_controls()

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

        # 快捷按钮 (角度从 JOINT_LIMITS 读取)
        l_open = JOINT_LIMITS['joint_L1'][1]   # 0.0 rad
        l_close = JOINT_LIMITS['joint_L1'][0]  # deg2rad(-30)
        r_open = JOINT_LIMITS['joint_R1'][1]
        r_close = JOINT_LIMITS['joint_R1'][0]

        quick_frame = ttk.LabelFrame(frame, text='快捷操作', padding=5)
        quick_frame.grid(row=6, column=0, columnspan=5, pady=10, padx=5, sticky='ew')

        ttk.Button(quick_frame, text='左夹爪打开',
                   command=lambda: self._set_gripper('left', l_open)).pack(side='left', padx=5)
        ttk.Button(quick_frame, text='左夹爪闭合',
                   command=lambda: self._set_gripper('left', l_close)).pack(side='left', padx=5)
        ttk.Button(quick_frame, text='右夹爪打开',
                   command=lambda: self._set_gripper('right', r_open)).pack(side='left', padx=15)
        ttk.Button(quick_frame, text='右夹爪闭合',
                   command=lambda: self._set_gripper('right', r_close)).pack(side='left', padx=5)


    def _create_world_pose_tab(self, notebook):
        """创建世界坐标位姿输入标签页"""
        frame = ttk.Frame(notebook)
        notebook.add(frame, text='世界坐标')

        # 臂选择
        ttk.Label(frame, text='目标臂:').grid(row=0, column=0, padx=5, pady=10)
        self.target_arm_var = tk.StringVar(value='arm_a')
        ttk.Radiobutton(frame, text='A臂', variable=self.target_arm_var, value='arm_a').grid(row=0, column=1)
        ttk.Radiobutton(frame, text='B臂', variable=self.target_arm_var, value='arm_b').grid(row=0, column=2)

        # 位姿输入
        labels = ['x(m)', 'y(m)', 'z(m)', 'rx(rad)', 'ry(rad)', 'rz(rad)']
        defaults = ['0.3', '0.0', '0.4', '0.0', '3.14', '0.0']
        self.world_pose_entries = []
        for col, (lbl, val) in enumerate(zip(labels, defaults)):
            ttk.Label(frame, text=lbl).grid(row=1, column=col, padx=5, pady=5)
            e = ttk.Entry(frame, width=10)
            e.insert(0, val)
            e.grid(row=2, column=col, padx=5)
            self.world_pose_entries.append(e)

        # 速度
        ttk.Label(frame, text='速度:').grid(row=3, column=0, pady=10)
        self.move_speed_entry = ttk.Entry(frame, width=6)
        self.move_speed_entry.insert(0, '20')
        self.move_speed_entry.grid(row=3, column=1)

        # 按钮
        btn_frame = ttk.Frame(frame)
        btn_frame.grid(row=4, column=0, columnspan=6, pady=15)
        ttk.Button(btn_frame, text='读取当前位姿', command=self._read_current_pose).pack(side='left', padx=10)
        ttk.Button(btn_frame, text='MoveL', command=lambda: self._send_world_pose('movel')).pack(side='left', padx=10)
        ttk.Button(btn_frame, text='MoveJP', command=lambda: self._send_world_pose('movejp')).pack(side='left', padx=10)
        ttk.Button(btn_frame, text='MoveP', command=lambda: self._send_world_pose('movep')).pack(side='left', padx=10)
        ttk.Button(btn_frame, text='随机可达点', command=self._gen_random_reachable).pack(side='left', padx=10)
        ttk.Button(btn_frame, text='直接执行(基座系)', command=self._send_base_pose).pack(side='left', padx=10)

        # 状态标签
        self.world_pose_status = ttk.Label(frame, text='就绪', foreground='green')
        self.world_pose_status.grid(row=5, column=0, columnspan=6, pady=10)

        # 当前位姿显示（base = IK/FK控制坐标系, World = Isaac Sim坐标系）
        current_frame = ttk.LabelFrame(frame, text='当前位姿', padding=10)
        current_frame.grid(row=6, column=0, columnspan=6, pady=10, padx=10, sticky='ew')

        self.current_pose_label = ttk.Label(
            current_frame,
            text='base:  X=-, Y=-, Z=- | RX=-, RY=-, RZ=- \n'
                 'World: X=-, Y=-, Z=- | RX=-, RY=-, RZ=- ',
            font=('Courier', 10),
            justify='left'
        )
        self.current_pose_label.pack()

    def _set_gripper(self, side: str, angle_rad: float):
        """快捷设置夹爪角度并立即发送

        仅发布 /gripper_target (4关节):
          sim:  unified_arm_node 合并到共享数组 → /target_joints → controller_node
          real: gripper_bridge → Modbus RTU

        不直接发布 /target_joints，避免与 unified_arm_node 的共享数组产生竞态。
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

        # 发布 /gripper_target，从滑块读取另一侧当前值避免覆盖归零
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
            self._update_current_pose_display()

    def _cmd_callback(self, msg: JointState):
        if msg.name and msg.position:
            for i, name in enumerate(msg.name):
                if i < len(msg.position) and name in JOINT_NAMES_LIST:
                    idx = JOINT_NAMES_LIST.index(name)
                    self.cmd_positions[idx] = msg.position[i]
            self._update_cmd_labels()
            self._sync_sliders_to_cmd()

    def _update_realtime_labels(self):
        for idx, label in self.realtime_labels:
            label.config(text=f'{math.degrees(self.current_positions[idx]):.1f}°')

    def _update_current_pose_display(self):
        """更新当前位姿显示（base_link + Isaac Sim World）"""
        try:
            if not self.has_state:
                return

            arm = self.target_arm_var.get()
            algo = self._algos.get(arm)
            if not algo or not algo.is_ready:
                return

            # 提取当前臂的关节角度
            arm_prefix = 'A' if arm == 'arm_a' else 'B'
            joints_deg = []
            for i in range(1, 7):
                idx = JOINT_NAMES_LIST.index(f'joint_platform_{arm_prefix}{i}')
                joints_deg.append(math.degrees(self.current_positions[idx]))

            # 设置 D1 角度
            d1_deg = math.degrees(self.current_positions[0])
            algo._robot.set_turntable_angle(-d1_deg)

            # FK 计算 — RM65Robot 世界坐标系（用于 IK 控制）
            result = algo.forward_kinematics(joints_deg, 1)
            if result:
                pos = result[:3]
                ori_rad = result[3:]
                ori_deg = [math.degrees(r) for r in ori_rad]

                # 计算 Isaac Sim World 坐标（使用 URDF 变换链）
                import numpy as np
                wx, wy, wz, wo_deg = self._calc_isaac_world_pose(
                    arm, joints_deg, d1_deg)

                text = (f'base:  X={pos[0]:.4f}, Y={pos[1]:.4f}, Z={pos[2]:.4f} (m) '
                       f'| RX={ori_deg[0]:.1f}, RY={ori_deg[1]:.1f}, RZ={ori_deg[2]:.1f} (°) \n'
                       f'World: X={wx:.4f}, Y={wy:.4f}, Z={wz:.4f} (m) '
                       f'| RX={wo_deg[0]:.1f}, RY={wo_deg[1]:.1f}, RZ={wo_deg[2]:.1f} (°)')

                self.current_pose_label.config(text=text)
        except Exception as e:
            self.get_logger().error(f'更新位姿显示失败: {e}')

    def _calc_isaac_world_pose(self, arm, joints_deg, d1_deg):
        """使用 URDF 变换链计算 Isaac Sim World 坐标"""
        import numpy as np

        def _Rx(a):
            c, s = math.cos(a), math.sin(a)
            return np.array([[1,0,0],[0,c,-s],[0,s,c]])
        def _Ry(a):
            c, s = math.cos(a), math.sin(a)
            return np.array([[c,0,s],[0,1,0],[-s,0,c]])
        def _Rz(a):
            c, s = math.cos(a), math.sin(a)
            return np.array([[c,-s,0],[s,c,0],[0,0,1]])

        def _T(rpy, xyz):
            T = np.eye(4)
            T[0:3, 0:3] = _Rz(rpy[2]) @ _Ry(rpy[1]) @ _Rx(rpy[0])
            T[0:3, 3] = xyz
            return T

        def _Tj(angle_rad, axis_sign=1):
            T = np.eye(4)
            T[0:3, 0:3] = _Rz(axis_sign * angle_rad)
            return T

        jr = [math.radians(d) for d in joints_deg]
        d1_rad = math.radians(d1_deg)

        if arm == 'arm_a':
            T_A0 = _T([0.7854, -1.5708, 0], [-0.04867, -0.053739, 0.2273])
            T_J1_origin = _T([3.1416, 0, 0], [0, 0, 0.2405])
            a1_sign = -1  # A1 axis = "0 0 -1"
        else:
            T_A0 = _T([2.3562, -1.5708, 0], [0.054575, -0.04863, 0.2273])
            T_J1_origin = _T([3.1416, 0, 0], [0, 0, 0.2405])
            a1_sign = -1  # B1 axis 也需要检查

        # D1
        T_D1 = _T([0, 0, 3.1416], [0, 0, 0]) @ _Tj(d1_rad)

        # 关节 1-6
        T = (T_D1 @ T_A0 @
             T_J1_origin @ _Tj(jr[0], a1_sign) @
             _T([1.5707963267949, 1.5707963267949, 0], [0, 0, 0]) @ _Tj(jr[1]) @
             _T([0, 0, 1.57079632679477], [0.256, 0, 0]) @ _Tj(jr[2]) @
             _T([1.5707963267949, 0, 0], [0, -0.21, -0.0003]) @ _Tj(jr[3]) @
             _T([-1.5707963267949, 0, 0], [0, 0, 0]) @ _Tj(jr[4]) @
             _T([1.5707963267949, 0, 0], [0, -0.1725, -0.0003]) @ _Tj(jr[5]))

        pos = T[0:3, 3]
        # Isaac Sim 的实际坐标需要额外 Rz(-pi/2) 修正
        # (URDF rpy 约定与 Isaac Sim 内部表示的差异)
        c90, s90 = math.cos(-math.pi/2), math.sin(-math.pi/2)
        Rz_corr = np.array([[c90, -s90, 0, 0], [s90, c90, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        T = Rz_corr @ T

        pos = T[0:3, 3]
        # 提取欧拉角 (ZYX)
        R = T[0:3, 0:3]
        ry = math.asin(-R[2, 0]) if abs(R[2, 0]) < 1 else math.copysign(math.pi/2, -R[2, 0])
        rx = math.atan2(R[2, 1], R[2, 2])
        rz = math.atan2(R[1, 0], R[0, 0])
        ori_deg = [math.degrees(rx), math.degrees(ry), math.degrees(rz)]

        return pos[0], pos[1], pos[2], ori_deg

    def _update_cmd_labels(self):
        for idx, label in self.cmd_labels:
            label.config(text=f'{math.degrees(self.cmd_positions[idx]):.1f}°')

    def _sync_sliders_to_cmd(self):
        """将滑块和输入框同步到当前指令值 (仅同步机械臂关节, 跳过夹爪 index 19-22)"""
        for idx, var, _, _ in self.sliders:
            if idx >= 19:
                continue
            var.set(round(math.degrees(self.cmd_positions[idx]), 1))

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

        if self.mode == 'sim':
            # Sim模式：发送到Controller
            msg = Float64MultiArray()
            msg.data = targets
            self.target_pub.publish(msg)
            self.status_label.config(text='已发送', foreground='orange')
        else:
            # Real模式：直接发送rm_driver指令
            # A臂 (index 1-6)
            msg_a = Movej()
            msg_a.joint = [math.radians(targets[i]) for i in range(1, 7)]
            msg_a.speed = 20
            self.movej_pub_a.publish(msg_a)
            # B臂 (index 7-12)
            msg_b = Movej()
            msg_b.joint = [math.radians(targets[i]) for i in range(7, 13)]
            msg_b.speed = 20
            self.movej_pub_b.publish(msg_b)
            self.status_label.config(text='已发送(Real)', foreground='green')

    def _reset_all(self):
        for idx, entry, _, _ in self.entries:
            entry.delete(0, tk.END)
            entry.insert(0, '0.0')
        for idx, var, _, _ in self.sliders:
            var.set(0.0)


    def _send_world_pose(self, mode: str):
        """发送世界坐标系位姿命令"""
        try:
            vals = [float(e.get()) for e in self.world_pose_entries]
            speed = int(self.move_speed_entry.get())
        except ValueError:
            self.get_logger().error('位姿或速度输入无效')
            return
        arm = self.target_arm_var.get()
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = vals[0], vals[1], vals[2]
        # 欧拉角转四元数
        import transforms3d.euler as txe
        import transforms3d.quaternions as txq
        R = txe.euler2mat(vals[3], vals[4], vals[5], 'sxyz')
        q = txq.mat2quat(R)
        pose.orientation.w, pose.orientation.x = float(q[0]), float(q[1])
        pose.orientation.y, pose.orientation.z = float(q[2]), float(q[3])

        if mode == 'movel':
            msg = Movel(pose=pose, speed=speed)
            self.movel_pubs[arm].publish(msg)
        elif mode == 'movejp':
            msg = Movejp(pose=pose, speed=speed)
            self.movejp_pubs[arm].publish(msg)
        else:  # movep
            msg = Movel(pose=pose, speed=speed)
            self.movep_pubs[arm].publish(msg)
        self.world_pose_status.config(text='执行中...', foreground='orange')
        with self._world_pose_cmd_lock:
            self._world_pose_cmd_seq += 1
            self._pending_world_pose_cmd = {
                'seq': self._world_pose_cmd_seq,
                'arm': arm,
                'mode': mode,
                'target': list(vals),
                'sent_at': time.time(),
            }
        self.get_logger().info(
            f'{mode} → {arm}: '
            f'xyz=[{vals[0]:.3f},{vals[1]:.3f},{vals[2]:.3f}] '
            f'rpy=[{vals[3]:.3f},{vals[4]:.3f},{vals[5]:.3f}]')

    def _make_move_result_cb(self, arm: str, mode: str):
        def _cb(msg: Bool):
            self._on_move_result(arm, mode, msg)
        return _cb

    def _current_world_pose(self, arm: str):
        if not self.has_state:
            return None
        algo = self._algos.get(arm)
        if not algo or not algo.is_ready:
            return None

        arm_prefix = 'A' if arm == 'arm_a' else 'B'
        indices = [JOINT_NAMES_LIST.index(f'joint_platform_{arm_prefix}{i}')
                   for i in range(1, 7)]
        joints_deg = [math.degrees(self.current_positions[i]) for i in indices]
        d1_deg = math.degrees(self.current_positions[0])
        algo._robot.set_turntable_angle(-d1_deg)
        return algo.forward_kinematics(joints_deg, 1)

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        diff = a - b
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return abs(diff)

    def _is_pose_close(self, current, target,
                       pos_tol: float = 0.03,
                       rot_tol: float = 0.12) -> bool:
        pos_err = max(abs(current[i] - target[i]) for i in range(3))
        rot_err = max(self._angle_diff(current[i], target[i]) for i in range(3, 6))
        return pos_err <= pos_tol and rot_err <= rot_tol

    def _verify_move_result(self, expected_seq: int, arm: str, mode: str,
                            target: list, result_ok: bool):
        """延迟核对实际位姿，避免旧 result 把当前命令误判为失败。"""
        deadline = time.time() + (2.0 if result_ok else 6.0)
        poll_interval = 0.12

        if not result_ok:
            self.world_pose_status.config(
                text='执行中(收到失败结果，等待实际位姿确认)', foreground='orange')

        while time.time() < deadline:
            with self._world_pose_cmd_lock:
                pending = self._pending_world_pose_cmd
            if not pending or pending['seq'] != expected_seq:
                return

            current = self._current_world_pose(arm)
            if current is not None and self._is_pose_close(current, target):
                self.world_pose_status.config(text='执行成功', foreground='green')
                with self._world_pose_cmd_lock:
                    if (self._pending_world_pose_cmd and
                            self._pending_world_pose_cmd['seq'] == expected_seq):
                        self._pending_world_pose_cmd = None
                return
            time.sleep(poll_interval)

        with self._world_pose_cmd_lock:
            pending = self._pending_world_pose_cmd
        if not pending or pending['seq'] != expected_seq:
            return

        if result_ok:
            self.world_pose_status.config(text='执行中(等待状态收敛)', foreground='orange')
            return

        self.world_pose_status.config(
            text=(
                f'求解/执行失败 xyz=[{target[0]:.3f},{target[1]:.3f},{target[2]:.3f}] '
                f'rpy=[{target[3]:.2f},{target[4]:.2f},{target[5]:.2f}]'
            ),
            foreground='red')
        with self._world_pose_cmd_lock:
            if (self._pending_world_pose_cmd and
                    self._pending_world_pose_cmd['seq'] == expected_seq):
                self._pending_world_pose_cmd = None

    def _on_move_result(self, arm: str, mode: str, msg: Bool):
        """处理运动结果。GUI 只把与当前待执行命令匹配的结果用于状态展示。"""
        with self._world_pose_cmd_lock:
            pending = dict(self._pending_world_pose_cmd) if self._pending_world_pose_cmd else None
        if not pending:
            return
        if pending['arm'] != arm or pending['mode'] != mode:
            return

        threading.Thread(
            target=self._verify_move_result,
            args=(pending['seq'], arm, mode, pending['target'], msg.data),
            daemon=True
        ).start()

    def _read_current_pose(self):
        """从当前关节状态做正解，将世界坐标系位姿填入输入框"""
        if not self.has_state:
            self.world_pose_status.config(text='等待关节状态...', foreground='red')
            return

        arm = self.target_arm_var.get()
        algo = self._algos.get(arm)
        if not algo or not algo.is_ready:
            self.world_pose_status.config(text='Algo未就绪', foreground='red')
            return

        # 臂关节索引映射
        arm_joint_indices = {'arm_a': list(range(1, 7)), 'arm_b': list(range(7, 13))}
        indices = arm_joint_indices.get(arm, [])
        joints_deg = [math.degrees(self.current_positions[i]) for i in indices]

        # 设置 D1 转盘角度 (与 unified_arm_node 一致: 取反)
        d1_deg = math.degrees(self.current_positions[0])
        algo._robot.set_turntable_angle(-d1_deg)

        # FK → 世界坐标系位姿
        pose_fk = algo.forward_kinematics(joints_deg, 1)
        if pose_fk is None:
            self.world_pose_status.config(text='正解失败', foreground='red')
            return

        for i, e in enumerate(self.world_pose_entries):
            e.delete(0, tk.END)
            e.insert(0, f'{pose_fk[i]:.4f}')

        self.world_pose_status.config(
            text=f'已读取 {arm} 当前位姿 (D1={d1_deg:.1f}°)',
            foreground='green')
        self.get_logger().info(
            f'读取{arm}当前位姿: xyz=[{pose_fk[0]:.4f},{pose_fk[1]:.4f},{pose_fk[2]:.4f}] '
            f'rpy=[{pose_fk[3]:.4f},{pose_fk[4]:.4f},{pose_fk[5]:.4f}]')

    def _gen_random_reachable(self):
        """随机生成可达点 (通过FK) → 世界坐标系位姿

        使用 per-arm algo (带正确 base 参数 + D1 转盘角度),
        FK 直接输出世界坐标系位姿, 与 unified_arm_node 的 IK 变换链一致。
        """
        import random

        arm = self.target_arm_var.get()
        algo = self._algos.get(arm)
        if not algo or not algo.is_ready:
            self.world_pose_status.config(text='Algo未就绪', foreground='red')
            return

        # 设置 D1 转盘角度 (与 unified_arm_node.world_pose_to_joints 一致: 取反)
        d1_deg = math.degrees(self.current_positions[0]) if self.has_state else 0.0
        algo._robot.set_turntable_angle(-d1_deg)

        # 随机关节角度
        arm_prefix = 'A' if arm == 'arm_a' else 'B'
        arm_joints = [f'joint_platform_{arm_prefix}{i}' for i in range(1, 7)]
        joints_deg = []
        for name in arm_joints:
            lo, hi = JOINT_LIMITS[name]
            joints_deg.append(random.uniform(math.degrees(lo), math.degrees(hi)))

        # FK → 世界坐标系位姿 (RM65Robot 内部已处理 turntable + base 变换)
        pose_fk = algo.forward_kinematics(joints_deg, 1)
        if pose_fk is None:
            self.world_pose_status.config(text='FK失败', foreground='red')
            return
        self.get_logger().info(
            f'FK世界位姿(m): [{pose_fk[0]:.4f},{pose_fk[1]:.4f},{pose_fk[2]:.4f}]')

        # IK 验证 (用同一个 per-arm algo, 与 unified_arm_node 保持一致)
        ik_result = algo.inverse_kinematics(joints_deg, pose_fk, 1)
        ik_ok = ik_result is not None
        self.get_logger().info(f'IK验证: {"成功" if ik_ok else "失败"}')

        # 填入 GUI
        for i, e in enumerate(self.world_pose_entries):
            e.delete(0, tk.END)
            e.insert(0, f'{pose_fk[i]:.4f}')

        self.get_logger().info(f'随机关节: {[f"{d:.1f}" for d in joints_deg]}')
        status = '已生成' if ik_ok else '已生成(IK可能失败)'
        self.world_pose_status.config(
            text=status, foreground='green' if ik_ok else 'orange')

    def _base_to_world(self, arm: str, pose_Bi_T: list):
        """臂基座系位姿 → 世界坐标系位姿
        Args:
            pose_Bi_T: [x,y,z,rx,ry,rz] m+rad (臂基座系)
        Returns:
            [x,y,z,rx,ry,rz] m+rad (世界系), 失败返回None
        """
        algo = self._algos.get(arm, self._algo)

        # 从ROS参数获取base变换 (平台到臂基座)
        base_pos = list(self.get_parameter(f'{arm}.base_position').value)
        base_ori_deg = list(self.get_parameter(f'{arm}.base_orientation_deg').value)
        base_ori_rad = [math.radians(d) for d in base_ori_deg]
        pose_P_Bi = base_pos + base_ori_rad

        # 获取当前D1角度 (URDF中D1轴为负Z，需取反)
        d1_deg = math.degrees(self.current_positions[0]) if self.has_state else 0.0
        d1_rad = math.radians(-d1_deg)

        # pos2matrix (通过RealManAlgo封装)
        T_W_P = algo.pos2matrix([0, 0, 0, 0, 0, d1_rad])
        T_P_Bi = algo.pos2matrix(pose_P_Bi)
        T_Bi_T = algo.pos2matrix(pose_Bi_T)

        if T_W_P is None or T_P_Bi is None or T_Bi_T is None:
            self.get_logger().error(f'pos2matrix失败')
            return None

        # T_W_T = T_W_P @ T_P_Bi @ T_Bi_T
        T_W_Bi = T_W_P @ T_P_Bi
        T_W_T = T_W_Bi @ T_Bi_T

        # matrix2pos (通过RealManAlgo封装)
        pose_W_T = algo.matrix2pos(T_W_T, 1)
        if pose_W_T is None:
            self.get_logger().error(f'matrix2pos失败')
            return None

        return list(pose_W_T)

    def _send_base_pose(self):
        """直接发送臂基座坐标系位姿 (跳过世界坐标变换)"""
        try:
            vals = [float(e.get()) for e in self.world_pose_entries]
            speed = int(self.move_speed_entry.get())
        except ValueError:
            self.world_pose_status.config(text='输入无效', foreground='red')
            return
        arm = self.target_arm_var.get()
        # 直接用SDK的movej_p，不经过坐标变换
        from std_msgs.msg import Float64MultiArray
        # 发布到专用话题，让unified_arm_node直接执行IK
        if not hasattr(self, 'base_pose_pub'):
            self.base_pose_pub = self.create_publisher(
                Float64MultiArray, f'{arm}/base_pose_cmd', 10)
        msg = Float64MultiArray()
        msg.data = vals + [float(speed)]
        self.base_pose_pub.publish(msg)
        self.world_pose_status.config(text='已发送(基座系)', foreground='orange')
        self._last_target_pose = vals

    def _on_smooth_changed(self):
        """平滑处理勾选框变化"""
        msg = Bool()
        msg.data = self.smooth_var.get()
        self.smooth_pub.publish(msg)
        status = "启用" if msg.data else "禁用"
        self.get_logger().info(f'平滑处理已{status}')

    def _on_velocity_changed(self, event=None):
        """速度模式变化"""
        msg = String()
        msg.data = self.velocity_mode.get()
        self.velocity_mode_pub.publish(msg)
        self.get_logger().info(f'速度模式: {msg.data}')

    def _create_gripper_controls(self):
        """创建夹爪控制区域（在普通页面底部）"""
        gripper_frame = ttk.LabelFrame(self.root, text='夹爪控制', padding=10)
        gripper_frame.pack(fill='x', padx=5, pady=5)

        # 左夹爪控制
        left_frame = ttk.Frame(gripper_frame)
        left_frame.pack(side='left', padx=20)
        ttk.Label(left_frame, text='左夹爪:', font=('', 10, 'bold')).grid(row=0, column=0, columnspan=2, pady=5)

        # 左夹爪按钮
        btn_frame_left = ttk.Frame(left_frame)
        btn_frame_left.grid(row=1, column=0, columnspan=2, pady=5)
        ttk.Button(btn_frame_left, text='打开',
                   command=lambda: self._send_gripper_command('left_open')).pack(side='left', padx=2)
        ttk.Button(btn_frame_left, text='闭合',
                   command=lambda: self._send_gripper_command('left_close')).pack(side='left', padx=2)

        # 左夹爪角度设置
        ttk.Label(left_frame, text='打开角度:').grid(row=2, column=0, sticky='e', padx=5, pady=2)
        self.left_open_entry = ttk.Entry(left_frame, width=8)
        self.left_open_entry.insert(0, str(self.left_open_angle))
        self.left_open_entry.grid(row=2, column=1, padx=5, pady=2)

        ttk.Label(left_frame, text='闭合角度:').grid(row=3, column=0, sticky='e', padx=5, pady=2)
        self.left_close_entry = ttk.Entry(left_frame, width=8)
        self.left_close_entry.insert(0, str(self.left_close_angle))
        self.left_close_entry.grid(row=3, column=1, padx=5, pady=2)

        ttk.Button(left_frame, text='应用参数',
                   command=lambda: self._update_gripper_params('left')).grid(row=4, column=0, columnspan=2, pady=5)

        # 右夹爪控制
        right_frame = ttk.Frame(gripper_frame)
        right_frame.pack(side='left', padx=20)
        ttk.Label(right_frame, text='右夹爪:', font=('', 10, 'bold')).grid(row=0, column=0, columnspan=2, pady=5)

        # 右夹爪按钮
        btn_frame_right = ttk.Frame(right_frame)
        btn_frame_right.grid(row=1, column=0, columnspan=2, pady=5)
        ttk.Button(btn_frame_right, text='打开',
                   command=lambda: self._send_gripper_command('right_open')).pack(side='left', padx=2)
        ttk.Button(btn_frame_right, text='闭合',
                   command=lambda: self._send_gripper_command('right_close')).pack(side='left', padx=2)

        # 右夹爪角度设置
        ttk.Label(right_frame, text='打开角度:').grid(row=2, column=0, sticky='e', padx=5, pady=2)
        self.right_open_entry = ttk.Entry(right_frame, width=8)
        self.right_open_entry.insert(0, str(self.right_open_angle))
        self.right_open_entry.grid(row=2, column=1, padx=5, pady=2)

        ttk.Label(right_frame, text='闭合角度:').grid(row=3, column=0, sticky='e', padx=5, pady=2)
        self.right_close_entry = ttk.Entry(right_frame, width=8)
        self.right_close_entry.insert(0, str(self.right_close_angle))
        self.right_close_entry.grid(row=3, column=1, padx=5, pady=2)

        ttk.Button(right_frame, text='应用参数',
                   command=lambda: self._update_gripper_params('right')).grid(row=4, column=0, columnspan=2, pady=5)

    def _send_gripper_command(self, command: str):
        """发送夹爪控制指令"""
        msg = String()
        msg.data = command
        self.gripper_pub.publish(msg)
        self.get_logger().info(f'发送夹爪指令: {command}')

    def _load_gripper_params_from_controller(self):
        """从controller节点获取夹爪参数"""
        from rcl_interfaces.srv import GetParameters
        import time

        # 默认值
        self.left_open_angle = 0.0
        self.left_close_angle = -30.0
        self.right_open_angle = 0.0
        self.right_close_angle = -30.0

        try:
            client = self.create_client(GetParameters, '/triarm_controller/get_parameters')
            if not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn('controller参数服务不可用，使用默认夹爪参数')
                return

            # 先获取 mode 参数，决定用 sim_ 还是 real_ 前缀
            req_mode = GetParameters.Request()
            req_mode.names = ['mode']
            fut_mode = client.call_async(req_mode)
            start = time.time()
            while not fut_mode.done() and (time.time() - start) < 2.0:
                rclpy.spin_once(self, timeout_sec=0.01)

            mode_prefix = 'sim'
            if fut_mode.done() and fut_mode.result().values:
                mode_val = fut_mode.result().values[0].string_value
                mode_prefix = 'sim' if mode_val == 'sim' else 'real'

            # 用正确的 sim/real 前缀请求夹爪参数
            request = GetParameters.Request()
            request.names = [
                f'left_gripper.{mode_prefix}_open_angle',
                f'left_gripper.{mode_prefix}_close_angle',
                f'right_gripper.{mode_prefix}_open_angle',
                f'right_gripper.{mode_prefix}_close_angle',
            ]
            future = client.call_async(request)
            start = time.time()
            while not future.done() and (time.time() - start) < 2.0:
                rclpy.spin_once(self, timeout_sec=0.01)

            if future.done():
                response = future.result()
                if len(response.values) >= 4:
                    self.left_open_angle = response.values[0].double_value
                    self.left_close_angle = response.values[1].double_value
                    self.right_open_angle = response.values[2].double_value
                    self.right_close_angle = response.values[3].double_value
                    self.get_logger().info(
                        f'从controller加载夹爪参数({mode_prefix}): '
                        f'左开={self.left_open_angle}° 左闭={self.left_close_angle}°, '
                        f'右开={self.right_open_angle}° 右闭={self.right_close_angle}°')
        except Exception as e:
            self.get_logger().warn(f'获取controller参数失败: {e}，使用默认值')

    def _update_gripper_params(self, side: str):
        """更新夹爪参数（热加载）"""
        from rcl_interfaces.srv import SetParameters, GetParameters
        from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
        import time

        try:
            client_set = self.create_client(SetParameters, '/triarm_controller/set_parameters')
            client_get = self.create_client(GetParameters, '/triarm_controller/get_parameters')

            if not client_get.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('参数服务不可用')
                return

            # 获取 mode 以确定 sim/real 前缀
            req_mode = GetParameters.Request()
            req_mode.names = ['mode']
            fut = client_get.call_async(req_mode)
            start = time.time()
            while not fut.done() and (time.time() - start) < 1.0:
                rclpy.spin_once(self, timeout_sec=0.01)
            mode_prefix = 'sim'
            if fut.done() and fut.result().values:
                mode_val = fut.result().values[0].string_value
                mode_prefix = 'sim' if mode_val == 'sim' else 'real'

            if side == 'left':
                open_angle = float(self.left_open_entry.get())
                close_angle = float(self.left_close_entry.get())
                params = [
                    Parameter(
                        name=f'left_gripper.{mode_prefix}_open_angle',
                        value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=open_angle)
                    ),
                    Parameter(
                        name=f'left_gripper.{mode_prefix}_close_angle',
                        value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=close_angle)
                    )
                ]
                self.get_logger().info(f'更新左夹爪参数({mode_prefix}): 打开={open_angle}°, 闭合={close_angle}°')

            elif side == 'right':
                open_angle = float(self.right_open_entry.get())
                close_angle = float(self.right_close_entry.get())
                params = [
                    Parameter(
                        name=f'right_gripper.{mode_prefix}_open_angle',
                        value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=open_angle)
                    ),
                    Parameter(
                        name=f'right_gripper.{mode_prefix}_close_angle',
                        value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=close_angle)
                    )
                ]
                self.get_logger().info(f'更新右夹爪参数({mode_prefix}): 打开={open_angle}°, 闭合={close_angle}°')
            else:
                return

            if not client_set.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('set_parameters服务不可用')
                return

            request = SetParameters.Request()
            request.parameters = params
            future = client_set.call_async(request)
            start = time.time()
            while not future.done() and (time.time() - start) < 1.0:
                rclpy.spin_once(self, timeout_sec=0.01)

            if future.done():
                response = future.result()
                if response.results[0].successful:
                    self.get_logger().info('参数更新成功')
                else:
                    self.get_logger().warn(f'参数更新失败: {response.results[0].reason}')

        except ValueError:
            self.get_logger().error('输入的角度值无效')
        except Exception as e:
            self.get_logger().error(f'更新参数时出错: {str(e)}')

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
