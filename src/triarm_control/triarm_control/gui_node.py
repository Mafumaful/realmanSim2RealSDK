"""
关节控制GUI模块

独立的GUI界面，通过ROS2话题与控制节点通信
支持23关节体系 (19臂关节 + 4夹爪关节)
"""

import math
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

from rm_ros_interfaces.msg import Movel, Movejp
from geometry_msgs.msg import Pose
from .joint_names import JOINT_NAMES_LIST, JOINT_LIMITS, TOTAL_JOINT_COUNT
from .realman_sdk_wrapper import RealManAlgo, matrix_multiply


class JointControlGUI(Node):
    """关节控制GUI节点"""

    def __init__(self):
        super().__init__('triarm_gui')

        # 声明参数
        self.declare_parameter('namespace', 'robot')
        self.declare_parameter('gui_width', 700)
        self.declare_parameter('gui_height', 600)

        # 获取参数
        ns = self.get_parameter('namespace').value
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

        # 发布夹爪目标 (供 gripper_bridge real模式 和 unified_arm_node sim模式 使用)
        gripper_topic = f'{prefix}gripper_target'
        self.gripper_target_pub = self.create_publisher(Float64MultiArray, gripper_topic, 10)

        # 发布控制参数（用于动态修改）
        self.smooth_pub = self.create_publisher(Bool, f'{prefix}enable_smooth', 10)
        self.velocity_mode_pub = self.create_publisher(String, f'{prefix}velocity_mode', 10)

        # 发布夹爪控制指令
        self.gripper_pub = self.create_publisher(String, f'{prefix}gripper_control', 10)

        # 发布 MoveL/MoveJP 命令
        self.movel_pubs = {
            'arm_a': self.create_publisher(Movel, 'arm_a/rm_driver/movel_cmd', 10),
            'arm_b': self.create_publisher(Movel, 'arm_b/rm_driver/movel_cmd', 10),
        }
        self.movejp_pubs = {
            'arm_a': self.create_publisher(Movejp, 'arm_a/rm_driver/movej_p_cmd', 10),
            'arm_b': self.create_publisher(Movejp, 'arm_b/rm_driver/movej_p_cmd', 10),
        }

        # 订阅运动结果
        for arm in ['arm_a', 'arm_b']:
            self.create_subscription(Bool, f'{arm}/rm_driver/movel_result', self._on_move_result, 10)
            self.create_subscription(Bool, f'{arm}/rm_driver/movej_p_result', self._on_move_result, 10)

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
        self._create_calibration_tab(notebook)
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

    def _create_calibration_tab(self, notebook):
        """创建坐标标定标签页"""
        frame = ttk.Frame(notebook)
        notebook.add(frame, text='坐标标定')

        # 标定参数输入
        self.calib_entries = {}
        arms = [('arm_a', 'A臂'), ('arm_b', 'B臂'), ('arm_s', 'S臂')]
        labels = ['x(m)', 'y(m)', 'z(m)', 'rx(rad)', 'ry(rad)', 'rz(rad)']

        for row, (arm_key, arm_name) in enumerate(arms):
            ttk.Label(frame, text=arm_name, font=('', 10, 'bold')).grid(
                row=row*2, column=0, padx=5, pady=(10,0), sticky='w')
            entry_frame = ttk.Frame(frame)
            entry_frame.grid(row=row*2+1, column=0, columnspan=7, padx=5, pady=2, sticky='w')
            self.calib_entries[arm_key] = []
            for col, lbl in enumerate(labels):
                ttk.Label(entry_frame, text=lbl).grid(row=0, column=col*2, padx=2)
                e = ttk.Entry(entry_frame, width=8)
                e.insert(0, '0.0')
                e.grid(row=0, column=col*2+1, padx=2)
                self.calib_entries[arm_key].append(e)

        # 按钮
        btn_frame = ttk.Frame(frame)
        btn_frame.grid(row=7, column=0, columnspan=7, pady=15)
        ttk.Button(btn_frame, text='加载当前参数', command=self._load_calib_params).pack(side='left', padx=5)
        ttk.Button(btn_frame, text='应用参数', command=self._apply_calib_params).pack(side='left', padx=5)

        # 启动时从配置文件加载
        self._load_calib_from_config()

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
        ttk.Button(btn_frame, text='MoveL', command=lambda: self._send_world_pose('movel')).pack(side='left', padx=10)
        ttk.Button(btn_frame, text='MoveJP', command=lambda: self._send_world_pose('movejp')).pack(side='left', padx=10)
        ttk.Button(btn_frame, text='随机可达点', command=self._gen_random_reachable).pack(side='left', padx=10)
        ttk.Button(btn_frame, text='直接执行(基座系)', command=self._send_base_pose).pack(side='left', padx=10)

        # 状态标签
        self.world_pose_status = ttk.Label(frame, text='就绪', foreground='green')
        self.world_pose_status.grid(row=5, column=0, columnspan=6, pady=10)

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

    def _load_calib_params(self):
        """从unified_arm_node加载标定参数"""
        from rcl_interfaces.srv import GetParameters
        client = self.create_client(GetParameters, '/unified_arm_node/get_parameters')
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('unified_arm_node参数服务不可用')
            return
        request = GetParameters.Request()
        request.names = ['pose_P_arm_a', 'pose_P_arm_b', 'pose_P_arm_s']
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if future.done():
            resp = future.result()
            for i, arm in enumerate(['arm_a', 'arm_b', 'arm_s']):
                if i < len(resp.values) and resp.values[i].double_array_value:
                    vals = resp.values[i].double_array_value
                    for j, e in enumerate(self.calib_entries[arm]):
                        e.delete(0, tk.END)
                        e.insert(0, f'{vals[j]:.4f}' if j < len(vals) else '0.0')
            self.get_logger().info('标定参数已加载')

    def _load_calib_from_config(self):
        """从配置文件加载标定参数"""
        import os
        import yaml
        config_path = os.path.join(
            os.path.dirname(__file__), '..', 'config', 'triarm_config.yaml')
        try:
            with open(config_path, 'r') as f:
                cfg = yaml.safe_load(f)
            params = cfg.get('unified_arm_node', {}).get('ros__parameters', {})
            for arm in ['arm_a', 'arm_b', 'arm_s']:
                vals = params.get(f'pose_P_{arm}', [0.0]*6)
                for j, e in enumerate(self.calib_entries[arm]):
                    e.delete(0, tk.END)
                    e.insert(0, f'{vals[j]:.4f}' if j < len(vals) else '0.0')
            self.get_logger().info(f'从配置文件加载标定参数: {config_path}')
        except Exception as e:
            self.get_logger().warn(f'加载配置文件失败: {e}')

    def _apply_calib_params(self):
        """应用标定参数到unified_arm_node"""
        from rcl_interfaces.srv import SetParameters
        from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
        client = self.create_client(SetParameters, '/unified_arm_node/set_parameters')
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('unified_arm_node参数服务不可用')
            return
        params = []
        for arm in ['arm_a', 'arm_b', 'arm_s']:
            vals = [float(e.get()) for e in self.calib_entries[arm]]
            params.append(Parameter(
                name=f'pose_P_{arm}',
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=vals)))
        request = SetParameters.Request(parameters=params)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        self.get_logger().info('标定参数已应用')

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
        else:
            msg = Movejp(pose=pose, speed=speed)
            self.movejp_pubs[arm].publish(msg)
        self.world_pose_status.config(text='执行中...', foreground='orange')
        self._last_target_pose = vals  # 记录目标位姿
        self.get_logger().info(f'{mode} → {arm}: [{vals[0]:.3f},{vals[1]:.3f},{vals[2]:.3f}]')

    def _on_move_result(self, msg: Bool):
        """处理运动结果"""
        if msg.data:
            self.world_pose_status.config(text='执行成功', foreground='green')
        else:
            vals = getattr(self, '_last_target_pose', [0]*6)
            self.world_pose_status.config(
                text=f'求解失败 xyz=[{vals[0]:.3f},{vals[1]:.3f},{vals[2]:.3f}] rpy=[{vals[3]:.2f},{vals[4]:.2f},{vals[5]:.2f}]',
                foreground='red')

    def _gen_random_reachable(self):
        """随机生成可达点 (通过FK) → 转换到世界坐标系"""
        import random
        from Robotic_Arm.rm_robot_interface import Algo, rm_robot_arm_model_e, rm_force_type_e
        try:
            algo = Algo(rm_robot_arm_model_e.RM_MODEL_RM_65_E, rm_force_type_e.RM_MODEL_RM_B_E)
        except Exception as e:
            self.world_pose_status.config(text=f'Algo初始化失败: {e}', foreground='red')
            return

        # 根据选择的臂获取关节限位
        arm = self.target_arm_var.get()
        arm_prefix = 'A' if arm == 'arm_a' else 'B'
        arm_joints = [f'joint_platform_{arm_prefix}{i}' for i in range(1, 7)]
        joints_deg = []
        for name in arm_joints:
            lo, hi = JOINT_LIMITS[name]
            joints_deg.append(random.uniform(math.degrees(lo), math.degrees(hi)))

        # FK: 关节角度 → 臂基座系位姿 (mm, rad)
        ret = algo.rm_algo_forward_kinematics(joints_deg, 1)
        if isinstance(ret, (list, tuple)) and len(ret) == 6:
            pose_Bi_T = ret
        elif isinstance(ret, (list, tuple)) and len(ret) >= 2 and ret[0] == 0:
            pose_Bi_T = ret[1]
        else:
            self.world_pose_status.config(text=f'FK失败: {ret}', foreground='red')
            return

        # 臂基座系 → 世界坐标系变换
        pose_W_T = self._base_to_world(algo, arm, pose_Bi_T)
        if pose_W_T is None:
            self.world_pose_status.config(text='坐标变换失败', foreground='red')
            return

        for i, e in enumerate(self.world_pose_entries):
            e.delete(0, tk.END)
            e.insert(0, f'{pose_W_T[i]:.4f}')
        self.world_pose_status.config(text='已生成 (世界坐标系)', foreground='green')

    def _base_to_world(self, algo, arm: str, pose_Bi_T: list):
        """臂基座系位姿 → 世界坐标系位姿
        Args:
            pose_Bi_T: [x,y,z,rx,ry,rz] mm+rad (臂基座系)
        Returns:
            [x,y,z,rx,ry,rz] m+rad (世界系), 失败返回None
        """
        # 从标定参数获取变换
        pose_P_Bi = [float(e.get()) for e in self.calib_entries[arm]]
        pose_P_Bi_mm = [pose_P_Bi[0]*1000, pose_P_Bi[1]*1000, pose_P_Bi[2]*1000,
                       pose_P_Bi[3], pose_P_Bi[4], pose_P_Bi[5]]

        # D1=0时，T_W_P = I (假设世界系与平台零位重合)
        # T_W_Bi = T_P_Bi
        ret = algo.rm_algo_pos2matrix(pose_P_Bi_mm)
        if not (isinstance(ret, (list, tuple)) and ret[0] == 0):
            return None
        T_W_Bi = ret[1]

        ret = algo.rm_algo_pos2matrix(pose_Bi_T)
        if not (isinstance(ret, (list, tuple)) and ret[0] == 0):
            return None
        T_Bi_T = ret[1]

        # T_W_T = T_W_Bi × T_Bi_T
        T_W_T = matrix_multiply(T_W_Bi, T_Bi_T)

        ret = algo.rm_algo_matrix2pos(T_W_T, 1)
        if not (isinstance(ret, (list, tuple)) and ret[0] == 0):
            return None
        pose_W_T_mm = ret[1]

        # mm → m
        return [pose_W_T_mm[0]/1000, pose_W_T_mm[1]/1000, pose_W_T_mm[2]/1000,
                pose_W_T_mm[3], pose_W_T_mm[4], pose_W_T_mm[5]]

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

            request = GetParameters.Request()
            request.names = [
                'left_gripper.open_angle',
                'left_gripper.close_angle',
                'right_gripper.open_angle',
                'right_gripper.close_angle'
            ]

            future = client.call_async(request)

            # 等待结果
            import time
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
                    self.get_logger().info(f'从controller加载夹爪参数: 左={self.left_close_angle}°, 右={self.right_close_angle}°')
        except Exception as e:
            self.get_logger().warn(f'获取controller参数失败: {e}，使用默认值')

    def _update_gripper_params(self, side: str):
        """更新夹爪参数（热加载）"""
        from rcl_interfaces.srv import SetParameters
        from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

        try:
            if side == 'left':
                open_angle = float(self.left_open_entry.get())
                close_angle = float(self.left_close_entry.get())

                params = [
                    Parameter(
                        name='left_gripper.open_angle',
                        value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=open_angle)
                    ),
                    Parameter(
                        name='left_gripper.close_angle',
                        value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=close_angle)
                    )
                ]
                self.get_logger().info(f'更新左夹爪参数: 打开={open_angle}°, 闭合={close_angle}°')

            elif side == 'right':
                open_angle = float(self.right_open_entry.get())
                close_angle = float(self.right_close_entry.get())

                params = [
                    Parameter(
                        name='right_gripper.open_angle',
                        value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=open_angle)
                    ),
                    Parameter(
                        name='right_gripper.close_angle',
                        value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=close_angle)
                    )
                ]
                self.get_logger().info(f'更新右夹爪参数: 打开={open_angle}°, 闭合={close_angle}°')

            # 创建服务客户端
            client = self.create_client(SetParameters, '/triarm_controller/set_parameters')

            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('参数服务不可用')
                return

            # 发送请求
            request = SetParameters.Request()
            request.parameters = params
            future = client.call_async(request)

            # 简单等待结果
            import time
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
