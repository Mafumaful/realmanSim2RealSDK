#!/usr/bin/env python3
"""
统一机械臂节点 - rm_driver 兼容替代品

为 A/B 臂各创建一套 rm_driver 兼容话题：
  {arm_name}/rm_driver/movel_cmd → movel_result
  {arm_name}/rm_driver/movej_cmd → movej_result
  {arm_name}/rm_driver/movej_p_cmd → movej_p_result
  {arm_name}/rm_driver/move_stop_cmd
  {arm_name}/rm_driver/udp_arm_position (Pose反馈)
  {arm_name}/joint_states (JointState反馈)

双模式 (由 RealManSDKWrapper 内部路由):
  sim: Pose → SDK Algo IK → 关节角度 → 映射到23关节数组 → 现有插值逻辑
  real: Pose → SDK TCP 直接控制 RM65

23关节体系:
  [0]     D1        (底盘)
  [1-6]   A1-A6     (左臂)
  [7-12]  B1-B6     (右臂)
  [13-18] S1-S6     (头部)
  [19-20] L1,L11    (左夹爪, 由 /gripper_target 更新)
  [21-22] R1,R11    (右夹爪, 由 /gripper_target 更新)
"""

import math
import time
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Float64, Float64MultiArray
from rm_ros_interfaces.msg import Movel, Movej, Movejp

import transforms3d.quaternions as txq
import transforms3d.euler as txe

from .realman_sdk_wrapper import RealManSDKWrapper, SDKMotionResult
from .joint_names import (JOINT_NAMES_LIST, TOTAL_JOINT_COUNT,
                          ARM_JOINT_COUNT, GRIPPER_JOINT_INDICES)


# 臂关节在23关节数组中的索引映射
ARM_JOINT_INDICES = {
    'arm_a': list(range(1, 7)),    # A臂: index 1-6
    'arm_b': list(range(7, 13)),   # B臂: index 7-12
    'arm_s': list(range(13, 19)),  # S臂: index 13-18 (预留)
}

# 臂配置默认值
ARM_DEFAULTS = {
    'arm_a': {'ip': '192.168.1.18', 'port': 8080, 'id': 'A'},
    'arm_b': {'ip': '192.168.1.19', 'port': 8080, 'id': 'B'},
    'arm_s': {'ip': '192.168.1.20', 'port': 8080, 'id': 'S'},
}

# ═══════════════════════════════════════════════════════════
# 臂基座配置 (基于 URDF joint_platform_A1/B1 标定)
# 用于世界坐标 → 臂基座坐标变换
# ═══════════════════════════════════════════════════════════
ARM_BASE_MAP = {
    'rotation_center': np.array([0.00043, 0.0004, -0.39995]),
    'arm_a': {
        'offset': np.array([0.15605, 0.14202, 0.17264]),
        'yaw_deg': -47.372,
        'roll_rad': -math.pi / 2,
    },
    'arm_b': {
        'offset': np.array([-0.14289, 0.15524, 0.17264]),
        'yaw_deg': 42.628,
        'roll_rad': -math.pi / 2,
    },
}


def _world_to_arm_base(x, y, z, rx, ry, rz, arm_key, d1_deg):
    """世界坐标 → 臂基座坐标 (含 -90° roll 补偿)

    Args:
        x, y, z: 世界系位置 (米)
        rx, ry, rz: 世界系姿态欧拉角 (弧度, sxyz)
        arm_key: 'arm_a' 或 'arm_b'
        d1_deg: D1 底盘当前角度 (度)

    Returns:
        (ax, ay, az, arx, ary, arz) 臂基座系下的位姿
    """
    cfg = ARM_BASE_MAP[arm_key]
    center = ARM_BASE_MAP['rotation_center']

    # 臂基座在世界系的位置
    theta_d1 = np.radians(d1_deg)
    c1, s1 = np.cos(theta_d1), np.sin(theta_d1)
    R_d1 = np.array([[c1, -s1, 0], [s1, c1, 0], [0, 0, 1]])
    arm_pos = center + R_d1 @ cfg['offset']

    # 臂基座完整旋转: Rz(yaw+d1) @ Rx(roll)
    total_yaw = np.radians(cfg['yaw_deg'] + d1_deg)
    cy, sy = np.cos(total_yaw), np.sin(total_yaw)
    R_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])

    cr, sr = np.cos(cfg['roll_rad']), np.sin(cfg['roll_rad'])
    R_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])

    R_arm = R_z @ R_x  # 世界→臂基座的旋转

    # 位置变换
    delta = np.array([x, y, z]) - arm_pos
    pos_arm = R_arm.T @ delta

    # 姿态变换
    R_target = txe.euler2mat(rx, ry, rz, 'sxyz')
    R_in_arm = R_arm.T @ R_target
    rot_arm = txe.mat2euler(R_in_arm, 'sxyz')

    return (*pos_arm, *rot_arm)


def pose_to_xyzrpy(pose: Pose):
    """将 geometry_msgs/Pose 转为 (x,y,z,rx,ry,rz) 欧拉角"""
    x = pose.position.x
    y = pose.position.y
    z = pose.position.z
    q = [pose.orientation.w, pose.orientation.x,
         pose.orientation.y, pose.orientation.z]
    R = txq.quat2mat(q)
    rx, ry, rz = txe.mat2euler(R, 'sxyz')
    return x, y, z, rx, ry, rz


class ArmBridge:
    """单臂桥接器 - 管理一个臂的话题和SDK实例"""

    def __init__(self, node: Node, arm_name: str, mode: str,
                 ip: str, port: int, arm_id: str,
                 sim_joint_tol: float = 0.02,
                 sim_motion_timeout: float = 10.0):
        self.node = node
        self.arm_name = arm_name
        self.mode = mode
        self.logger = node.get_logger()
        self._tag = f'[ArmBridge:{arm_name}]'
        self._joint_indices = ARM_JOINT_INDICES.get(arm_name, [])
        self._sim_joint_tol = sim_joint_tol
        self._sim_motion_timeout = sim_motion_timeout

        # SDK wrapper (模式感知，内部自动路由 sim/real)
        self._sdk = RealManSDKWrapper(ip, port, arm_id, mode=mode)

        # 当前状态
        self._current_joints = [0.0] * 6  # 6关节弧度
        self._current_pose = Pose()
        self._lock = threading.Lock()

        # 运动命令串行化锁 (同一时刻只允许一个运动执行)
        self._motion_lock = threading.Lock()
        # sim模式运动取消信号
        self._stop_event = threading.Event()
        # D1角度获取回调 (由 UnifiedArmNode 设置)
        self._get_d1_angle = None

        prefix = f'{arm_name}/'

        # ── 发布器: rm_driver 兼容结果话题 ──
        self._movel_result_pub = node.create_publisher(
            Bool, f'{prefix}rm_driver/movel_result', 10)
        self._movej_result_pub = node.create_publisher(
            Bool, f'{prefix}rm_driver/movej_result', 10)
        self._movejp_result_pub = node.create_publisher(
            Bool, f'{prefix}rm_driver/movej_p_result', 10)

        # 状态反馈
        self._arm_pos_pub = node.create_publisher(
            Pose, f'{prefix}rm_driver/udp_arm_position', 10)
        self._joint_state_pub = node.create_publisher(
            JointState, f'{prefix}joint_states', 10)

        # ── 订阅器: rm_driver 兼容命令话题 ──
        self._movel_sub = node.create_subscription(
            Movel, f'{prefix}rm_driver/movel_cmd',
            self._on_movel, 10)
        self._movej_sub = node.create_subscription(
            Movej, f'{prefix}rm_driver/movej_cmd',
            self._on_movej, 10)
        self._movejp_sub = node.create_subscription(
            Movejp, f'{prefix}rm_driver/movej_p_cmd',
            self._on_movejp, 10)
        self._stop_sub = node.create_subscription(
            Empty, f'{prefix}rm_driver/move_stop_cmd',
            self._on_stop, 10)

        # sim模式: 共享目标发布回调 (由外部设置)
        self._publish_shared_target = None

        self.logger.info(
            f'{self._tag} 初始化完成 (mode={mode}, ip={ip}:{port})')

    def set_publish_target_fn(self, fn):
        """设置共享目标发布回调 (sim模式)
        fn(indices, joints_rad) → 更新共享23关节数组并发布
        """
        self._publish_shared_target = fn

    def set_d1_angle_fn(self, fn):
        """设置D1角度获取回调: fn() → float (度)"""
        self._get_d1_angle = fn

    def connect_sdk(self) -> bool:
        ok = self._sdk.connect()
        if ok:
            self.logger.info(f'{self._tag} SDK就绪')
        else:
            self.logger.error(f'{self._tag} SDK连接失败!')
        return ok

    def disconnect_sdk(self):
        self._sdk.disconnect()

    # ─── 命令回调 ───

    def _on_movel(self, msg: Movel):
        threading.Thread(
            target=self._exec_movel, args=(msg,), daemon=True).start()

    def _on_movej(self, msg: Movej):
        threading.Thread(
            target=self._exec_movej, args=(msg,), daemon=True).start()

    def _on_movejp(self, msg: Movejp):
        threading.Thread(
            target=self._exec_movejp, args=(msg,), daemon=True).start()

    def _on_stop(self, msg: Empty):
        self.logger.warn(f'{self._tag} 收到停止命令')
        self._stop_event.set()  # 取消 sim 模式等待
        if self.mode == 'real':
            self._sdk.stop()

    # ─── 执行逻辑 ───

    def _exec_movel(self, msg: Movel):
        with self._motion_lock:
            self._stop_event.clear()
            x, y, z, rx, ry, rz = pose_to_xyzrpy(msg.pose)
            speed = msg.speed
            if self.mode == 'real':
                success = self._real_movel(x, y, z, rx, ry, rz, speed)
            else:
                success = self._sim_move_to_pose(x, y, z, rx, ry, rz)
        result = Bool()
        result.data = success
        self._movel_result_pub.publish(result)

    def _exec_movej(self, msg: Movej):
        with self._motion_lock:
            self._stop_event.clear()
            joints = list(msg.joint)
            speed = msg.speed
            if self.mode == 'real':
                success = self._real_movej(joints, speed)
            else:
                success = self._sim_move_joints(joints)
        result = Bool()
        result.data = success
        self._movej_result_pub.publish(result)

    def _exec_movejp(self, msg: Movejp):
        with self._motion_lock:
            self._stop_event.clear()
            x, y, z, rx, ry, rz = pose_to_xyzrpy(msg.pose)
            speed = msg.speed
            if self.mode == 'real':
                success = self._real_movejp(x, y, z, rx, ry, rz, speed)
            else:
                success = self._sim_move_to_pose(x, y, z, rx, ry, rz)
        result = Bool()
        result.data = success
        self._movejp_result_pub.publish(result)

    # ─── Real 模式 ───

    def _real_movel(self, x, y, z, rx, ry, rz, speed) -> bool:
        ret = self._sdk.movel(x, y, z, rx, ry, rz, speed)
        return ret == SDKMotionResult.SUCCESS

    def _real_movej(self, joints, speed) -> bool:
        ret = self._sdk.movej(joints, speed)
        return ret == SDKMotionResult.SUCCESS

    def _real_movejp(self, x, y, z, rx, ry, rz, speed) -> bool:
        ret = self._sdk.movej_p(x, y, z, rx, ry, rz, speed)
        return ret == SDKMotionResult.SUCCESS

    # ─── Sim 模式 ───

    def _sim_move_to_pose(self, x, y, z, rx, ry, rz) -> bool:
        """sim模式: 臂基座Pose → SDK IK → 关节角度 → /target_joints"""
        if not self._sdk.is_ready:
            self.logger.error(f'{self._tag} SDK未就绪，无法执行IK')
            return False

        self.logger.info(
            f'{self._tag} '
            f'臂基座=[{x:.4f},{y:.4f},{z:.4f}]')

        with self._lock:
            q_ref = list(self._current_joints)

        joints = self._sdk.inverse_kinematics(
            x, y, z, rx, ry, rz, q_ref=q_ref)

        if joints is None:
            self.logger.warn(
                f'{self._tag} IK求解失败 '
                f'(臂基座=[{x:.3f},{y:.3f},{z:.3f},{rx:.3f},{ry:.3f},{rz:.3f}])')
            return False

        return self._sim_move_joints(joints)

    def _sim_move_joints(self, joints) -> bool:
        """sim模式: 关节角度 → 更新共享目标数组 → /target_joints → 等待到位"""
        if not self._publish_shared_target:
            self.logger.error(f'{self._tag} 共享目标发布回调未设置')
            return False

        if len(self._joint_indices) != len(joints):
            self.logger.error(
                f'{self._tag} 关节数不匹配: '
                f'indices={len(self._joint_indices)}, joints={len(joints)}')
            return False

        self._publish_shared_target(self._joint_indices, joints)

        self.logger.info(
            f'{self._tag} sim发送关节目标: '
            f'{[f"{math.degrees(j):.1f}" for j in joints]}')

        # 轮询等待关节到位 (可被 stop_event 取消)
        target = list(joints)
        start = time.time()
        while time.time() - start < self._sim_motion_timeout:
            if self._stop_event.is_set():
                self.logger.warn(f'{self._tag} sim运动被取消')
                return False
            time.sleep(0.05)
            with self._lock:
                current = list(self._current_joints)
            max_err = max(abs(c - t) for c, t in zip(current, target))
            if max_err < self._sim_joint_tol:
                self.logger.info(
                    f'{self._tag} sim关节到位 (max_err={max_err:.4f} rad)')
                return True

        self.logger.warn(
            f'{self._tag} sim运动超时 ({self._sim_motion_timeout}s)')
        return False

    # ─── 状态反馈 ───

    @staticmethod
    def _dict_to_pose(pose_dict) -> Pose:
        """将 {x,y,z,rx,ry,rz} 字典转为 Pose 消息"""
        pose = Pose()
        pose.position.x = pose_dict['x']
        pose.position.y = pose_dict['y']
        pose.position.z = pose_dict['z']
        R = txe.euler2mat(
            pose_dict['rx'], pose_dict['ry'],
            pose_dict['rz'], 'sxyz')
        q = txq.mat2quat(R)
        pose.orientation.w = float(q[0])
        pose.orientation.x = float(q[1])
        pose.orientation.y = float(q[2])
        pose.orientation.z = float(q[3])
        return pose

    def publish_state(self):
        """发布当前状态 (由定时器调用)"""
        if self.mode == 'real' and self._sdk.is_connected:
            joints = self._sdk.get_joint_positions()
            if joints:
                with self._lock:
                    self._current_joints = joints

            pose_dict = self._sdk.get_current_pose()
            if pose_dict:
                pose = self._dict_to_pose(pose_dict)
                with self._lock:
                    self._current_pose = pose

        elif self.mode == 'sim' and self._sdk.is_ready:
            with self._lock:
                joints_copy = list(self._current_joints)
            pose_dict = self._sdk.forward_kinematics(joints_copy)
            if pose_dict:
                pose = self._dict_to_pose(pose_dict)
                with self._lock:
                    self._current_pose = pose

        js_msg = JointState()
        js_msg.header.stamp = self.node.get_clock().now().to_msg()
        with self._lock:
            js_msg.name = [JOINT_NAMES_LIST[idx]
                           for idx in self._joint_indices]
            js_msg.position = list(self._current_joints)
        self._joint_state_pub.publish(js_msg)

        with self._lock:
            self._arm_pos_pub.publish(self._current_pose)

    def update_joints_from_sim(self, joint_positions: list):
        """从 /joint_states (Isaac Sim) 更新臂关节状态"""
        with self._lock:
            for i, idx in enumerate(self._joint_indices):
                if idx < len(joint_positions):
                    self._current_joints[i] = joint_positions[idx]


class UnifiedArmNode(Node):
    """统一机械臂节点 - 管理所有臂的桥接"""

    def __init__(self):
        super().__init__('unified_arm_node')

        # 参数声明
        self.declare_parameter('mode', 'sim')
        self.declare_parameter('arm_a.ip', '192.168.1.18')
        self.declare_parameter('arm_a.port', 8080)
        self.declare_parameter('arm_b.ip', '192.168.1.19')
        self.declare_parameter('arm_b.port', 8080)
        self.declare_parameter('arm_s.ip', '192.168.1.20')
        self.declare_parameter('arm_s.port', 8080)
        self.declare_parameter('namespace', 'robot')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('sim_joint_tolerance', 0.02)
        self.declare_parameter('sim_motion_timeout', 10.0)

        mode = self.get_parameter('mode').value
        ns = self.get_parameter('namespace').value
        rate = self.get_parameter('publish_rate').value
        self._sim_joint_tol = self.get_parameter('sim_joint_tolerance').value
        self._sim_motion_timeout = self.get_parameter('sim_motion_timeout').value

        self.get_logger().info(f'=== UnifiedArmNode 启动 (mode={mode}) ===')

        # sim模式: /target_joints 发布器 (共享) + 底盘角度桥接
        prefix = f'{ns}/' if ns else '/'
        self._mode = mode
        self._target_joints_pub = None
        self._base_angle_pub = None

        # 共享23关节目标数组 (角度制, sim模式核心状态)
        self._shared_target = [0.0] * TOTAL_JOINT_COUNT
        self._shared_target_lock = threading.Lock()
        self._d1_lock = threading.Lock()
        self._current_d1_angle = 0.0

        if mode == 'sim':
            self._target_joints_pub = self.create_publisher(
                Float64MultiArray, f'{prefix}target_joints', 10)
            self._base_angle_pub = self.create_publisher(
                Float64, '/base_controller/current_angle', 10)

            # D1旋转: 订阅 rotate_cmd, 发布 rotate_result
            self._rotate_cmd_sub = self.create_subscription(
                Float64, '/base_controller/rotate_cmd',
                self._on_rotate_cmd, 10)
            self._rotate_result_pub = self.create_publisher(
                Bool, '/base_controller/rotate_result', 10)

            # 订阅 /gripper_target (来自 GripperController 或 gui_node)
            self._gripper_target_sub = self.create_subscription(
                Float64MultiArray, f'{prefix}gripper_target',
                self._on_gripper_target, 10)

        # 创建 A/B 臂桥接器
        self._bridges = {}
        for arm_name in ['arm_a', 'arm_b']:
            ip = self.get_parameter(f'{arm_name}.ip').value
            port = self.get_parameter(f'{arm_name}.port').value
            arm_id = arm_name[-1].upper()

            bridge = ArmBridge(
                self, arm_name, mode, ip, port, arm_id,
                sim_joint_tol=self._sim_joint_tol,
                sim_motion_timeout=self._sim_motion_timeout)
            if mode == 'sim':
                bridge.set_publish_target_fn(self._update_and_publish_target)
                bridge.set_d1_angle_fn(self._get_current_d1)
            bridge.connect_sdk()
            self._bridges[arm_name] = bridge

        self.get_logger().info('S臂接口预留，暂未启用')

        # sim模式: 订阅 /joint_states 获取仿真反馈
        if mode == 'sim':
            self._sim_state_sub = self.create_subscription(
                JointState, f'{prefix}joint_states',
                self._sim_state_cb, 10)

        # 状态发布定时器
        period = 1.0 / rate
        self._state_timer = self.create_timer(
            period, self._publish_states)

        self.get_logger().info('=== UnifiedArmNode 就绪 ===')

    def _sim_state_cb(self, msg: JointState):
        """从 Isaac Sim /joint_states 更新所有关节状态 (含夹爪)"""
        # 按名称映射，兼容 Isaac Sim 任意关节顺序
        name_to_pos = dict(zip(msg.name, msg.position))
        positions = [name_to_pos.get(n, 0.0) for n in JOINT_NAMES_LIST]

        # 更新臂关节
        for bridge in self._bridges.values():
            bridge.update_joints_from_sim(positions)

        # 底盘角度桥接: D1
        d1_name = JOINT_NAMES_LIST[0]
        if self._base_angle_pub and d1_name in name_to_pos:
            d1_deg = math.degrees(name_to_pos[d1_name])
            with self._d1_lock:
                self._current_d1_angle = d1_deg
            angle_msg = Float64()
            angle_msg.data = d1_deg
            self._base_angle_pub.publish(angle_msg)

    # ─── 共享目标管理 (sim模式) ───

    def _get_current_d1(self) -> float:
        """获取当前D1角度 (度)"""
        with self._d1_lock:
            return self._current_d1_angle

    def _update_and_publish_target(self, indices, joints_rad):
        """更新共享23关节目标数组的指定部分并发布
        Args:
            indices: 关节索引列表
            joints_rad: 对应的关节角度 (弧度)
        """
        if not self._target_joints_pub:
            return
        with self._shared_target_lock:
            for idx, val in zip(indices, joints_rad):
                self._shared_target[idx] = math.degrees(val)
            msg = Float64MultiArray()
            msg.data = list(self._shared_target)
        self._target_joints_pub.publish(msg)

    def _on_gripper_target(self, msg: Float64MultiArray):
        """处理夹爪目标 (来自 GripperController 或 gui_node)
        msg.data = [L1, L11, R1, R11] (弧度)
        → 更新共享目标 [19-22] 并发布 /target_joints
        """
        if len(msg.data) != 4:
            self.get_logger().warn(
                f'gripper_target 数据长度错误: 需要4, 收到{len(msg.data)}')
            return

        if not self._target_joints_pub:
            return

        # 夹爪索引: arm_a=[19,20], arm_b=[21,22]
        gripper_indices = (GRIPPER_JOINT_INDICES['arm_a'] +
                           GRIPPER_JOINT_INDICES['arm_b'])
        with self._shared_target_lock:
            for idx, val in zip(gripper_indices, msg.data):
                self._shared_target[idx] = math.degrees(val)
            out = Float64MultiArray()
            out.data = list(self._shared_target)
        self._target_joints_pub.publish(out)

    def _on_rotate_cmd(self, msg: Float64):
        """sim模式: 处理底盘旋转命令 (D1)"""
        threading.Thread(
            target=self._exec_sim_rotate,
            args=(msg.data,), daemon=True).start()

    def _exec_sim_rotate(self, target_angle: float):
        """sim模式: 执行D1旋转 → 更新共享目标 → 等待到位 → 发布结果"""
        D1_INDEX = 0
        POSITION_TOL = 0.5  # 度

        self.get_logger().info(
            f'[D1] sim旋转命令: {target_angle}°')

        with self._shared_target_lock:
            self._shared_target[D1_INDEX] = target_angle
            msg = Float64MultiArray()
            msg.data = list(self._shared_target)
        self._target_joints_pub.publish(msg)

        timeout = self._sim_motion_timeout
        start = time.time()
        while time.time() - start < timeout:
            time.sleep(0.1)
            with self._d1_lock:
                current = self._current_d1_angle
            if abs(current - target_angle) < POSITION_TOL:
                self.get_logger().info(
                    f'[D1] sim旋转到位 ({current:.1f}°)')
                result = Bool()
                result.data = True
                self._rotate_result_pub.publish(result)
                return

        self.get_logger().warn(
            f'[D1] sim旋转超时 ({timeout}s), '
            f'当前={self._current_d1_angle:.1f}° 目标={target_angle}°')
        result = Bool()
        result.data = False
        self._rotate_result_pub.publish(result)

    def _publish_states(self):
        for bridge in self._bridges.values():
            bridge.publish_state()

    def destroy_node(self):
        for bridge in self._bridges.values():
            bridge.disconnect_sdk()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UnifiedArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
