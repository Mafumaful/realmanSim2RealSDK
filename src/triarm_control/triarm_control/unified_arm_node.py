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
from typing import List, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Float64, Float64MultiArray
from rm_ros_interfaces.msg import Movel, Movej, Movejp
from tf2_ros import TransformBroadcaster

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
                 sim_motion_timeout: float = 40.0,
                 sim_motion_grace_period: float = 8.0,
                 base_position: List[float] = None,
                 base_orientation_deg: List[float] = None,
                 d6_mm: float = 144):
        self.node = node
        self.arm_name = arm_name
        self.mode = mode
        self.logger = node.get_logger()
        self._tag = f'[ArmBridge:{arm_name}]'
        self._joint_indices = ARM_JOINT_INDICES.get(arm_name, [])
        self._sim_joint_tol = sim_joint_tol
        self._sim_motion_timeout = sim_motion_timeout
        self._sim_motion_grace_period = sim_motion_grace_period

        # SDK wrapper (模式感知，内部自动路由 sim/real)
        self._sdk = RealManSDKWrapper(
            ip, port, arm_id, mode=mode,
            base_position=base_position,
            base_orientation_deg=base_orientation_deg,
            d6_mm=d6_mm)

        # 当前状态
        self._current_joints = [0.0] * 6  # 6关节弧度
        self._commanded_joints = [0.0] * 6  # /joint_command 中的6关节弧度
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
        self._movep_result_pub = node.create_publisher(
            Bool, f'{prefix}rm_driver/movep_result', 10)

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
        self._movep_sub = node.create_subscription(
            Movel, f'{prefix}rm_driver/movep_cmd',
            self._on_movep, 10)
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

    def _on_movep(self, msg: Movel):
        threading.Thread(
            target=self._exec_movep, args=(msg,), daemon=True).start()

    def _on_stop(self, msg: Empty):
        self.logger.warn(f'{self._tag} 收到停止命令')
        self._stop_event.set()  # 取消 sim 模式等待
        if self.mode == 'real':
            self._sdk.stop()

    def world_pose_to_joints(
        self, x, y, z, rx, ry, rz
    ) -> Optional[List[float]]:
        """世界坐标系位姿 → 关节角度 (弧度)。

        base 安装位姿和 D1 转盘变换已经由 RM65Robot 内部处理，
        这里不能再手动把世界坐标转一次臂基座坐标。
        """
        algo = self._sdk._algo
        if not algo.is_ready or not algo._robot:
            self.logger.warn(f'{self._tag} Algo未就绪')
            return None

        d1_deg = self._get_d1_angle() if self._get_d1_angle else 0.0
        # URDF 中 D1 轴方向与 Isaac Sim 当前反馈符号相反。
        algo._robot.set_turntable_angle(-d1_deg)

        with self._lock:
            q_ref_deg = [math.degrees(q) for q in self._current_joints]

        self.logger.info(
            f'{self._tag} '
            f'世界xyz=[{x:.3f},{y:.3f},{z:.3f}] '
            f'rpy=[{rx:.3f},{ry:.3f},{rz:.3f}] '
            f'(D1={d1_deg:.1f}°)')

        result = algo._robot.inverse_kinematics(
            target_position=[x, y, z],
            target_orientation_deg=[
                math.degrees(rx),
                math.degrees(ry),
                math.degrees(rz),
            ],
            current_joint_angles_deg=q_ref_deg,
        )
        if not result or not result.get('success'):
            self.logger.warn(
                f'{self._tag} IK求解失败 '
                f'(世界xyz=[{x:.3f},{y:.3f},{z:.3f}] '
                f'rpy=[{rx:.3f},{ry:.3f},{rz:.3f}], '
                f'q_ref_deg={[round(v, 1) for v in q_ref_deg]})')
            return None

        return [math.radians(d) for d in result['joint_angles_deg']]

    # ─── 执行逻辑 ───

    def _exec_movel(self, msg: Movel):
        with self._motion_lock:
            self._stop_event.clear()
            x, y, z, rx, ry, rz = pose_to_xyzrpy(msg.pose)
            speed = msg.speed
            if self.mode == 'real':
                success = self._real_movel(x, y, z, rx, ry, rz, speed)
            else:
                success = self._sim_movel_cartesian(x, y, z, rx, ry, rz)
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

    def _exec_movep(self, msg: Movel):
        with self._motion_lock:
            self._stop_event.clear()
            x, y, z, rx, ry, rz = pose_to_xyzrpy(msg.pose)
            speed = msg.speed
            if self.mode == 'real':
                success = self._real_movep(x, y, z, rx, ry, rz, speed)
            else:
                success = self._sim_move_to_pose(x, y, z, rx, ry, rz)
        result = Bool()
        result.data = success
        self._movep_result_pub.publish(result)

    # ─── Real 模式 ───

    def _real_movel(self, x, y, z, rx, ry, rz, speed) -> bool:
        self.logger.info(f'{self._tag} Real MoveL: xyz=[{x:.3f},{y:.3f},{z:.3f}] speed={speed}')
        ret = self._sdk.movel(x, y, z, rx, ry, rz, speed)
        success = ret == SDKMotionResult.SUCCESS
        if not success:
            self.logger.error(f'{self._tag} Real MoveL失败: {ret}')
        return success

    def _real_movej(self, joints, speed) -> bool:
        self.logger.info(f'{self._tag} Real MoveJ: joints={[f"{math.degrees(j):.1f}" for j in joints[:3]]}... speed={speed}')
        ret = self._sdk.movej(joints, speed)
        success = ret == SDKMotionResult.SUCCESS
        if not success:
            self.logger.error(f'{self._tag} Real MoveJ失败: {ret}')
        return success

    def _real_movejp(self, x, y, z, rx, ry, rz, speed) -> bool:
        self.logger.info(f'{self._tag} Real MoveJP: xyz=[{x:.3f},{y:.3f},{z:.3f}] speed={speed}')
        ret = self._sdk.movej_p(x, y, z, rx, ry, rz, speed)
        success = ret == SDKMotionResult.SUCCESS
        if not success:
            self.logger.error(f'{self._tag} Real MoveJP失败: {ret}')
        return success

    def _real_movep(self, x, y, z, rx, ry, rz, speed) -> bool:
        self.logger.info(f'{self._tag} Real MoveP: xyz=[{x:.3f},{y:.3f},{z:.3f}] speed={speed}')
        ret = self._sdk.movep(x, y, z, rx, ry, rz, speed)
        success = ret == SDKMotionResult.SUCCESS
        if not success:
            self.logger.error(f'{self._tag} Real MoveP失败: {ret}')
        return success

    # ─── Sim 模式 ───

    def _sim_move_to_pose(self, x, y, z, rx, ry, rz) -> bool:
        """sim模式: 世界坐标系Pose → RM65Robot IK → 关节角度"""
        if not self._sdk.is_ready:
            self.logger.error(f'{self._tag} SDK未就绪，无法执行IK')
            return False

        joints = self.world_pose_to_joints(x, y, z, rx, ry, rz)
        if joints is None:
            return False

        return self._sim_move_joints(joints)

    def _sim_movel_cartesian(self, x_end, y_end, z_end, rx_end, ry_end, rz_end) -> bool:
        """sim模式: 笛卡尔空间直线插值"""
        if not self._sdk.is_ready:
            self.logger.error(f'{self._tag} SDK未就绪')
            return False

        # 获取当前位姿
        with self._lock:
            current_joints = list(self._current_joints)
        pose_dict = self._sdk.forward_kinematics(current_joints)
        if not pose_dict:
            self.logger.error(f'{self._tag} FK失败')
            return False

        x_start, y_start, z_start = pose_dict['x'], pose_dict['y'], pose_dict['z']
        rx_start, ry_start, rz_start = pose_dict['rx'], pose_dict['ry'], pose_dict['rz']

        # 计算插值步数
        distance = math.sqrt((x_end - x_start)**2 + (y_end - y_start)**2 + (z_end - z_start)**2)
        num_steps = max(10, int(math.ceil(distance / 0.01)))

        self.logger.info(f'{self._tag} 笛卡尔插值: {num_steps}步, 距离={distance:.3f}m')

        # 逐点插值并发送
        for i in range(1, num_steps + 1):
            if self._stop_event.is_set():
                self.logger.warn(f'{self._tag} 运动被取消')
                return False

            t = i / num_steps
            # 位置线性插值
            x_interp = x_start + t * (x_end - x_start)
            y_interp = y_start + t * (y_end - y_start)
            z_interp = z_start + t * (z_end - z_start)

            # 姿态线性插值（欧拉角）
            rx_interp = rx_start + t * (rx_end - rx_start)
            ry_interp = ry_start + t * (ry_end - ry_start)
            rz_interp = rz_start + t * (rz_end - rz_start)

            # IK求解（使用上一个插值点的解作为参考）
            joints = self.world_pose_to_joints(x_interp, y_interp, z_interp,
                                                rx_interp, ry_interp, rz_interp)
            if joints is None:
                self.logger.error(f'{self._tag} IK失败于插值点{i}/{num_steps}')
                return False

            # 更新参考解为当前IK解，提高下一个插值点的IK成功率
            with self._lock:
                self._current_joints = list(joints)

            # 发送关节目标
            self._publish_shared_target(self._joint_indices, joints)
            time.sleep(0.05)

        self.logger.info(f'{self._tag} 笛卡尔插值完成')
        return True

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
        max_err = float('inf')
        cmd_err = float('inf')
        start = time.time()
        while time.time() - start < self._sim_motion_timeout:
            if self._stop_event.is_set():
                self.logger.warn(f'{self._tag} sim运动被取消')
                return False
            time.sleep(0.05)
            with self._lock:
                current = list(self._current_joints)
                commanded = list(self._commanded_joints)
            max_err = max(abs(c - t) for c, t in zip(current, target))
            if max_err < self._sim_joint_tol:
                self.logger.info(
                    f'{self._tag} sim关节到位 (max_err={max_err:.4f} rad)')
                return True
            cmd_err = max(abs(c - t) for c, t in zip(commanded, target))
            if cmd_err < self._sim_joint_tol:
                self.logger.warn(
                    f'{self._tag} /joint_states 未及时收敛，'
                    f'使用 /joint_command 兜底判定到位 '
                    f'(cmd_err={cmd_err:.4f} rad, '
                    f'state_err={max_err:.4f} rad)')
                return True

        # /joint_states 在 Isaac Sim 中经常明显滞后于 /joint_command。
        # 已经接近目标时再给一段缓冲时间，避免最后收敛阶段被过早判超时。
        if self._sim_motion_grace_period > 0.0:
            self.logger.warn(
                f'{self._tag} 主超时已到，进入额外等待 '
                f'({self._sim_motion_grace_period}s, '
                f'cmd_err={cmd_err:.4f} rad, state_err={max_err:.4f} rad)')
            grace_start = time.time()
            while time.time() - grace_start < self._sim_motion_grace_period:
                if self._stop_event.is_set():
                    self.logger.warn(f'{self._tag} sim运动被取消')
                    return False
                time.sleep(0.05)
                with self._lock:
                    current = list(self._current_joints)
                    commanded = list(self._commanded_joints)
                max_err = max(abs(c - t) for c, t in zip(current, target))
                if max_err < self._sim_joint_tol:
                    self.logger.info(
                        f'{self._tag} sim关节到位(额外等待) '
                        f'(max_err={max_err:.4f} rad)')
                    return True
                cmd_err = max(abs(c - t) for c, t in zip(commanded, target))
                if cmd_err < self._sim_joint_tol:
                    self.logger.warn(
                        f'{self._tag} 额外等待阶段使用 /joint_command 兜底判定到位 '
                        f'(cmd_err={cmd_err:.4f} rad, '
                        f'state_err={max_err:.4f} rad)')
                    return True

        self.logger.warn(
            f'{self._tag} sim运动超时 '
            f'({self._sim_motion_timeout + self._sim_motion_grace_period}s) '
            f'max_err={max_err:.4f} rad ({math.degrees(max_err):.2f}°)')
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
            # FK 前同步转盘角度，确保与 IK 一致
            d1_deg = self._get_d1_angle() if self._get_d1_angle else 0.0
            algo = self._sdk._algo
            if algo.is_ready:
                algo._robot.set_turntable_angle(-d1_deg)
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
        vals = [joint_positions[idx] if idx < len(joint_positions) else 0.0
                for idx in self._joint_indices]
        with self._lock:
            self._current_joints = vals

    def update_commanded_joints(self, joint_positions: list):
        """从 /joint_command 更新当前插值命令关节状态"""
        vals = [joint_positions[idx] if idx < len(joint_positions) else 0.0
                for idx in self._joint_indices]
        with self._lock:
            self._commanded_joints = vals


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
        self.declare_parameter('sim_joint_tolerance', 0.03)
        self.declare_parameter('sim_motion_timeout', 40.0)
        self.declare_parameter('sim_motion_grace_period', 8.0)

        # Base 参数 (RM65Robot 初始化参数, 单位: m, deg)
        self.declare_parameter('arm_a.base_position', [0.05457, -0.04863, 0.2273])
        self.declare_parameter('arm_a.base_orientation_deg', [45.0, 90.0, 0.0])
        self.declare_parameter('arm_a.d6_mm', 144)
        self.declare_parameter('arm_b.base_position', [-0.04867, -0.05374, 0.2273])
        self.declare_parameter('arm_b.base_orientation_deg', [135.0, 90.0, 0.0])
        self.declare_parameter('arm_b.d6_mm', 144)
        self.declare_parameter('arm_s.base_position', [0.0, 0.0, 0.0])
        self.declare_parameter('arm_s.base_orientation_deg', [0.0, 0.0, 0.0])
        self.declare_parameter('arm_s.d6_mm', 144)

        mode = self.get_parameter('mode').value
        ns = self.get_parameter('namespace').value
        rate = self.get_parameter('publish_rate').value
        self._sim_joint_tol = self.get_parameter('sim_joint_tolerance').value
        self._sim_motion_timeout = self.get_parameter('sim_motion_timeout').value
        self._sim_motion_grace_period = self.get_parameter(
            'sim_motion_grace_period').value

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

        # 合并的joint_states发布器 (real/sim通用)
        self._merged_joint_states_pub = self.create_publisher(
            JointState, f'{prefix}joint_states', 10)
        self._merged_joint_command_pub = self.create_publisher(
            JointState, f'{prefix}joint_command', 10)

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

            # 订阅 /joint_command (controller_node 插值输出)
            # 作为 sim 运动完成判定的兜底反馈，同时同步共享目标。
            self._cmd_sync_sub = self.create_subscription(
                JointState, f'{prefix}joint_command',
                self._on_cmd_sync, 10)

        # 创建 A/B 臂桥接器
        self._bridges = {}
        for arm_name in ['arm_a', 'arm_b']:
            ip = self.get_parameter(f'{arm_name}.ip').value
            port = self.get_parameter(f'{arm_name}.port').value
            arm_id = arm_name[-1].upper()

            # 获取 base 参数
            base_position = list(self.get_parameter(f'{arm_name}.base_position').value)
            base_orientation_deg = list(self.get_parameter(f'{arm_name}.base_orientation_deg').value)
            d6_mm = self.get_parameter(f'{arm_name}.d6_mm').value

            bridge = ArmBridge(
                self, arm_name, mode, ip, port, arm_id,
                sim_joint_tol=self._sim_joint_tol,
                sim_motion_timeout=self._sim_motion_timeout,
                sim_motion_grace_period=self._sim_motion_grace_period,
                base_position=base_position,
                base_orientation_deg=base_orientation_deg,
                d6_mm=d6_mm)
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

        # sim模式: 启动时发布零位目标，确保机械臂归零
        if mode == 'sim' and self._target_joints_pub:
            self._init_timer = self.create_timer(0.5, self._publish_initial_zero_position)
        else:
            self._init_timer = None

        # TF发布器：动态发布平台和相机TF
        self._tf_broadcaster = TransformBroadcaster(self)
        self._publish_base_tfs()  # 发布基础静态TF

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

    def _on_cmd_sync(self, msg: JointState):
        """从 controller_node 的 /joint_command 同步插值命令状态"""
        name_to_pos = dict(zip(msg.name, msg.position))
        positions = [name_to_pos.get(n, 0.0) for n in JOINT_NAMES_LIST]

        for bridge in self._bridges.values():
            bridge.update_commanded_joints(positions)

        with self._shared_target_lock:
            for i, name in enumerate(msg.name):
                if i < len(msg.position) and name in JOINT_NAMES_LIST:
                    idx = JOINT_NAMES_LIST.index(name)
                    self._shared_target[idx] = math.degrees(msg.position[i])

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

    def _publish_initial_zero_position(self):
        """启动时发布零位目标（仅执行一次）"""
        if self._target_joints_pub:
            msg = Float64MultiArray()
            msg.data = self._shared_target.copy()
            self._target_joints_pub.publish(msg)
            self.get_logger().info('已发布初始零位目标')
        if self._init_timer:
            self._init_timer.cancel()
            self._init_timer = None

    def _publish_base_tfs(self):
        """发布基础静态TF：base_link → platform_link"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'platform_link'
        t.transform.translation.z = 0.05
        t.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(t)

    def _publish_camera_tfs(self):
        """根据当前关节状态发布相机TF（动态更新）"""
        transforms = []
        now = self.get_clock().now().to_msg()

        def _append_wrist_camera_tf(arm_name: str, camera_frame: str):
            bridge = self._bridges.get(arm_name)
            if bridge is None:
                return

            with bridge._lock:
                pose = bridge._current_pose
                qx = pose.orientation.x
                qy = pose.orientation.y
                qz = pose.orientation.z
                qw = pose.orientation.w
                px = pose.position.x
                py = pose.position.y
                pz = pose.position.z

            quat_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
            if quat_norm < 1e-6:
                return

            pose_copy = Pose()
            pose_copy.position.x = px
            pose_copy.position.y = py
            pose_copy.position.z = pz
            pose_copy.orientation.x = qx
            pose_copy.orientation.y = qy
            pose_copy.orientation.z = qz
            pose_copy.orientation.w = qw

            x, y, z, rx, ry, rz = pose_to_xyzrpy(pose_copy)
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'platform_link'
            t.child_frame_id = camera_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            transforms.append(t)

            # self.get_logger().info(
            #     f'[{arm_name}] 发布腕部相机TF platform_link←{camera_frame}: '
            #     f'trans=[{x:.3f}, {y:.3f}, {z:.3f}], '
            #     f'rpy=[{rx:.3f}, {ry:.3f}, {rz:.3f}]',
            #     throttle_duration_sec=5.0)

        # 腕部相机：直接复用当前末端完整姿态，避免只发布 yaw 导致坐标轴错误
        _append_wrist_camera_tf('arm_a', 'camera_a_link')
        _append_wrist_camera_tf('arm_b', 'camera_b_link')

        # Link_S6 (S臂末端全局相机) - 使用标定验证后的实际位姿
        # 平移: 相机在 platform_link 下的位置 (由 grasp_test 验证)
        # 旋转: 绕固定轴 XYZ 顺序: RX=π(向下看) + RZ=π/2(相机X轴对齐)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'platform_link'
        t.child_frame_id = 'Link_S6'
        t.transform.translation.x = 0.40991
        t.transform.translation.y = -0.01001
        t.transform.translation.z = 0.63422
        # 旋转: euler('sxyz', [π, 0, π/2]) — 与 grasp_test 验证一致
        R = txe.euler2mat(math.pi, 0.0, math.pi / 2.0, 'sxyz')
        q = txq.mat2quat(R)
        t.transform.rotation.x = float(q[1])
        t.transform.rotation.y = float(q[2])
        t.transform.rotation.z = float(q[3])
        t.transform.rotation.w = float(q[0])
        transforms.append(t)

        if transforms:
            self._tf_broadcaster.sendTransform(transforms)

    def _publish_states(self):
        for bridge in self._bridges.values():
            bridge.publish_state()

        # real模式: 合并各臂关节状态发布到 robot/joint_states
        if self._mode == 'real':
            merged_positions = [0.0] * TOTAL_JOINT_COUNT

            # 收集各臂关节数据
            for arm_name, bridge in self._bridges.items():
                indices = ARM_JOINT_INDICES.get(arm_name, [])
                with bridge._lock:
                    for i, idx in enumerate(indices):
                        if i < len(bridge._current_joints):
                            merged_positions[idx] = bridge._current_joints[i]

            # 发布合并的joint_states
            js_msg = JointState()
            js_msg.header.stamp = self.get_clock().now().to_msg()
            js_msg.name = JOINT_NAMES_LIST
            js_msg.position = merged_positions
            self._merged_joint_states_pub.publish(js_msg)

        # 动态发布相机TF
        self._publish_camera_tfs()

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
