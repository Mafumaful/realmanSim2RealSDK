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

双模式：
  sim: Pose → 欧拉角 → IK(SDK Algo) → 关节角度 → 映射到19关节数组 → 现有插值逻辑
  real: Pose → 欧拉角 → SDK TCP 直接控制 RM65

夹爪桥接话题也在此节点中创建。
"""

import math
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Float64MultiArray
from rm_ros_interfaces.msg import Movel, Movej, Movejp

import transforms3d.quaternions as txq
import transforms3d.euler as txe

from .realman_sdk_wrapper import RealManSDKWrapper, SDKMotionResult
from .joint_names import JOINT_NAMES_LIST


# 臂关节在19关节数组中的索引映射
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
                 ip: str, port: int, arm_id: str):
        self.node = node
        self.arm_name = arm_name
        self.mode = mode
        self.logger = node.get_logger()
        self._tag = f'[ArmBridge:{arm_name}]'
        self._joint_indices = ARM_JOINT_INDICES.get(arm_name, [])

        # SDK wrapper (real模式 或 sim模式IK解算都需要)
        self._sdk = RealManSDKWrapper(ip, port, arm_id)
        self._sdk_connected = False

        # 当前状态
        self._current_joints = [0.0] * 6  # 6关节弧度
        self._current_pose = Pose()
        self._lock = threading.Lock()

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

        # sim模式: 发布到 /target_joints 的发布器 (由外部设置)
        self._target_joints_pub = None

        self.logger.info(
            f'{self._tag} 初始化完成 (mode={mode}, ip={ip}:{port})')

    def set_target_joints_pub(self, pub):
        """设置 /target_joints 发布器 (sim模式共享)"""
        self._target_joints_pub = pub

    def connect_sdk(self) -> bool:
        """连接SDK (real模式必须, sim模式可选用于IK)"""
        if self.mode == 'sim':
            # sim模式尝试连接用于IK，失败不阻塞
            self._sdk_connected = self._sdk.connect()
            if not self._sdk_connected:
                self.logger.warn(
                    f'{self._tag} sim模式SDK连接失败，将使用简化IK')
            return True
        else:
            # real模式必须连接
            self._sdk_connected = self._sdk.connect()
            if not self._sdk_connected:
                self.logger.error(
                    f'{self._tag} real模式SDK连接失败!')
            return self._sdk_connected

    def disconnect_sdk(self):
        if self._sdk_connected:
            self._sdk.disconnect()

    # ─── 命令回调 ───

    def _on_movel(self, msg: Movel):
        """MoveL 命令处理"""
        threading.Thread(
            target=self._exec_movel, args=(msg,), daemon=True).start()

    def _on_movej(self, msg: Movej):
        """MoveJ 命令处理"""
        threading.Thread(
            target=self._exec_movej, args=(msg,), daemon=True).start()

    def _on_movejp(self, msg: Movejp):
        """MoveJP 命令处理"""
        threading.Thread(
            target=self._exec_movejp, args=(msg,), daemon=True).start()

    def _on_stop(self, msg: Empty):
        """停止命令"""
        self.logger.warn(f'{self._tag} 收到停止命令')
        if self.mode == 'real' and self._sdk_connected:
            self._sdk.stop()

    # ─── 执行逻辑 ───

    def _exec_movel(self, msg: Movel):
        """执行 MoveL"""
        x, y, z, rx, ry, rz = pose_to_xyzrpy(msg.pose)
        speed = msg.speed
        success = False

        if self.mode == 'real':
            success = self._real_movel(x, y, z, rx, ry, rz, speed)
        else:
            success = self._sim_move_to_pose(x, y, z, rx, ry, rz)

        result = Bool()
        result.data = success
        self._movel_result_pub.publish(result)

    def _exec_movej(self, msg: Movej):
        """执行 MoveJ"""
        joints = list(msg.joint)
        speed = msg.speed
        success = False

        if self.mode == 'real':
            success = self._real_movej(joints, speed)
        else:
            success = self._sim_move_joints(joints)

        result = Bool()
        result.data = success
        self._movej_result_pub.publish(result)

    def _exec_movejp(self, msg: Movejp):
        """执行 MoveJP"""
        x, y, z, rx, ry, rz = pose_to_xyzrpy(msg.pose)
        speed = msg.speed
        success = False

        if self.mode == 'real':
            success = self._real_movejp(x, y, z, rx, ry, rz, speed)
        else:
            success = self._sim_move_to_pose(x, y, z, rx, ry, rz)

        result = Bool()
        result.data = success
        self._movejp_result_pub.publish(result)

    # ─── Real 模式 ───

    def _real_movel(self, x, y, z, rx, ry, rz, speed) -> bool:
        if not self._sdk_connected:
            return False
        ret = self._sdk.movel(x, y, z, rx, ry, rz, speed)
        return ret == SDKMotionResult.SUCCESS

    def _real_movej(self, joints, speed) -> bool:
        if not self._sdk_connected:
            return False
        ret = self._sdk.movej(joints, speed)
        return ret == SDKMotionResult.SUCCESS

    def _real_movejp(self, x, y, z, rx, ry, rz, speed) -> bool:
        if not self._sdk_connected:
            return False
        ret = self._sdk.movej_p(x, y, z, rx, ry, rz, speed)
        return ret == SDKMotionResult.SUCCESS

    # ─── Sim 模式 ───

    def _sim_move_to_pose(self, x, y, z, rx, ry, rz) -> bool:
        """sim模式: Pose → IK → 关节角度 → /target_joints"""
        joints = None
        if self._sdk_connected:
            joints = self._sdk.inverse_kinematics(
                x, y, z, rx, ry, rz)

        if joints is None:
            # 简化IK不可用时，直接用当前关节 (降级)
            self.logger.warn(
                f'{self._tag} IK不可用，sim模式降级')
            return False

        return self._sim_move_joints(joints)

    def _sim_move_joints(self, joints) -> bool:
        """sim模式: 关节角度 → 映射到19关节数组 → /target_joints"""
        if not self._target_joints_pub:
            self.logger.error(f'{self._tag} target_joints发布器未设置')
            return False

        if len(self._joint_indices) != len(joints):
            self.logger.error(
                f'{self._tag} 关节数不匹配: '
                f'indices={len(self._joint_indices)}, joints={len(joints)}')
            return False

        # 构建19关节数组 (角度制)
        msg = Float64MultiArray()
        data = [0.0] * 19
        for idx, joint_val in zip(self._joint_indices, joints):
            data[idx] = math.degrees(joint_val)
        msg.data = data

        self._target_joints_pub.publish(msg)

        # 更新内部状态
        with self._lock:
            self._current_joints = list(joints)

        self.logger.info(
            f'{self._tag} sim发送关节目标: '
            f'{[f"{math.degrees(j):.1f}" for j in joints]}')
        return True

    # ─── 状态反馈 ───

    def publish_state(self):
        """发布当前状态 (由定时器调用)"""
        if self.mode == 'real' and self._sdk_connected:
            # 从SDK获取真实状态
            joints = self._sdk.get_joint_positions()
            if joints:
                with self._lock:
                    self._current_joints = joints

            pose_dict = self._sdk.get_current_pose()
            if pose_dict:
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
                with self._lock:
                    self._current_pose = pose

        # 发布 JointState
        js_msg = JointState()
        js_msg.header.stamp = self.node.get_clock().now().to_msg()
        with self._lock:
            js_msg.name = [f'joint_{self.arm_name[-1]}{i+1}'
                           for i in range(6)]
            js_msg.position = list(self._current_joints)
        self._joint_state_pub.publish(js_msg)

        # 发布 Pose
        with self._lock:
            self._arm_pos_pub.publish(self._current_pose)

    def update_joints_from_sim(self, joint_positions: list):
        """从 /joint_states (Isaac Sim) 更新关节状态"""
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
        self.declare_parameter('namespace', '')
        self.declare_parameter('publish_rate', 20.0)

        mode = self.get_parameter('mode').value
        ns = self.get_parameter('namespace').value
        rate = self.get_parameter('publish_rate').value

        self.get_logger().info(f'=== UnifiedArmNode 启动 (mode={mode}) ===')

        # sim模式: /target_joints 发布器 (共享)
        prefix = f'{ns}/' if ns else '/'
        self._target_joints_pub = None
        if mode == 'sim':
            self._target_joints_pub = self.create_publisher(
                Float64MultiArray, f'{prefix}target_joints', 10)

        # 创建 A/B 臂桥接器
        self._bridges = {}
        for arm_name in ['arm_a', 'arm_b']:
            ip = self.get_parameter(f'{arm_name}.ip').value
            port = self.get_parameter(f'{arm_name}.port').value
            arm_id = arm_name[-1].upper()

            bridge = ArmBridge(
                self, arm_name, mode, ip, port, arm_id)
            if mode == 'sim':
                bridge.set_target_joints_pub(self._target_joints_pub)
            bridge.connect_sdk()
            self._bridges[arm_name] = bridge

        # S臂预留 (暂不创建)
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
        """从 Isaac Sim 的 /joint_states 更新各臂状态"""
        positions = list(msg.position)
        for bridge in self._bridges.values():
            bridge.update_joints_from_sim(positions)

    def _publish_states(self):
        """定时发布所有臂的状态"""
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
