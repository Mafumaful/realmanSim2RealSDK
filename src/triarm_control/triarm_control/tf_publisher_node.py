#!/usr/bin/env python3
"""
TF 发布节点 — 统一管理所有坐标变换

发布内容：
  静态变换 (StaticTransformBroadcaster)：
    base_link        → platform_link
    platform_link    → Link_S6

  准静态变换 (TransformBroadcaster, 定时刷新)：
    platform_link    → turntable_link   (D1 关节驱动，Rz)
    turntable_link   → arm_a_base_link  (臂安装偏移，sim/real 两套)
    turntable_link   → arm_b_base_link

  动态变换 (TransformBroadcaster, 跟随 FK 实时更新)：
    platform_link    → camera_a_link    (A臂 RM65Robot FK 末端)
    platform_link    → camera_b_link    (B臂 RM65Robot FK FK 末端)

所有参数从 triarm_config.yaml 的 tf_publisher_node 段读取。
其他节点不再发布任何 TF，从 /tf 和 /tf_static 查询所需变换。
"""

import math
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

import numpy as np


def _euler_deg_to_quat(roll_deg, pitch_deg, yaw_deg):
    """固定轴 XYZ 欧拉角(deg) → 四元数 (w, x, y, z)

    解析公式（sxyz：先绕固定 X，再 Y，再 Z）：
      q = q_z ⊗ q_y ⊗ q_x
    直接用半角三角函数计算，无需构造旋转矩阵。
    """
    cr, sr = math.cos(math.radians(roll_deg)  / 2), math.sin(math.radians(roll_deg)  / 2)
    cp, sp = math.cos(math.radians(pitch_deg) / 2), math.sin(math.radians(pitch_deg) / 2)
    cy, sy = math.cos(math.radians(yaw_deg)   / 2), math.sin(math.radians(yaw_deg)   / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,   # w
        sr * cp * cy - cr * sp * sy,   # x
        cr * sp * cy + sr * cp * sy,   # y
        cr * cp * sy - sr * sp * cy,   # z
    ])


def _make_stamped(stamp, parent, child, xyz, quat_wxyz):
    """构造 TransformStamped"""
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = float(xyz[0])
    t.transform.translation.y = float(xyz[1])
    t.transform.translation.z = float(xyz[2])
    t.transform.rotation.w = float(quat_wxyz[0])
    t.transform.rotation.x = float(quat_wxyz[1])
    t.transform.rotation.y = float(quat_wxyz[2])
    t.transform.rotation.z = float(quat_wxyz[3])
    return t


class TFPublisherNode(Node):
    """统一 TF 发布节点"""

    def __init__(self):
        super().__init__('tf_publisher_node')

        # ── 读取参数 ──
        self.declare_parameter('namespace', 'robot')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('mode', 'sim')

        # 静态变换
        self.declare_parameter('static_transforms.base_to_platform.parent', 'base_link')
        self.declare_parameter('static_transforms.base_to_platform.child', 'platform_link')
        self.declare_parameter('static_transforms.base_to_platform.xyzrpy',
                               [0.0, 0.0, 0.05, 0.0, 0.0, 0.0])
        self.declare_parameter('static_transforms.platform_to_s6_camera.parent', 'platform_link')
        self.declare_parameter('static_transforms.platform_to_s6_camera.child', 'Link_S6')
        self.declare_parameter('static_transforms.platform_to_s6_camera.xyzrpy',
                               [0.40991, -0.01001, 0.63422, 180.0, 0.0, 90.0])

        # 转盘
        self.declare_parameter('turntable.parent', 'platform_link')
        self.declare_parameter('turntable.child', 'turntable_link')
        self.declare_parameter('turntable.axis_sign', -1)

        # 臂安装偏移
        for arm in ['arm_a', 'arm_b']:
            self.declare_parameter(f'arm_bases.{arm}.parent', 'turntable_link')
            self.declare_parameter(f'arm_bases.{arm}.child', f'{arm}_base_link')
            for mode in ['sim', 'real']:
                self.declare_parameter(f'arm_bases.{arm}.{mode}.xyzrpy',
                                       [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # 腕部相机
        for arm in ['arm_a', 'arm_b']:
            self.declare_parameter(f'wrist_cameras.{arm}.parent', 'platform_link')
            self.declare_parameter(f'wrist_cameras.{arm}.child', f'camera_{arm[-1]}_link')

        # 臂 FK 参数（与 unified_arm_node / triarm_gui 共享，值来自 /** 全局参数）
        for arm in ['arm_a', 'arm_b']:
            self.declare_parameter(f'{arm}.base_position', [0.0, 0.0, 0.0])
            self.declare_parameter(f'{arm}.base_orientation_deg', [0.0, 0.0, 0.0])
            self.declare_parameter(f'{arm}.sim_d6_mm', 172.5)
            self.declare_parameter(f'{arm}.real_d6_mm', 144.0)

        ns = self.get_parameter('namespace').value
        rate = self.get_parameter('publish_rate').value
        self._mode = self.get_parameter('mode').value

        # ── 广播器 ──
        self._static_br = StaticTransformBroadcaster(self)
        self._dynamic_br = TransformBroadcaster(self)

        # ── 状态 ──
        self._d1_angle_rad = 0.0          # D1 关节角（弧度）
        self._arm_poses = {}              # {arm_name: (xyz, quat_wxyz) | None}
        self._lock = threading.Lock()

        # ── 初始化 FK 算法 ──
        self._algos = {}
        self._init_algos()

        # ── 发布静态 TF（只发一次）──
        self._publish_static_tfs()

        # ── 订阅 joint_states ──
        state_topic = f'/{ns}/joint_states' if ns else '/joint_states'
        self.create_subscription(JointState, state_topic, self._on_joint_states, 10)

        # ── 定时发布动态 TF ──
        self.create_timer(1.0 / rate, self._publish_dynamic_tfs)

        self.get_logger().info(
            f'[TFPublisher] 启动 (mode={self._mode}, rate={rate}Hz, ns={ns})')

    # ================================================================
    # 初始化
    # ================================================================

    def _init_algos(self):
        """为 A/B 两臂各初始化一个 RM65Robot 算法实例，用于腕部相机 FK"""
        try:
            from .rm65_robot import RM65Robot
        except ImportError:
            self.get_logger().error('[TFPublisher] 无法导入 RM65Robot，腕部相机 TF 不可用')
            return

        for arm in ['arm_a', 'arm_b']:
            d6_key = f'{arm}.sim_d6_mm' if self._mode == 'sim' else f'{arm}.real_d6_mm'
            d6 = self.get_parameter(d6_key).value
            bp = list(self.get_parameter(f'{arm}.base_position').value)
            bo = list(self.get_parameter(f'{arm}.base_orientation_deg').value)
            try:
                robot = RM65Robot(base_position=bp, base_orientation_deg=bo, d6_mm=d6)
                self._algos[arm] = robot
                self._arm_poses[arm] = None
                self.get_logger().info(
                    f'[TFPublisher] {arm} RM65Robot 初始化成功 '
                    f'base_pos={bp} base_ori={bo}')
            except Exception as e:
                self.get_logger().error(f'[TFPublisher] {arm} RM65Robot 初始化失败: {e}')

    # ================================================================
    # 静态 TF
    # ================================================================

    def _publish_static_tfs(self):
        """发布所有静态变换（用 StaticTransformBroadcaster，只发一次永久有效）"""
        now = self.get_clock().now().to_msg()
        static_list = []

        # base_link → platform_link
        p1 = self.get_parameter('static_transforms.base_to_platform.parent').value
        c1 = self.get_parameter('static_transforms.base_to_platform.child').value
        xyzrpy1 = list(self.get_parameter('static_transforms.base_to_platform.xyzrpy').value)
        static_list.append(self._xyzrpy_to_stamped(now, p1, c1, xyzrpy1))

        # platform_link → Link_S6
        p2 = self.get_parameter('static_transforms.platform_to_s6_camera.parent').value
        c2 = self.get_parameter('static_transforms.platform_to_s6_camera.child').value
        xyzrpy2 = list(self.get_parameter('static_transforms.platform_to_s6_camera.xyzrpy').value)
        static_list.append(self._xyzrpy_to_stamped(now, p2, c2, xyzrpy2))

        self._static_br.sendTransform(static_list)
        self.get_logger().info(
            f'[TFPublisher] 已发布 {len(static_list)} 条静态 TF')

    # ================================================================
    # joint_states 回调
    # ================================================================

    def _on_joint_states(self, msg: JointState):
        """从 joint_states 更新 D1 角度 + 各臂关节角，触发 FK 计算"""
        name_to_pos = dict(zip(msg.name, msg.position))

        with self._lock:
            # D1 转盘
            d1 = name_to_pos.get('joint_platform_D1', None)
            if d1 is not None:
                self._d1_angle_rad = d1

            # 各臂 FK
            arm_joints_map = {
                'arm_a': [f'joint_platform_A{i}' for i in range(1, 7)],
                'arm_b': [f'joint_platform_B{i}' for i in range(1, 7)],
            }
            for arm, joint_names in arm_joints_map.items():
                robot = self._algos.get(arm)
                if robot is None:
                    continue
                joints_deg = []
                ok = True
                for jn in joint_names:
                    val = name_to_pos.get(jn)
                    if val is None:
                        ok = False
                        break
                    joints_deg.append(math.degrees(val))
                if not ok:
                    continue

                # 注入转盘角度（取反，与 unified_arm_node 一致）
                d1_deg = math.degrees(self._d1_angle_rad)
                robot.set_turntable_angle(-d1_deg)

                result = robot.forward_kinematics(joints_deg)
                if result:
                    pos = result['position']
                    rx, ry, rz = result['euler_rad']   # ZYX 欧拉角 (szyx)
                    # ZYX → 四元数：q = q_x ⊗ q_y ⊗ q_z
                    cx, sx = math.cos(rx / 2), math.sin(rx / 2)
                    cy, sy = math.cos(ry / 2), math.sin(ry / 2)
                    cz, sz = math.cos(rz / 2), math.sin(rz / 2)
                    q = np.array([
                        cx * cy * cz + sx * sy * sz,   # w
                        sx * cy * cz - cx * sy * sz,   # x
                        cx * sy * cz + sx * cy * sz,   # y
                        cx * cy * sz - sx * sy * cz,   # z
                    ])
                    self._arm_poses[arm] = (pos, q)

    # ================================================================
    # 动态 TF 定时发布
    # ================================================================

    def _publish_dynamic_tfs(self):
        """定时发布：转盘 + 臂安装偏移 + 腕部相机"""
        now = self.get_clock().now().to_msg()
        transforms = []

        with self._lock:
            d1_rad = self._d1_angle_rad
            arm_poses = dict(self._arm_poses)

        # ── 转盘：platform_link → turntable_link ──
        tt_parent = self.get_parameter('turntable.parent').value
        tt_child = self.get_parameter('turntable.child').value
        axis_sign = int(self.get_parameter('turntable.axis_sign').value)
        angle = axis_sign * d1_rad
        # 绕 Z 轴旋转：q = [cos(θ/2), 0, 0, sin(θ/2)]
        q_d1 = np.array([math.cos(angle / 2), 0.0, 0.0, math.sin(angle / 2)])
        transforms.append(_make_stamped(now, tt_parent, tt_child, [0, 0, 0], q_d1))

        # ── 臂安装偏移：turntable_link → arm_x_base_link ──
        for arm in ['arm_a', 'arm_b']:
            parent = self.get_parameter(f'arm_bases.{arm}.parent').value
            child = self.get_parameter(f'arm_bases.{arm}.child').value
            xyzrpy = list(self.get_parameter(
                f'arm_bases.{arm}.{self._mode}.xyzrpy').value)
            transforms.append(self._xyzrpy_to_stamped(now, parent, child, xyzrpy))

        # ── 腕部相机：platform_link → camera_x_link ──
        for arm in ['arm_a', 'arm_b']:
            pose = arm_poses.get(arm)
            if pose is None:
                continue
            pos, q = pose
            parent = self.get_parameter(f'wrist_cameras.{arm}.parent').value
            child = self.get_parameter(f'wrist_cameras.{arm}.child').value
            transforms.append(_make_stamped(now, parent, child, pos, q))

        self._dynamic_br.sendTransform(transforms)

    # ================================================================
    # 工具方法
    # ================================================================

    def _xyzrpy_to_stamped(self, stamp, parent, child, xyzrpy):
        """[x,y,z,roll_deg,pitch_deg,yaw_deg] → TransformStamped"""
        xyz = xyzrpy[:3]
        q = _euler_deg_to_quat(xyzrpy[3], xyzrpy[4], xyzrpy[5])
        return _make_stamped(stamp, parent, child, xyz, q)


# ================================================================
# 入口
# ================================================================

def main(args=None):
    rclpy.init(args=args)
    node = TFPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

