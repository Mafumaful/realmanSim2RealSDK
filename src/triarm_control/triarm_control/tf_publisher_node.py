#!/usr/bin/env python3
"""
TF 发布节点 — 统一管理所有坐标变换

TF 树结构：
  静态变换 (StaticTransformBroadcaster)：
    base_link        → platform_link            （机器人底座）
    turntable_link   → center_camera_link       （中间相机，随转盘转动，相对转盘静止）
    arm_x_link_6     → camera_x_link            （末端→腕部相机，config 配置偏移）

  准静态变换 (TransformBroadcaster, 定时刷新)：
    platform_link    → turntable_link            （D1 关节驱动，绕 Z 轴）
    turntable_link   → arm_x_base_link           （臂安装偏移，sim/real 两套）

  动态变换 (TransformBroadcaster, 跟随 joint_states 更新)：
    arm_x_base_link  → arm_x_link_1             (关节 1)
    arm_x_link_1     → arm_x_link_2             (关节 2)
    ...
    arm_x_link_5     → arm_x_link_6             (关节 6，即末端)

可选：当 publish_robot_model=true 时，向 /joint_states 发布关节状态，
      配合 robot_state_publisher + URDF 在 RViz2 中显示模型。

所有参数从 triarm_config.yaml 的 tf_publisher_node 段读取。
其他节点不再发布任何 TF，从 /tf 和 /tf_static 查询所需变换。
"""

import math
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


# ================================================================
# 纯函数工具
# ================================================================

def _euler_deg_to_quat(roll_deg, pitch_deg, yaw_deg):
    """固定轴 XYZ 欧拉角(deg) → 四元数 (w, x, y, z)

    解析公式（sxyz：先绕固定 X，再 Y，再 Z）：
      q = q_z ⊗ q_y ⊗ q_x
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


def _xyzrpy_to_stamped(stamp, parent, child, xyzrpy):
    """[x,y,z,roll_deg,pitch_deg,yaw_deg] → TransformStamped"""
    q = _euler_deg_to_quat(xyzrpy[3], xyzrpy[4], xyzrpy[5])
    return _make_stamped(stamp, parent, child, xyzrpy[:3], q)


# ================================================================
# 节点
# ================================================================

class TFPublisherNode(Node):
    """统一 TF 发布节点"""

    # RM65 每个关节的 link 帧名模板（相对臂前缀）
    _LINK_TMPL = 'arm_{arm}_link_{i}'   # arm='a'/'b', i=1..6

    def __init__(self):
        super().__init__('tf_publisher_node')

        # ── 基础参数 ──
        self.declare_parameter('namespace',           'robot')
        self.declare_parameter('publish_rate',        20.0)
        self.declare_parameter('mode',                'sim')
        self.declare_parameter('publish_robot_model', False)

        # ── 静态变换：base_link → platform_link ──
        self.declare_parameter('static_transforms.base_to_platform.parent', 'base_link')
        self.declare_parameter('static_transforms.base_to_platform.child',  'platform_link')
        self.declare_parameter('static_transforms.base_to_platform.xyzrpy',
                               [0.0, 0.0, 0.05, 0.0, 0.0, 0.0])

        # ── 转盘 ──
        self.declare_parameter('turntable.parent',    'platform_link')
        self.declare_parameter('turntable.child',     'turntable_link')
        self.declare_parameter('turntable.axis_sign', -1)

        # ── 臂安装偏移 ──
        for arm in ['arm_a', 'arm_b']:
            self.declare_parameter(f'arm_bases.{arm}.parent', 'turntable_link')
            self.declare_parameter(f'arm_bases.{arm}.child',  f'{arm}_base_link')
            for m in ['sim', 'real']:
                self.declare_parameter(f'arm_bases.{arm}.{m}.xyzrpy',
                                       [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # ── 腕部相机（末端→相机静态偏移）──
        for arm in ['arm_a', 'arm_b']:
            arm_ch = arm[-1]   # 'a' 或 'b'
            self.declare_parameter(f'wrist_cameras.{arm}.child',
                                   f'camera_{arm_ch}_link')
            self.declare_parameter(f'wrist_cameras.{arm}.end_to_camera_xyzrpy',
                                   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # ── 中间相机（静态变换，相对 turntable_link 固定不动）──
        self.declare_parameter('center_camera.parent', 'turntable_link')
        self.declare_parameter('center_camera.child',  'center_camera_link')
        self.declare_parameter('center_camera.xyzrpy', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # ── 臂 FK 参数 ──
        for arm in ['arm_a', 'arm_b']:
            self.declare_parameter(f'{arm}.base_position',        [0.0, 0.0, 0.0])
            self.declare_parameter(f'{arm}.base_orientation_deg', [0.0, 0.0, 0.0])
            self.declare_parameter(f'{arm}.sim_d6_mm',  172.5)
            self.declare_parameter(f'{arm}.real_d6_mm', 144.0)

        # ── 读取常用参数 ──
        ns   = self.get_parameter('namespace').value
        rate = self.get_parameter('publish_rate').value
        self._mode              = self.get_parameter('mode').value
        self._publish_robot_mdl = self.get_parameter('publish_robot_model').value

        # ── 广播器 ──
        self._static_br  = StaticTransformBroadcaster(self)
        self._dynamic_br = TransformBroadcaster(self)

        # ── 状态 ──
        self._d1_angle_rad = 0.0
        self._arm_joints: dict = {'arm_a': None, 'arm_b': None}
        self._lock = threading.Lock()

        # ── 初始化 RM65Robot FK 算法 ──
        self._algos: dict = {}
        self._init_algos()

        # ── 发布静态 TF（只发一次，永久有效）──
        self._publish_static_tfs()

        # ── 订阅 joint_states ──
        state_topic = f'/{ns}/joint_states' if ns else '/joint_states'
        self.create_subscription(JointState, state_topic,
                                 self._on_joint_states, 10)

        # ── 可选：向 robot_state_publisher 发布关节状态（用于 RViz2）──
        if self._publish_robot_mdl:
            self._rviz_joint_pub = self.create_publisher(
                JointState, '/joint_states', 10)
            self.get_logger().info(
                '[TFPublisher] publish_robot_model=true，'
                '将向 /joint_states 转发关节状态供 RViz2 使用')
        else:
            self._rviz_joint_pub = None

        # ── 定时发布动态 TF ──
        self.create_timer(1.0 / rate, self._publish_dynamic_tfs)

        self.get_logger().info(
            f'[TFPublisher] 启动 '
            f'(mode={self._mode}, rate={rate}Hz, ns={ns}, '
            f'publish_robot_model={self._publish_robot_mdl})')

    # ================================================================
    # 初始化：RM65Robot FK 算法
    # ================================================================

    def _init_algos(self):
        """为 A/B 两臂各初始化一个 RM65Robot 实例，用于逐关节 TF 计算"""
        try:
            from .rm65_robot import RM65Robot
        except ImportError:
            self.get_logger().error(
                '[TFPublisher] 无法导入 RM65Robot，关节 TF 不可用')
            return

        for arm in ['arm_a', 'arm_b']:
            d6_key = (f'{arm}.sim_d6_mm' if self._mode == 'sim'
                      else f'{arm}.real_d6_mm')
            d6 = self.get_parameter(d6_key).value
            bp = list(self.get_parameter(f'{arm}.base_position').value)
            bo = list(self.get_parameter(f'{arm}.base_orientation_deg').value)
            try:
                self._algos[arm] = RM65Robot(
                    base_position=bp,
                    base_orientation_deg=bo,
                    d6_mm=d6)
                self.get_logger().info(
                    f'[TFPublisher] {arm} RM65Robot 初始化成功 '
                    f'base_pos={bp} base_ori={bo} d6={d6}mm')
            except Exception as e:
                self.get_logger().error(
                    f'[TFPublisher] {arm} RM65Robot 初始化失败: {e}')

    # ================================================================
    # 静态 TF
    # ================================================================

    def _publish_static_tfs(self):
        """发布所有静态变换（StaticTransformBroadcaster，只发一次永久有效）"""
        now = self.get_clock().now().to_msg()
        static_list = []

        # base_link → platform_link
        p = self.get_parameter('static_transforms.base_to_platform.parent').value
        c = self.get_parameter('static_transforms.base_to_platform.child').value
        xyzrpy = list(self.get_parameter(
            'static_transforms.base_to_platform.xyzrpy').value)
        static_list.append(_xyzrpy_to_stamped(now, p, c, xyzrpy))

        # arm_x_link_6 → camera_x_link（末端→腕部相机静态偏移）
        for arm in ['arm_a', 'arm_b']:
            arm_ch    = arm[-1]
            ee_frame  = f'arm_{arm_ch}_link_6'
            cam_frame = self.get_parameter(f'wrist_cameras.{arm}.child').value
            xyzrpy_ec = list(self.get_parameter(
                f'wrist_cameras.{arm}.end_to_camera_xyzrpy').value)
            static_list.append(_xyzrpy_to_stamped(now, ee_frame, cam_frame, xyzrpy_ec))

        # turntable_link → center_camera_link（中间相机，随转盘转动，相对转盘静止）
        cc_parent = self.get_parameter('center_camera.parent').value
        cc_child  = self.get_parameter('center_camera.child').value
        cc_xyzrpy = list(self.get_parameter('center_camera.xyzrpy').value)
        static_list.append(_xyzrpy_to_stamped(now, cc_parent, cc_child, cc_xyzrpy))

        self._static_br.sendTransform(static_list)
        self.get_logger().info(
            f'[TFPublisher] 已发布 {len(static_list)} 条静态 TF')

    # ================================================================
    # joint_states 回调
    # ================================================================

    def _on_joint_states(self, msg: JointState):
        """更新 D1 角度 + 各臂关节角；可选转发给 robot_state_publisher"""
        name_to_pos = dict(zip(msg.name, msg.position))

        with self._lock:
            d1 = name_to_pos.get('joint_platform_D1')
            if d1 is not None:
                self._d1_angle_rad = d1

            for arm, prefix in [('arm_a', 'A'), ('arm_b', 'B')]:
                joints = []
                ok = True
                for i in range(1, 7):
                    val = name_to_pos.get(f'joint_platform_{prefix}{i}')
                    if val is None:
                        ok = False
                        break
                    joints.append(val)
                if ok:
                    self._arm_joints[arm] = joints

        if self._rviz_joint_pub:
            self._rviz_joint_pub.publish(msg)

    # ================================================================
    # 动态 TF 定时发布
    # ================================================================

    def _publish_dynamic_tfs(self):
        """定时发布：转盘 + 臂安装偏移 + 各关节链"""
        now = self.get_clock().now().to_msg()
        transforms = []

        with self._lock:
            d1_rad     = self._d1_angle_rad
            arm_joints = {k: list(v) if v else None
                          for k, v in self._arm_joints.items()}

        # ── 转盘：platform_link → turntable_link ──
        tt_parent = self.get_parameter('turntable.parent').value
        tt_child  = self.get_parameter('turntable.child').value
        axis_sign = int(self.get_parameter('turntable.axis_sign').value)
        angle     = axis_sign * d1_rad
        q_d1 = np.array([math.cos(angle / 2), 0.0, 0.0, math.sin(angle / 2)])
        transforms.append(_make_stamped(now, tt_parent, tt_child, [0, 0, 0], q_d1))

        # ── 臂安装偏移：turntable_link → arm_x_base_link ──
        for arm in ['arm_a', 'arm_b']:
            parent = self.get_parameter(f'arm_bases.{arm}.parent').value
            child  = self.get_parameter(f'arm_bases.{arm}.child').value
            xyzrpy = list(self.get_parameter(
                f'arm_bases.{arm}.{self._mode}.xyzrpy').value)
            transforms.append(_xyzrpy_to_stamped(now, parent, child, xyzrpy))

        # ── 各臂关节链：arm_x_base_link → arm_x_link_1 → ... → arm_x_link_6 ──
        for arm in ['arm_a', 'arm_b']:
            joints_rad = arm_joints.get(arm)
            robot      = self._algos.get(arm)
            if joints_rad is None or robot is None:
                continue

            arm_ch     = arm[-1]
            joints_deg = [math.degrees(j) for j in joints_rad]
            robot.set_turntable_angle(-math.degrees(d1_rad))

            try:
                link_tfs = robot.joint_transforms(joints_deg)
            except AttributeError:
                link_tfs = None   # RM65Robot 尚未实现，降级

            if link_tfs is not None:
                base_frame = f'arm_{arm_ch}_base_link'
                for idx, (xyz, q) in enumerate(link_tfs):
                    p_frame = (base_frame if idx == 0
                               else f'arm_{arm_ch}_link_{idx}')
                    c_frame = f'arm_{arm_ch}_link_{idx + 1}'
                    transforms.append(_make_stamped(now, p_frame, c_frame, xyz, q))
            else:
                # 降级：只发末端帧 arm_x_base_link → arm_x_link_6
                result = robot.forward_kinematics(joints_deg)
                if result:
                    pos = result['position']
                    rx, ry, rz = result['euler_rad']
                    cx, sx = math.cos(rx / 2), math.sin(rx / 2)
                    cy, sy = math.cos(ry / 2), math.sin(ry / 2)
                    cz, sz = math.cos(rz / 2), math.sin(rz / 2)
                    q = np.array([
                        cx * cy * cz + sx * sy * sz,
                        sx * cy * cz - cx * sy * sz,
                        cx * sy * cz + sx * cy * sz,
                        cx * cy * sz - sx * sy * cz,
                    ])
                    transforms.append(_make_stamped(
                        now, f'arm_{arm_ch}_base_link',
                        f'arm_{arm_ch}_link_6', pos, q))

        self._dynamic_br.sendTransform(transforms)


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
