"""
三臂机器人控制节点 (sim-only 插值)

功能：
- 订阅 /joint_states 获取实时状态 (23关节)
- 发布 /joint_command 发送控制指令 (23关节)
- 订阅 /target_joints 接收目标位置 (23关节，兼容19)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from .arm_controller import ArmController
from .joint_names import JOINT_NAMES_LIST, TOTAL_JOINT_COUNT


class TriarmControllerNode(Node):
    """三臂机器人控制节点"""

    def __init__(self):
        super().__init__('triarm_controller')

        # 声明参数
        self.declare_parameter('namespace', '')
        self.declare_parameter('mode', 'sim')
        self.declare_parameter('joint_velocity', 30.0)
        self.declare_parameter('publish_rate', 50.0)

        # 获取参数
        ns = self.get_parameter('namespace').value
        self._mode = self.get_parameter('mode').value
        velocity = self.get_parameter('joint_velocity').value
        rate = self.get_parameter('publish_rate').value

        # 创建控制器
        self.controller = ArmController(
            joint_velocity=velocity,
            publish_rate=rate
        )
        self.controller.set_command_callback(self._on_command)
        self.controller.set_motion_complete_callback(self._on_complete)

        # 话题名称
        prefix = f'{ns}/' if ns else '/'
        cmd_topic = f'{prefix}joint_command'
        state_topic = f'{prefix}joint_states'
        target_topic = f'{prefix}target_joints'

        # 发布者
        self.cmd_pub = self.create_publisher(JointState, cmd_topic, 10)

        # 订阅者
        self.state_sub = self.create_subscription(
            JointState, state_topic, self._state_callback, 10)

        # 目标输入话题
        self.target_sub = self.create_subscription(
            Float64MultiArray, target_topic, self._target_callback, 10)

        # 定时器
        period = 1.0 / rate
        self.timer = self.create_timer(period, self._timer_callback)

        self.get_logger().info(f'控制节点已启动 (mode={self._mode})')
        self.get_logger().info(f'  订阅状态: {state_topic}')
        self.get_logger().info(f'  发布指令: {cmd_topic}')
        self.get_logger().info(f'  目标输入: {target_topic}')

    def _state_callback(self, msg: JointState):
        if msg.name and msg.position:
            self.controller.update_current_state(
                list(msg.name), list(msg.position))

    def _target_callback(self, msg: Float64MultiArray):
        """处理目标位置输入 (支持23关节，向后兼容19关节)"""
        n = len(msg.data)
        if n == TOTAL_JOINT_COUNT:
            self.controller.set_target_positions(list(msg.data), in_degrees=True)
        elif n == 19:
            # 向后兼容: 19关节数据，夹爪保持当前目标值
            targets = list(msg.data) + self.controller.target_positions[19:]
            self.controller.set_target_positions(targets, in_degrees=True)
        else:
            self.get_logger().warn(
                f'目标位置数量错误: 需要{TOTAL_JOINT_COUNT}或19个，收到{n}个')
            return

        if self.controller.start_motion():
            self.get_logger().info('收到目标位置，开始运动')
        else:
            self.get_logger().warn('等待关节状态数据...')

    def _timer_callback(self):
        self.controller.step()

    def _on_command(self, positions: list):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES_LIST
        msg.position = positions
        self.cmd_pub.publish(msg)

    def _on_complete(self):
        self.get_logger().info('运动完成')


def main(args=None):
    rclpy.init(args=args)
    node = TriarmControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
