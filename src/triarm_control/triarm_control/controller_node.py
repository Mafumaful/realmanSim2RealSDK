"""
三臂机器人控制节点

功能：
- 订阅 /joint_states 获取实时状态
- 发布 /joint_command 发送控制指令
- 订阅 /target_joints 接收矩阵形式的目标位置
- 订阅 /gripper_command 接收夹爪控制指令
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
import math

from .arm_controller import ArmController
from .joint_names import JOINT_NAMES_LIST


class TriarmControllerNode(Node):
    """三臂机器人控制节点"""

    def __init__(self):
        super().__init__('triarm_controller')

        # 声明参数
        self.declare_parameter('namespace', '')
        self.declare_parameter('mode', 'sim')
        self.declare_parameter('joint_velocity', 30.0)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('enable_smooth', True)
        self.declare_parameter('acceleration', 100.0)
        self.declare_parameter('velocity_mode', 'normal')

        # 夹爪参数
        self.declare_parameter('left_gripper.open_angle', 0.0)
        self.declare_parameter('left_gripper.close_angle', 30.0)
        self.declare_parameter('right_gripper.open_angle', 0.0)
        self.declare_parameter('right_gripper.close_angle', 30.0)

        # 获取参数
        ns = self.get_parameter('namespace').value
        self._mode = self.get_parameter('mode').value
        velocity = self.get_parameter('joint_velocity').value
        rate = self.get_parameter('publish_rate').value
        enable_smooth = self.get_parameter('enable_smooth').value
        acceleration = self.get_parameter('acceleration').value
        velocity_mode = self.get_parameter('velocity_mode').value

        # 夹爪参数
        self.left_open = self.get_parameter('left_gripper.open_angle').value
        self.left_close = self.get_parameter('left_gripper.close_angle').value
        self.right_open = self.get_parameter('right_gripper.open_angle').value
        self.right_close = self.get_parameter('right_gripper.close_angle').value

        # 创建控制器
        self.controller = ArmController(
            joint_velocity=velocity,
            publish_rate=rate,
            enable_smooth=enable_smooth,
            acceleration=acceleration
        )
        self.controller.set_velocity_mode(velocity_mode)
        self.controller.set_command_callback(self._on_command)
        self.controller.set_motion_complete_callback(self._on_complete)

        # 添加参数变化回调（支持热加载）
        self.add_on_set_parameters_callback(self._parameters_callback)

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

        # 矩阵输入话题
        self.target_sub = self.create_subscription(
            Float64MultiArray, target_topic, self._target_callback, 10)

        # 控制参数话题
        from std_msgs.msg import Bool, String
        self.smooth_sub = self.create_subscription(
            Bool, f'{prefix}enable_smooth', self._smooth_callback, 10)
        self.velocity_mode_sub = self.create_subscription(
            String, f'{prefix}velocity_mode', self._velocity_mode_callback, 10)

        # 夹爪控制话题
        self.gripper_sub = self.create_subscription(
            String, f'{prefix}gripper_control', self._gripper_callback, 10)

        # 定时器
        period = 1.0 / rate
        self.timer = self.create_timer(period, self._timer_callback)

        self.get_logger().info(f'控制节点已启动 (mode={self._mode})')
        self.get_logger().info(f'  订阅状态: {state_topic}')
        self.get_logger().info(f'  发布指令: {cmd_topic}')
        self.get_logger().info(f'  目标输入: {target_topic}')

    def _state_callback(self, msg: JointState):
        """处理关节状态"""
        if msg.name and msg.position:
            self.controller.update_current_state(
                list(msg.name), list(msg.position))

    def _target_callback(self, msg: Float64MultiArray):
        """处理矩阵形式的目标位置输入"""
        if len(msg.data) != 19:
            self.get_logger().warn(
                f'目标位置数量错误: 需要19个，收到{len(msg.data)}个')
            return

        self.controller.set_target_positions(list(msg.data), in_degrees=True)

        if self.controller.start_motion():
            self.get_logger().info('收到目标位置，开始运动')
        else:
            self.get_logger().warn('等待关节状态数据...')

    def _smooth_callback(self, msg):
        """处理平滑处理开关"""
        self.controller.set_smooth_enabled(msg.data)
        status = "启用" if msg.data else "禁用"
        self.get_logger().info(f'平滑处理已{status}')

    def _velocity_mode_callback(self, msg):
        """处理速度模式变化"""
        self.controller.set_velocity_mode(msg.data)
        self.get_logger().info(f'速度模式: {msg.data}')

    def _gripper_callback(self, msg):
        """处理夹爪控制指令

        指令格式: "left_open", "left_close", "right_open", "right_close"
        """
        command = msg.data.lower()

        # 构建夹爪指令消息
        gripper_msg = JointState()
        gripper_msg.header.stamp = self.get_clock().now().to_msg()

        if command == 'left_open':
            gripper_msg.name = ['joint_L1', 'joint_L11']
            angle_rad = math.radians(self.left_open)
            gripper_msg.position = [angle_rad, -angle_rad]  # L1和L11方向相反
            self.get_logger().info(f'左夹爪打开: {self.left_open}°')

        elif command == 'left_close':
            gripper_msg.name = ['joint_L1', 'joint_L11']
            angle_rad = math.radians(self.left_close)
            gripper_msg.position = [angle_rad, -angle_rad]  # L1和L11方向相反
            self.get_logger().info(f'左夹爪闭合: {self.left_close}°')

        elif command == 'right_open':
            gripper_msg.name = ['joint_R1', 'joint_R11']
            angle_rad = math.radians(self.right_open)
            gripper_msg.position = [angle_rad, -angle_rad]  # R1和R11方向相反
            self.get_logger().info(f'右夹爪打开: {self.right_open}°')

        elif command == 'right_close':
            gripper_msg.name = ['joint_R1', 'joint_R11']
            angle_rad = math.radians(self.right_close)
            gripper_msg.position = [angle_rad, -angle_rad]  # R1和R11方向相反
            self.get_logger().info(f'右夹爪闭合: {self.right_close}°')

        else:
            self.get_logger().warn(f'未知的夹爪指令: {command}')
            return

        # 发布夹爪指令
        self.cmd_pub.publish(gripper_msg)

    def _parameters_callback(self, params):
        """参数热加载回调"""
        from rcl_interfaces.msg import SetParametersResult

        for param in params:
            if param.name == 'left_gripper.open_angle':
                self.left_open = param.value
                self.get_logger().info(f'更新左夹爪打开角度: {self.left_open}°')
            elif param.name == 'left_gripper.close_angle':
                self.left_close = param.value
                self.get_logger().info(f'更新左夹爪闭合角度: {self.left_close}°')
            elif param.name == 'right_gripper.open_angle':
                self.right_open = param.value
                self.get_logger().info(f'更新右夹爪打开角度: {self.right_open}°')
            elif param.name == 'right_gripper.close_angle':
                self.right_close = param.value
                self.get_logger().info(f'更新右夹爪闭合角度: {self.right_close}°')

        return SetParametersResult(successful=True)

    def _timer_callback(self):
        """定时执行控制步进"""
        self.controller.step()

    def _on_command(self, positions: list):
        """发布关节指令"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES_LIST
        msg.position = positions
        self.cmd_pub.publish(msg)

    def _on_complete(self):
        """运动完成回调"""
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
