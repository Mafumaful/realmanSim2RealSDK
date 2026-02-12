#!/usr/bin/env python3
"""
夹爪桥接节点

双模式：
  real: 调用官方SDK夹爪接口 (通过 RealManSDKWrapper)
  sim:  暂时只发布 result=True (mock)，Isaac Sim 夹爪暂未配置

为 A/B 臂各创建一套 rm_driver 兼容夹爪话题：
  {arm_name}/rm_driver/set_gripper_pick_on_cmd → result
  {arm_name}/rm_driver/set_gripper_pick_cmd → result
  {arm_name}/rm_driver/set_gripper_position_cmd → result
"""

import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rm_ros_interfaces.msg import Gripperpick, Gripperset

from .realman_sdk_wrapper import RealManSDKWrapper, SDKMotionResult


class GripperBridge:
    """单臂夹爪桥接器"""

    def __init__(self, node: Node, arm_name: str, mode: str,
                 sdk: RealManSDKWrapper = None,
                 sim_gripper_delay: float = 0.5):
        self.node = node
        self.arm_name = arm_name
        self.mode = mode
        self._sdk = sdk
        self._sim_gripper_delay = sim_gripper_delay
        self.logger = node.get_logger()
        self._tag = f'[GripperBridge:{arm_name}]'

        prefix = f'{arm_name}/'

        # 结果发布器
        self._pick_on_result_pub = node.create_publisher(
            Bool, f'{prefix}rm_driver/set_gripper_pick_on_result', 10)
        self._pick_result_pub = node.create_publisher(
            Bool, f'{prefix}rm_driver/set_gripper_pick_result', 10)
        self._position_result_pub = node.create_publisher(
            Bool, f'{prefix}rm_driver/set_gripper_position_result', 10)

        # 命令订阅器
        self._pick_on_sub = node.create_subscription(
            Gripperpick, f'{prefix}rm_driver/set_gripper_pick_on_cmd',
            self._on_pick_on, 10)
        self._pick_sub = node.create_subscription(
            Gripperpick, f'{prefix}rm_driver/set_gripper_pick_cmd',
            self._on_pick, 10)
        self._position_sub = node.create_subscription(
            Gripperset, f'{prefix}rm_driver/set_gripper_position_cmd',
            self._on_position, 10)

        self.logger.info(f'{self._tag} 初始化完成 (mode={mode})')

    def _on_pick_on(self, msg: Gripperpick):
        threading.Thread(
            target=self._exec_pick_on, args=(msg,), daemon=True).start()

    def _on_pick(self, msg: Gripperpick):
        threading.Thread(
            target=self._exec_pick, args=(msg,), daemon=True).start()

    def _on_position(self, msg: Gripperset):
        threading.Thread(
            target=self._exec_position, args=(msg,), daemon=True).start()

    def _exec_pick_on(self, msg: Gripperpick):
        result = Bool()
        if self.mode == 'real' and self._sdk and self._sdk.is_connected:
            ret = self._sdk.gripper_pick_on(
                msg.speed, msg.force, msg.block, msg.timeout)
            result.data = (ret == SDKMotionResult.SUCCESS)
        else:
            # sim模式: mock成功 + 延时模拟
            self.logger.info(
                f'{self._tag} [sim] pick_on (speed={msg.speed}, '
                f'force={msg.force}) delay={self._sim_gripper_delay}s')
            time.sleep(self._sim_gripper_delay)
            result.data = True
        self._pick_on_result_pub.publish(result)

    def _exec_pick(self, msg: Gripperpick):
        result = Bool()
        if self.mode == 'real' and self._sdk and self._sdk.is_connected:
            ret = self._sdk.gripper_pick(
                msg.speed, msg.force, msg.block, msg.timeout)
            result.data = (ret == SDKMotionResult.SUCCESS)
        else:
            self.logger.info(
                f'{self._tag} [sim] pick (speed={msg.speed}, '
                f'force={msg.force}) delay={self._sim_gripper_delay}s')
            time.sleep(self._sim_gripper_delay)
            result.data = True
        self._pick_result_pub.publish(result)

    def _exec_position(self, msg: Gripperset):
        result = Bool()
        if self.mode == 'real' and self._sdk and self._sdk.is_connected:
            ret = self._sdk.gripper_set_position(
                msg.position, msg.block, msg.timeout)
            result.data = (ret == SDKMotionResult.SUCCESS)
        else:
            self.logger.info(
                f'{self._tag} [sim] position({msg.position}) '
                f'delay={self._sim_gripper_delay}s')
            time.sleep(self._sim_gripper_delay)
            result.data = True
        self._position_result_pub.publish(result)


class GripperBridgeNode(Node):
    """夹爪桥接节点 - 管理 A/B 臂夹爪"""

    def __init__(self):
        super().__init__('gripper_bridge_node')

        self.declare_parameter('mode', 'sim')
        self.declare_parameter('sim_gripper_delay', 0.5)
        self.declare_parameter('arm_a.ip', '192.168.1.18')
        self.declare_parameter('arm_a.port', 8080)
        self.declare_parameter('arm_b.ip', '192.168.1.19')
        self.declare_parameter('arm_b.port', 8080)

        mode = self.get_parameter('mode').value
        sim_delay = self.get_parameter('sim_gripper_delay').value
        self.get_logger().info(
            f'=== GripperBridgeNode 启动 (mode={mode}) ===')

        self._bridges = {}
        for arm_name in ['arm_a', 'arm_b']:
            sdk = None
            if mode == 'real':
                ip = self.get_parameter(f'{arm_name}.ip').value
                port = self.get_parameter(f'{arm_name}.port').value
                arm_id = arm_name[-1].upper()
                sdk = RealManSDKWrapper(ip, port, arm_id, mode='real')
                if not sdk.connect():
                    self.get_logger().error(
                        f'[{arm_name}] SDK连接失败，夹爪不可用')
                    sdk = None

            bridge = GripperBridge(self, arm_name, mode, sdk,
                                   sim_gripper_delay=sim_delay)
            self._bridges[arm_name] = bridge

        self.get_logger().info('=== GripperBridgeNode 就绪 ===')

    def destroy_node(self):
        for bridge in self._bridges.values():
            if bridge._sdk and bridge._sdk.is_connected:
                bridge._sdk.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GripperBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
