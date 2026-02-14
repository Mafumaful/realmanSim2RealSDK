#!/usr/bin/env python3
"""
夹爪桥接节点 (ROS2)

双模式：
  sim:  夹爪关节由 unified_arm_node 合并到 /target_joints 控制，
        本节点负责发布 /{arm_name}/gripper_result 反馈给 GripperController
  real: 订阅 /gripper_target (Float64MultiArray[4])，
        将关节角度转换为 Modbus RTU 命令控制 crt_ctag2f90c 夹爪，
        并发布 /{arm_name}/gripper_result 反馈

夹爪关节映射 (crt_ctag2f90c):
  arm_a → joint_L1 (index 19), joint_L11 (index 20, 与L1反向)
  arm_b → joint_R1 (index 21), joint_R11 (index 22, 与R1反向)
  关节范围: 0~1 rad (0=全开, 1=全闭)

Modbus 位置映射 (参考 GUI2Robot_90.py):
  joint_angle (0~1) → modbus_position = int((1 - angle) * 9000)
  0 rad (全开) → 9000, 1 rad (全闭) → 0
"""

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray


# ─── Modbus 夹爪控制 (real 模式, 基于 crt_ctag2f90c) ───

# 寄存器地址
_REG_ENABLE = 0x0100
_REG_POS_HIGH = 0x0102
_REG_SPEED = 0x0104
_REG_FORCE = 0x0105
_REG_ACCEL = 0x0106
_REG_DEACCEL = 0x0107
_REG_TRIGGER = 0x0108


class ModbusGripper:
    """Modbus RTU 夹爪控制器 (crt_ctag2f90c)"""

    def __init__(self, port: str = '/dev/ttyUSB0', baud: int = 115200,
                 slave_addr: int = 1):
        self._port = port
        self._baud = baud
        self._slave_addr = slave_addr
        self._instrument = None
        self._lock = threading.Lock()
        # 防重复: 记录上次发送的位置，相同位置不重复发送
        self._last_sent = None
        self._logger = None

    def connect(self, logger=None) -> bool:
        try:
            import minimalmodbus
            self._instrument = minimalmodbus.Instrument(self._port, self._slave_addr)
            self._instrument.serial.baudrate = self._baud
            self._instrument.serial.timeout = 1
            self._write_reg(_REG_ENABLE, 1)
            self._write_reg(_REG_SPEED, 100)
            self._write_reg(_REG_FORCE, 100)
            self._write_reg(_REG_ACCEL, 100)
            self._write_reg(_REG_DEACCEL, 100)
            return True
        except Exception as e:
            if logger:
                logger.error(f'Modbus连接失败 ({self._port}): {e}')
            self._instrument = None
            return False

    @property
    def is_connected(self) -> bool:
        return self._instrument is not None

    def _write_reg(self, reg, value):
        with self._lock:
            self._instrument.write_register(reg, value, functioncode=6)

    def _write_long(self, reg, value):
        with self._lock:
            self._instrument.write_long(reg, value)

    def set_speed(self, speed: int):
        if self._instrument:
            self._write_reg(_REG_SPEED, max(0, min(100, speed)))

    def set_force(self, force: int):
        if self._instrument:
            self._write_reg(_REG_FORCE, max(0, min(100, force)))

    def move_to(self, joint_angle: float):
        """关节角度 → Modbus 位置并发送
        joint_angle: 0~1 rad (0=全开, 1=全闭)
        modbus_position: 0~9000 (0=全闭, 9000=全开)
        Returns: True=命令已发送, None=位置未变化(跳过), False=发送失败
        """
        if not self._instrument:
            return False
        angle = max(0.0, min(1.0, joint_angle))
        pos = int((1.0 - angle) * 9000)

        # 防抖滤波:
        #   新位置 (pos != _last_sent) → 直接发送 (支持离散命令)
        #   相同位置重复 → 过滤 (防止 GUI 滑块抖动重复写 Modbus)
        if pos == self._last_sent:
            return None  # 与上次相同，跳过

        try:
            self._write_long(_REG_POS_HIGH, pos)
            self._write_reg(_REG_TRIGGER, 1)
            self._last_sent = pos
            return True
        except Exception as e:
            if self._logger:
                self._logger.error(f'Modbus写入失败: {e}')
            return False

    def close(self):
        if self._instrument:
            try:
                self._instrument.serial.close()
            except Exception:
                pass
            self._instrument = None


# ─── 夹爪桥接节点 ───

class GripperBridgeNode(Node):
    """夹爪桥接节点

    sim 模式: 夹爪关节由 unified_arm_node 合并到 /target_joints 控制，
              本节点负责发布 /{arm_name}/gripper_result 反馈给 GripperController
    real 模式: 订阅 /gripper_target，转换为 Modbus RTU 命令
    """

    def __init__(self):
        super().__init__('gripper_bridge_node')

        self.declare_parameter('mode', 'sim')
        self.declare_parameter('namespace', '')
        # real 模式串口配置
        self.declare_parameter('gripper_a.serial_port', '/dev/ttyUSB0')
        self.declare_parameter('gripper_a.baud_rate', 115200)
        self.declare_parameter('gripper_a.slave_addr', 1)
        self.declare_parameter('gripper_b.serial_port', '/dev/ttyUSB1')
        self.declare_parameter('gripper_b.baud_rate', 115200)
        self.declare_parameter('gripper_b.slave_addr', 1)

        self._mode = self.get_parameter('mode').value
        ns = self.get_parameter('namespace').value
        prefix = f'{ns}/' if ns else '/'

        self.get_logger().info(
            f'=== GripperBridgeNode 启动 (mode={self._mode}) ===')

        # 上次收到的目标角度 (用于判断哪个臂的目标发生了变化)
        self._last_target = {'arm_a': None, 'arm_b': None}

        # Modbus 夹爪 (real 模式)
        self._grippers = {}
        if self._mode == 'real':
            for arm_name, key in [('arm_a', 'gripper_a'), ('arm_b', 'gripper_b')]:
                port = self.get_parameter(f'{key}.serial_port').value
                baud = self.get_parameter(f'{key}.baud_rate').value
                addr = self.get_parameter(f'{key}.slave_addr').value
                gripper = ModbusGripper(port, baud, addr)
                gripper._logger = self.get_logger()
                if gripper.connect(logger=self.get_logger()):
                    self.get_logger().info(
                        f'[{arm_name}] Modbus夹爪已连接 ({port})')
                    self._grippers[arm_name] = gripper
                else:
                    self.get_logger().error(
                        f'[{arm_name}] Modbus夹爪连接失败 ({port})')

        # 结果反馈发布器 (per-arm, 供 GripperController 等待完成)
        self._result_pubs = {}
        for arm_name in ['arm_a', 'arm_b']:
            self._result_pubs[arm_name] = self.create_publisher(
                Bool, f'{prefix}{arm_name}/gripper_result', 10)

        # 订阅 /gripper_target (Float64MultiArray[4]: L1, L11, R1, R11 弧度)
        self._gripper_target_sub = self.create_subscription(
            Float64MultiArray, f'{prefix}gripper_target',
            self._on_gripper_target, 10)

        self.get_logger().info('=== GripperBridgeNode 就绪 ===')

    def _on_gripper_target(self, msg: Float64MultiArray):
        """处理夹爪目标角度
        msg.data = [L1, L11, R1, R11] (弧度, 0=全开, 1=全闭)

        只给目标实际变化的臂发布 gripper_result，避免误触发对侧臂。
        sim 模式: unified_arm_node 处理关节控制，本节点仅发布 result 反馈
        real 模式: 转换为 Modbus 命令
        """
        if len(msg.data) != 4:
            self.get_logger().warn(
                f'gripper_target 长度错误: 需要4, 收到{len(msg.data)}')
            return

        # 提取主指角度，判断哪个臂的目标发生了变化
        arm_angles = {'arm_a': msg.data[0], 'arm_b': msg.data[2]}
        changed_arms = []
        for arm_name, angle in arm_angles.items():
            if self._last_target[arm_name] is None or \
               abs(angle - self._last_target[arm_name]) > 1e-6:
                changed_arms.append(arm_name)
                self._last_target[arm_name] = angle

        if not changed_arms:
            return  # 无变化，不发布任何 result

        if self._mode != 'real':
            # sim 模式: unified_arm_node 处理关节控制，
            # 仅给变化的臂发布 result
            for arm_name in changed_arms:
                result = Bool()
                result.data = True
                self._result_pubs[arm_name].publish(result)
            return

        # real 模式: 发送 Modbus，根据结果发布反馈
        for arm_name in changed_arms:
            angle = arm_angles[arm_name]
            if arm_name not in self._grippers:
                # 该臂夹爪未连接
                result = Bool()
                result.data = False
                self._result_pubs[arm_name].publish(result)
                continue

            ret = self._grippers[arm_name].move_to(angle)
            if ret is None:
                # Modbus 位置未变化 (整数舍入后相同)，无需发送，直接视为成功
                result = Bool()
                result.data = True
                self._result_pubs[arm_name].publish(result)
            else:
                result = Bool()
                result.data = bool(ret)
                self._result_pubs[arm_name].publish(result)

    def destroy_node(self):
        for gripper in self._grippers.values():
            gripper.close()
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
