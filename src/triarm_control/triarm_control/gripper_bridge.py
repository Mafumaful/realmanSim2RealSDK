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
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray


# ─── Modbus 夹爪控制 (real 模式, 基于 crt_ctag2f90c) ───

# 控制寄存器地址 (可读写, 功能码 06/10)
_REG_ENABLE = 0x0100
_REG_POS_HIGH = 0x0102
_REG_SPEED = 0x0104
_REG_FORCE = 0x0105
_REG_ACCEL = 0x0106
_REG_DEACCEL = 0x0107
_REG_TRIGGER = 0x0108

# 反馈寄存器地址 (只读, 功能码 03)
_REG_FORCE_REACHED = 0x0601      # 力矩到达 (0/1)
_REG_POSITION_REACHED = 0x0602   # 位置到达 (0/1)
_REG_READY = 0x0604              # 准备完成: 力矩到达 OR 位置到达 (0/1)
_REG_POS_FB_HIGH = 0x0609        # 实时反馈位置 高16位
_REG_SPEED_FB = 0x060B           # 实时反馈转速
_REG_CURRENT_FB = 0x060C         # 实时反馈电流
_REG_ALARM = 0x0612              # 报警信息 (bit flags)

# 报警位掩码 (0x0612)
ALARM_OVER_TEMP = 0x01       # 过温警报
ALARM_STALL = 0x02           # 堵转警报
ALARM_OVER_SPEED = 0x04      # 超速警报
ALARM_INIT_FAULT = 0x08      # 初始化故障
ALARM_OVER_LIMIT = 0x10      # 超限检测警报
ALARM_GRIP_DROP = 0x20       # 夹取掉落警报


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

    # ─── 读取方法 (反馈寄存器, 功能码 03) ───

    def _read_reg(self, reg) -> int:
        """读取单个 16 位寄存器"""
        with self._lock:
            return self._instrument.read_register(reg, functioncode=3)

    def _read_long(self, reg) -> int:
        """读取 32 位值 (2 个连续寄存器, high<<16 + low)"""
        with self._lock:
            return self._instrument.read_long(reg, functioncode=3)

    def read_position(self) -> int:
        """读取实时反馈位置 (0x0609-0x060A, 32位)
        Returns: 位置值, 失败返回 -1
        """
        if not self._instrument:
            return -1
        try:
            return self._read_long(_REG_POS_FB_HIGH)
        except Exception as e:
            if self._logger:
                self._logger.warn(f'Modbus读取位置失败: {e}')
            return -1

    def read_alarm(self) -> int:
        """读取报警信息 (0x0612, bit flags)
        Returns: 报警位掩码, 失败返回 -1
        """
        if not self._instrument:
            return -1
        try:
            return self._read_reg(_REG_ALARM)
        except Exception as e:
            if self._logger:
                self._logger.warn(f'Modbus读取报警失败: {e}')
            return -1

    def is_motion_done(self) -> bool:
        """读取准备完成标志 (0x0604): 力矩到达 OR 位置到达"""
        if not self._instrument:
            return False
        try:
            return self._read_reg(_REG_READY) == 1
        except Exception:
            return False

    def is_force_reached(self) -> bool:
        """读取力矩到达标志 (0x0601): 夹住物体"""
        if not self._instrument:
            return False
        try:
            return self._read_reg(_REG_FORCE_REACHED) == 1
        except Exception:
            return False

    def is_position_reached(self) -> bool:
        """读取位置到达标志 (0x0602): 到达目标位置(空夹)"""
        if not self._instrument:
            return False
        try:
            return self._read_reg(_REG_POSITION_REACHED) == 1
        except Exception:
            return False

    def wait_motion_done(self, timeout: float = 5.0) -> bool:
        """轮询等待运动完成 (0x0604=1)
        Returns: True=运动完成, False=超时
        """
        if not self._instrument:
            return False
        start = time.time()
        while time.time() - start < timeout:
            try:
                if self._read_reg(_REG_READY) == 1:
                    return True
            except Exception:
                pass
            time.sleep(0.05)  # 50ms 轮询间隔
        return False

    def get_grip_status(self) -> dict:
        """运动完成后读取夹持状态
        Returns: {'force': bool, 'position': bool, 'alarm': int}
            force=True + position=False → 夹住物体
            force=False + position=True → 空夹(到达目标位置无阻力)
        """
        try:
            force = self._read_reg(_REG_FORCE_REACHED) == 1
            position = self._read_reg(_REG_POSITION_REACHED) == 1
            alarm = self._read_reg(_REG_ALARM)
            return {'force': force, 'position': position, 'alarm': alarm}
        except Exception as e:
            if self._logger:
                self._logger.warn(f'Modbus读取夹持状态失败: {e}')
            return None

    def is_holding(self) -> bool:
        """检查是否仍在夹持物体
        判断逻辑: 力矩到达=True 且 无夹取掉落警报
        Returns: True=持有, False=未持有, None=读取失败
        """
        if not self._instrument:
            return None
        try:
            alarm = self._read_reg(_REG_ALARM)
            if alarm & ALARM_GRIP_DROP:
                return False
            return self._read_reg(_REG_FORCE_REACHED) == 1
        except Exception:
            return None

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
        self.declare_parameter('namespace', 'robot')
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

        # per-arm 互斥锁: 防止同臂 _exec_real_gripper 并发执行
        self._arm_locks = {'arm_a': threading.Lock(), 'arm_b': threading.Lock()}

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

        # real 模式参数
        self.declare_parameter('motion_timeout', 5.0)
        self._motion_timeout = self.get_parameter('motion_timeout').value

        # 结果反馈发布器 (per-arm, 供 GripperController 等待完成)
        self._result_pubs = {}
        # 持有状态发布器 (per-arm, 供 GripperController.check_holding 使用)
        self._holding_pubs = {}
        for arm_name in ['arm_a', 'arm_b']:
            self._result_pubs[arm_name] = self.create_publisher(
                Bool, f'{prefix}{arm_name}/gripper_result', 10)
            self._holding_pubs[arm_name] = self.create_publisher(
                Bool, f'{prefix}{arm_name}/gripper_holding', 10)

        # 订阅 /gripper_target (Float64MultiArray[4]: L1, L11, R1, R11 弧度)
        self._gripper_target_sub = self.create_subscription(
            Float64MultiArray, f'{prefix}gripper_target',
            self._on_gripper_target, 10)

        # real 模式: 周期性读取夹持状态 (2Hz)
        if self._mode == 'real' and self._grippers:
            self._holding_timer = self.create_timer(
                0.5, self._check_holding_periodic)

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

        # real 模式: 线程化 Modbus 操作 (避免阻塞 ROS 回调)
        for arm_name in changed_arms:
            angle = arm_angles[arm_name]
            threading.Thread(
                target=self._exec_real_gripper,
                args=(arm_name, angle), daemon=True).start()

    def _exec_real_gripper(self, arm_name: str, angle: float):
        """real 模式: 发送 Modbus → 等待运动完成 → 判断夹持结果 → 发布
        在独立线程中执行，不阻塞 ROS 回调。
        per-arm Lock 保证同一臂不会并发执行。
        """
        with self._arm_locks[arm_name]:
            self._exec_real_gripper_locked(arm_name, angle)

    def _exec_real_gripper_locked(self, arm_name: str, angle: float):
        """_exec_real_gripper 的实际逻辑 (已持有 per-arm Lock)"""
        if arm_name not in self._grippers:
            result = Bool()
            result.data = False
            self._result_pubs[arm_name].publish(result)
            return

        gripper = self._grippers[arm_name]
        ret = gripper.move_to(angle)

        if ret is None:
            # Modbus 位置未变化 (整数舍入后相同)，直接成功
            result = Bool()
            result.data = True
            self._result_pubs[arm_name].publish(result)
            return

        if not ret:
            # Modbus 写入失败
            result = Bool()
            result.data = False
            self._result_pubs[arm_name].publish(result)
            return

        # 等待运动完成 (轮询 0x0604)
        done = gripper.wait_motion_done(timeout=self._motion_timeout)
        if not done:
            self.get_logger().warn(
                f'[{arm_name}] 夹爪运动超时 ({self._motion_timeout}s)')
            result = Bool()
            result.data = False
            self._result_pubs[arm_name].publish(result)
            return

        # 读取夹持状态
        status = gripper.get_grip_status()
        if status is None:
            # 读取失败，保守返回成功 (命令已发送且运动完成)
            self.get_logger().warn(f'[{arm_name}] 夹持状态读取失败')
            result = Bool()
            result.data = True
            self._result_pubs[arm_name].publish(result)
            return

        is_closing = angle > 0.5  # 闭合动作
        if is_closing:
            # 闭合: force_reached=True → 夹住物体, position_reached=True → 空夹
            success = status['force']
            if not success:
                self.get_logger().info(
                    f'[{arm_name}] 闭合完成但未夹住物体 '
                    f'(force={status["force"]}, pos={status["position"]})')
        else:
            # 张开: position_reached=True → 到位
            success = status['position'] or status['force']

        # 报警检查
        if status['alarm']:
            alarm = status['alarm']
            alarm_names = []
            if alarm & ALARM_GRIP_DROP:
                alarm_names.append('夹取掉落')
            if alarm & ALARM_STALL:
                alarm_names.append('堵转')
            if alarm & ALARM_OVER_TEMP:
                alarm_names.append('过温')
            if alarm_names:
                self.get_logger().warn(
                    f'[{arm_name}] 夹爪报警: {", ".join(alarm_names)} '
                    f'(0x{alarm:02X})')

        result = Bool()
        result.data = success
        self._result_pubs[arm_name].publish(result)

        self.get_logger().info(
            f'[{arm_name}] 夹爪动作完成 '
            f'(angle={angle:.2f}, force={status["force"]}, '
            f'pos={status["position"]}, result={success})')

    def _check_holding_periodic(self):
        """周期性读取夹持状态 (real 模式, 2Hz)
        在独立线程中执行 Modbus 读取，避免阻塞 ROS executor。
        使用 non-blocking acquire 跳过正在执行动作的臂。
        """
        threading.Thread(
            target=self._check_holding_worker, daemon=True).start()

    def _check_holding_worker(self):
        """_check_holding_periodic 的工作线程"""
        for arm_name, gripper in self._grippers.items():
            if not gripper.is_connected:
                continue
            # non-blocking: 如果臂正在执行动作，跳过本次检查
            if not self._arm_locks[arm_name].acquire(blocking=False):
                continue
            try:
                holding = gripper.is_holding()
                if holding is not None:
                    msg = Bool()
                    msg.data = holding
                    self._holding_pubs[arm_name].publish(msg)
            finally:
                self._arm_locks[arm_name].release()

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
