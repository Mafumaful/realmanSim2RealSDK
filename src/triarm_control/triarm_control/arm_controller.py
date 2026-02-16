"""
机械臂核心控制器 (仅用于 sim 模式)

负责：
- 23关节状态管理 (19臂关节 + 4夹爪关节)
- 线性插值运动控制
- 与ROS2通信解耦

注意：real 模式由官方 SDK 自带平滑，不经过此插值器
"""

import math
from typing import List, Optional, Callable
from .joint_names import JOINT_NAMES_LIST, JOINT_LIMITS, TOTAL_JOINT_COUNT


class ArmController:
    """机械臂控制器核心类 (sim-only 线性/S型曲线插值)"""

    def __init__(self, joint_velocity: float = 30.0, publish_rate: float = 50.0,
                 enable_smooth: bool = True, acceleration: float = 100.0):
        """
        初始化控制器

        Args:
            joint_velocity: 关节运动速度 (度/秒)
            publish_rate: 控制频率 (Hz)
            enable_smooth: 是否启用平滑处理
            acceleration: 加速度限制 (度/秒²)
        """
        self.base_velocity = joint_velocity  # 基准速度
        self.joint_velocity = joint_velocity
        self.velocity_multiplier = 1.0  # 速度倍率
        self.publish_rate = publish_rate
        self.enable_smooth = enable_smooth
        self.acceleration = acceleration
        self.num_joints = TOTAL_JOINT_COUNT  # 23

        # 状态变量
        self.current_positions: List[float] = [0.0] * self.num_joints
        self.target_positions: List[float] = [0.0] * self.num_joints
        self.cmd_positions: List[float] = [0.0] * self.num_joints
        self.current_velocities: List[float] = [0.0] * self.num_joints  # 当前速度

        # 控制标志
        self.is_moving = False
        self.has_received_state = False
        self.is_initialized = False

        # 回调函数
        self._on_command_ready: Optional[Callable[[List[float]], None]] = None
        self._on_motion_complete: Optional[Callable[[], None]] = None

    @property
    def joint_names(self) -> List[str]:
        return JOINT_NAMES_LIST.copy()

    @property
    def joint_limits(self) -> dict:
        return JOINT_LIMITS.copy()

    def set_command_callback(self, callback: Callable[[List[float]], None]):
        self._on_command_ready = callback

    def set_motion_complete_callback(self, callback: Callable[[], None]):
        self._on_motion_complete = callback

    def set_smooth_enabled(self, enabled: bool):
        """设置是否启用平滑处理"""
        self.enable_smooth = enabled
        if not enabled:
            self.current_velocities = [0.0] * self.num_joints

    def set_velocity_mode(self, mode: str):
        """设置速度模式 (slow=0.5x, normal=1.0x, fast=2.0x)"""
        mode_map = {'slow': 0.5, 'normal': 1.0, 'fast': 2.0}
        if mode in mode_map:
            self.velocity_multiplier = mode_map[mode]
            self.joint_velocity = self.base_velocity * self.velocity_multiplier

    def update_current_state(self, names: List[str], positions: List[float]):
        """
        更新当前关节状态

        Args:
            names: 关节名称列表
            positions: 对应的位置列表 (弧度)
        """
        for i, name in enumerate(names):
            if i < len(positions) and name in JOINT_NAMES_LIST:
                idx = JOINT_NAMES_LIST.index(name)
                self.current_positions[idx] = positions[i]
        self.has_received_state = True

    def set_target_positions(self, positions: List[float], in_degrees: bool = True):
        """
        设置目标位置

        Args:
            positions: 目标位置列表 (23个关节，兼容19个)
            in_degrees: 是否为角度制，默认True
        """
        for i, pos in enumerate(positions):
            if i >= self.num_joints:
                break
            if in_degrees:
                pos = math.radians(pos)
            name = JOINT_NAMES_LIST[i]
            limits = JOINT_LIMITS[name]
            pos = max(limits[0], min(limits[1], pos))
            self.target_positions[i] = pos

    def set_single_joint_target(self, joint_idx: int, position: float, in_degrees: bool = True):
        if joint_idx < 0 or joint_idx >= self.num_joints:
            raise ValueError(f"关节索引超出范围: {joint_idx}")
        if in_degrees:
            position = math.radians(position)
        name = JOINT_NAMES_LIST[joint_idx]
        limits = JOINT_LIMITS[name]
        position = max(limits[0], min(limits[1], position))
        self.target_positions[joint_idx] = position

    def start_motion(self) -> bool:
        if not self.has_received_state:
            return False
        if not self.is_initialized:
            self.cmd_positions = list(self.current_positions)
            self.is_initialized = True
        self.is_moving = True
        return True

    def stop_motion(self):
        self.is_moving = False

    def reset_targets(self):
        self.target_positions = [0.0] * self.num_joints

    def step(self) -> bool:
        """执行一步插值计算"""
        if not self.is_moving:
            return False

        if self.enable_smooth:
            return self._step_smooth()
        else:
            return self._step_linear()

    def _step_linear(self) -> bool:
        """线性插值"""
        dt = 1.0 / self.publish_rate
        max_delta = math.radians(self.joint_velocity) * dt

        all_reached = True

        for i in range(self.num_joints):
            diff = self.target_positions[i] - self.cmd_positions[i]
            if abs(diff) > 0.001:
                all_reached = False
                if abs(diff) > max_delta:
                    diff = max_delta if diff > 0 else -max_delta
                self.cmd_positions[i] += diff

        if self._on_command_ready:
            self._on_command_ready(list(self.cmd_positions))

        if all_reached:
            self.is_moving = False
            if self._on_motion_complete:
                self._on_motion_complete()

        return self.is_moving

    def _step_smooth(self) -> bool:
        """S型曲线插值（平滑加减速）"""
        dt = 1.0 / self.publish_rate
        max_velocity = math.radians(self.joint_velocity)
        max_acceleration = math.radians(self.acceleration)

        all_reached = True

        for i in range(self.num_joints):
            diff = self.target_positions[i] - self.cmd_positions[i]

            if abs(diff) > 0.001:
                all_reached = False

                # 计算期望速度方向
                desired_velocity = max_velocity if diff > 0 else -max_velocity

                # 计算减速所需距离
                decel_distance = (self.current_velocities[i] ** 2) / (2 * max_acceleration)

                # 判断是否需要减速
                if abs(diff) <= abs(decel_distance):
                    # 减速阶段
                    if abs(self.current_velocities[i]) > 0.001:
                        accel = -max_acceleration if self.current_velocities[i] > 0 else max_acceleration
                    else:
                        accel = 0
                else:
                    # 加速或匀速阶段
                    velocity_diff = desired_velocity - self.current_velocities[i]
                    if abs(velocity_diff) > max_acceleration * dt:
                        accel = max_acceleration if velocity_diff > 0 else -max_acceleration
                    else:
                        accel = velocity_diff / dt

                # 更新速度
                self.current_velocities[i] += accel * dt

                # 限制速度
                self.current_velocities[i] = max(-max_velocity, min(max_velocity, self.current_velocities[i]))

                # 更新位置
                delta = self.current_velocities[i] * dt
                if abs(delta) > abs(diff):
                    delta = diff
                    self.current_velocities[i] = 0

                self.cmd_positions[i] += delta
            else:
                self.current_velocities[i] = 0

        if self._on_command_ready:
            self._on_command_ready(list(self.cmd_positions))

        if all_reached:
            self.is_moving = False
            self.current_velocities = [0.0] * self.num_joints
            if self._on_motion_complete:
                self._on_motion_complete()

        return self.is_moving

    def get_current_positions_deg(self) -> List[float]:
        return [math.degrees(p) for p in self.current_positions]

    def get_cmd_positions_deg(self) -> List[float]:
        return [math.degrees(p) for p in self.cmd_positions]

    def get_target_positions_deg(self) -> List[float]:
        return [math.degrees(p) for p in self.target_positions]
