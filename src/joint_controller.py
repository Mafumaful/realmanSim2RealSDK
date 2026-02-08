"""
三臂机器人关节控制器

支持仿真模式和真实机械臂模式的关节角度控制。
"""

import sys
import os
import numpy as np
from typing import Optional, Tuple, List, Dict

# 添加SDK路径
SDK_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'Python'))
if SDK_PATH not in sys.path:
    sys.path.insert(0, SDK_PATH)

from .config import (
    JOINT_LIMITS, JOINT_NAMES,
    ROW_PLATFORM, ROW_ARM_A, ROW_ARM_B, ROW_ARM_S,
    DEFAULT_ROBOT_IPS, DEFAULT_ROBOT_PORT
)


class JointController:
    """
    三臂机器人关节控制器

    支持通过4x7矩阵控制19个关节的角度。

    矩阵格式：
        行0: [D1, 0, 0, 0, 0, 0, 0]     # 平台旋转
        行1: [A1, A2, A3, A4, A5, A6, 0] # 机械臂A
        行2: [B1, B2, B3, B4, B5, B6, 0] # 机械臂B
        行3: [S1, S2, S3, S4, S5, S6, 0] # 机械臂S
    """

    def __init__(self, mode: str = 'sim', robot_ips: Optional[Dict[str, str]] = None):
        """
        初始化关节控制器

        Args:
            mode: 运行模式，'sim' 仿真模式，'real' 真实机械臂
            robot_ips: 真实模式下的机械臂IP配置字典
        """
        self.mode = mode
        self.robot_ips = robot_ips or DEFAULT_ROBOT_IPS
        self.robot_port = DEFAULT_ROBOT_PORT

        # 当前关节角度 (4x7矩阵)
        self._current_positions = np.zeros((4, 7), dtype=np.float64)

        # 机械臂连接句柄 (真实模式)
        self._robot_handles = {}

        # 初始化
        self._initialize()

    def _initialize(self):
        """根据模式初始化"""
        if self.mode == 'sim':
            self._init_simulation()
        elif self.mode == 'real':
            self._init_real_robots()
        else:
            raise ValueError(f"不支持的模式: {self.mode}，请使用 'sim' 或 'real'")

    def _init_simulation(self):
        """初始化仿真模式"""
        print("[JointController] 仿真模式初始化完成")

    def _init_real_robots(self):
        """初始化真实机械臂连接"""
        from Robotic_Arm.rm_robot_interface import (
            RoboticArm, rm_thread_mode_e
        )

        arm_keys = ['arm_a', 'arm_b', 'arm_s']
        for key in arm_keys:
            ip = self.robot_ips.get(key)
            if ip:
                robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
                handle = robot.rm_create_robot_arm(ip, self.robot_port)
                if handle.contents.id != -1:
                    self._robot_handles[key] = robot
                    print(f"[JointController] {key} 连接成功: {ip}")
                else:
                    print(f"[JointController] {key} 连接失败: {ip}")

    def validate_limits(self, matrix: np.ndarray) -> Tuple[bool, List[str]]:
        """
        验证关节角度是否在限位范围内

        Args:
            matrix: 4x7 关节角度矩阵

        Returns:
            (是否有效, 超限关节列表)
        """
        violations = []
        limit_groups = ['platform', 'arm_a', 'arm_b', 'arm_s']

        for row, group in enumerate(limit_groups):
            limits = JOINT_LIMITS[group]
            names = JOINT_NAMES[row]

            for col, name in enumerate(names):
                if name is None:
                    continue
                value = matrix[row, col]
                min_val, max_val = limits[name]

                if value < min_val or value > max_val:
                    violations.append(
                        f"{name}: {value:.2f}° (范围: {min_val}~{max_val}°)"
                    )

        return len(violations) == 0, violations

    def set_joint_positions(self, matrix: np.ndarray, validate: bool = True) -> bool:
        """
        设置目标关节角度

        Args:
            matrix: 4x7 关节角度矩阵，单位：度
            validate: 是否验证限位

        Returns:
            是否成功
        """
        matrix = np.asarray(matrix, dtype=np.float64)

        if matrix.shape != (4, 7):
            raise ValueError(f"矩阵形状必须为 (4, 7)，当前为 {matrix.shape}")

        if validate:
            valid, violations = self.validate_limits(matrix)
            if not valid:
                print(f"[JointController] 关节超限: {violations}")
                return False

        if self.mode == 'sim':
            return self._set_positions_sim(matrix)
        else:
            return self._set_positions_real(matrix)

    def _set_positions_sim(self, matrix: np.ndarray) -> bool:
        """仿真模式设置关节角度"""
        self._current_positions = matrix.copy()
        print(f"[SIM] 关节角度已更新")
        return True

    def _set_positions_real(self, matrix: np.ndarray) -> bool:
        """真实模式设置关节角度"""
        success = True

        # 机械臂A
        if 'arm_a' in self._robot_handles:
            joints_a = matrix[ROW_ARM_A, :6].tolist()
            ret = self._robot_handles['arm_a'].rm_movej(
                joints_a, v=20, r=0, connect=0, block=1
            )
            if ret != 0:
                print(f"[REAL] arm_a 运动失败: {ret}")
                success = False

        # 机械臂B
        if 'arm_b' in self._robot_handles:
            joints_b = matrix[ROW_ARM_B, :6].tolist()
            ret = self._robot_handles['arm_b'].rm_movej(
                joints_b, v=20, r=0, connect=0, block=1
            )
            if ret != 0:
                print(f"[REAL] arm_b 运动失败: {ret}")
                success = False

        # 机械臂S
        if 'arm_s' in self._robot_handles:
            joints_s = matrix[ROW_ARM_S, :6].tolist()
            ret = self._robot_handles['arm_s'].rm_movej(
                joints_s, v=20, r=0, connect=0, block=1
            )
            if ret != 0:
                print(f"[REAL] arm_s 运动失败: {ret}")
                success = False

        if success:
            self._current_positions = matrix.copy()

        return success

    def get_joint_positions(self) -> np.ndarray:
        """
        获取当前关节角度

        Returns:
            4x7 关节角度矩阵
        """
        if self.mode == 'real':
            self._update_positions_from_real()
        return self._current_positions.copy()

    def _update_positions_from_real(self):
        """从真实机械臂读取当前关节角度"""
        arm_map = {
            'arm_a': ROW_ARM_A,
            'arm_b': ROW_ARM_B,
            'arm_s': ROW_ARM_S,
        }

        for key, row in arm_map.items():
            if key in self._robot_handles:
                ret, joints = self._robot_handles[key].rm_get_joint_degree()
                if ret == 0 and joints:
                    self._current_positions[row, :6] = joints[:6]

    def switch_mode(self, mode: str):
        """
        切换运行模式

        Args:
            mode: 'sim' 或 'real'
        """
        if mode == self.mode:
            return

        # 断开现有连接
        self.disconnect()

        self.mode = mode
        self._initialize()

    def disconnect(self):
        """断开所有机械臂连接"""
        for key, robot in self._robot_handles.items():
            robot.rm_delete_robot_arm()
            print(f"[JointController] {key} 已断开")
        self._robot_handles.clear()

    def __del__(self):
        """析构时断开连接"""
        self.disconnect()
