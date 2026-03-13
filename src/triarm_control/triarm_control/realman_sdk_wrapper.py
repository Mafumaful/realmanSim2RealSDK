#!/usr/bin/env python3
"""
RealMan RM65 官方SDK封装 - 模式感知统一接口

双模式：
  sim:  RM65Robot IK/FK 解算 (支持自定义base坐标系) + SDK Algo 辅助功能
  real: RM65Robot IK/FK 解算 + SDK Algo 辅助功能 + RoboticArm TCP 控制真实机械臂

调用方只需:
  sdk = RealManSDKWrapper(
      ip, port, arm_id, mode='sim',
      base_position=[x, y, z],
      base_orientation_deg=[rx, ry, rz],
      d6_mm=172.5
  )
  sdk.connect()          # 内部根据 mode 自动初始化
  sdk.inverse_kinematics(...)  # 使用 RM65Robot 进行 IK 解算
  sdk.forward_kinematics(...)  # 使用 RM65Robot 进行 FK 解算
  sdk.movel(...)         # real模式走TCP, sim模式返回NOT_CONNECTED

依赖: pip install Robotic_Arm
"""

import math
import threading
from enum import Enum
from typing import List, Optional

import numpy as np


class SDKMotionResult(Enum):
    SUCCESS = 0
    FAILED = 1
    TIMEOUT = 2
    NOT_CONNECTED = 3
    IK_FAILED = 4

class RealManAlgo:
    """RealMan 算法库封装 - 使用 RM65Robot 进行 IK/FK 解算

    基于 rm65_robot.RM65Robot，支持自定义 base 坐标系，
    纯本地计算，不需要TCP连接。
    同时保留原始 SDK Algo 用于辅助功能（pos2matrix/matrix2pos/pose_move）。
    """

    def __init__(self, base_position=None, base_orientation_deg=None, d6_mm=172.5):
        """
        初始化算法库

        Args:
            base_position: [x, y, z] 单位 m，机械臂base相对转盘的位置
            base_orientation_deg: [rx, ry, rz] ZYX欧拉角，单位 deg，机械臂base相对转盘的姿态
            d6_mm: 末端 d6 参数，单位 mm，默认 172.5 (RM65-6F)
        """
        self._robot = None  # RM65Robot 实例（用于 IK/FK）
        self._algo = None   # 原始 SDK Algo 实例（用于辅助功能）
        self._initialized = False
        self._lock = threading.Lock()
        self._base_position = base_position or [0.0, 0.0, 0.0]
        self._base_orientation_deg = base_orientation_deg or [0.0, 0.0, 0.0]
        self._d6_mm = d6_mm

    def initialize(self) -> bool:
        """初始化算法库"""
        if self._initialized:
            return True
        try:
            # 初始化 RM65Robot (用于 IK/FK)
            from .rm65_robot import RM65Robot
            self._robot = RM65Robot(
                base_position=self._base_position,
                base_orientation_deg=self._base_orientation_deg,
                d6_mm=self._d6_mm,
            )
            print(f'[Algo] RM65Robot 初始化成功, base_pos={self._base_position}, base_ori={self._base_orientation_deg}')

            # 初始化原始 SDK Algo (用于辅助功能)
            try:
                from Robotic_Arm.rm_robot_interface import (
                    Algo,
                    rm_robot_arm_model_e,
                    rm_force_type_e,
                )
                self._algo = Algo(
                    rm_robot_arm_model_e.RM_MODEL_RM_65_E,
                    rm_force_type_e.RM_MODEL_RM_B_E,
                )
                print(f'[Algo] SDK Algo 初始化成功 (用于辅助功能)')
            except Exception as e:
                print(f'[Algo] SDK Algo 初始化失败: {e}，辅助功能不可用')

            self._initialized = True
            return True
        except ImportError as e:
            print(f'[Algo] 未安装 Robotic_Arm SDK (pip install Robotic_Arm): {e}')
            return False
        except Exception as e:
            print(f'[Algo] 初始化失败: {e}')
            return False

    @property
    def is_ready(self) -> bool:
        return self._initialized

    def inverse_kinematics(
        self,
        q_ref_deg: List[float],
        target_pose: List[float],
        flag: int = 1,
    ) -> Optional[List[float]]:
        """逆运动学解算

        Args:
            q_ref_deg: 参考关节角度 [j1..j6] (角度制)
            target_pose: 目标位姿 [x,y,z,rx,ry,rz]
                         位置(m), 姿态为欧拉角(弧度)
            flag: 保留参数，兼容旧接口

        Returns:
            关节角度列表 [j1..j6] (角度制), 失败返回 None
        """
        if not self._initialized:
            return None

        with self._lock:
            try:
                # RM65Robot.inverse_kinematics 需要的参数:
                # - target_position: [x,y,z] m
                # - target_orientation_deg: [rx,ry,rz] deg (可选)
                # - current_joint_angles_deg: [j1..j6] deg
                target_position = target_pose[:3]
                target_orientation_deg = [math.degrees(target_pose[3]),
                                         math.degrees(target_pose[4]),
                                         math.degrees(target_pose[5])]

                result = self._robot.inverse_kinematics(
                    target_position=target_position,
                    target_orientation_deg=target_orientation_deg,
                    current_joint_angles_deg=q_ref_deg
                )

                if result and result.get('success'):
                    return result['joint_angles_deg']

                print(f'[Algo] IK失败: pose={target_pose[:3]}')
                return None
            except Exception as e:
                print(f'[Algo] IK异常: {e}')
                return None

    def pos2matrix(self, pose: List[float]) -> Optional[np.ndarray]:
        """位姿转4x4齐次变换矩阵

        Args:
            pose: [x,y,z,rx,ry,rz] 位置(m)+姿态(rad)
        Returns:
            np.ndarray(4,4) 齐次变换矩阵, 失败返回 None

        使用原始 SDK Algo 实现"""
        if not self._initialized or not self._algo:
            return None
        with self._lock:
            try:
                result = self._algo.rm_algo_pos2matrix(pose)
                # SDK可能返回 rm_matrix_t 对象或 (code, data) 元组
                if hasattr(result, 'data'):
                    return np.array(result.data, dtype=np.float64).reshape(4, 4)
                elif isinstance(result, (list, tuple)) and result[0] == 0:
                    return np.array(result[1], dtype=np.float64).reshape(4, 4)
                return None
            except Exception as e:
                print(f'[Algo] pos2matrix异常: {e}')
                return None

    def matrix2pos(self, matrix: np.ndarray, flag: int = 1) -> Optional[List[float]]:
        """4x4齐次变换矩阵转位姿

        Args:
            matrix: np.ndarray(4,4) 齐次变换矩阵
            flag: 1=欧拉角输出
        Returns:
            [x,y,z,rx,ry,rz] 位置(m)+姿态(rad), 失败返回 None

        使用原始 SDK Algo 实现"""
        if not self._initialized or not self._algo:
            return None
        with self._lock:
            try:
                from Robotic_Arm.rm_ctypes_wrap import rm_matrix_t
                import ctypes
                matrix_obj = rm_matrix_t()
                flat = np.asarray(matrix, dtype=np.float32).flatten()
                matrix_obj.data = (ctypes.c_float * 16)(*flat)
                result = self._algo.rm_algo_matrix2pos(matrix_obj, flag)
                # SDK可能返回列表或 (code, data) 元组
                if isinstance(result, (list, tuple)) and len(result) == 6:
                    return list(result)
                elif isinstance(result, (list, tuple)) and result[0] == 0:
                    return list(result[1])
                return None
            except Exception as e:
                print(f'[Algo] matrix2pos异常: {e}')
                return None

    def pose_move(self, pose: List[float], delta: List[float], mode: int = 1) -> Optional[List[float]]:
        """位姿叠加 (delta角度单位为度)

        使用原始 SDK Algo 实现"""
        if not self._initialized or not self._algo:
            return None
        with self._lock:
            try:
                result = self._algo.rm_algo_pose_move(pose, delta, mode)
                if isinstance(result, (list, tuple)) and result[0] == 0:
                    return list(result[1])
                return None
            except Exception as e:
                print(f'[Algo] pose_move异常: {e}')
                return None

    def forward_kinematics(
        self,
        joints_deg: List[float],
        flag: int = 1,
    ) -> Optional[List[float]]:
        """正运动学解算

        Args:
            joints_deg: 关节角度 [j1..j6] (角度制)
            flag: 保留参数，兼容旧接口

        Returns:
            位姿 [x,y,z,rx,ry,rz] (姿态单位为弧度) 或 None
        """
        if not self._initialized:
            return None

        with self._lock:
            try:
                # RM65Robot.forward_kinematics 返回字典:
                # {'position': [x,y,z], 'euler_deg': [rx,ry,rz], 'euler_rad': [rx,ry,rz]}
                result = self._robot.forward_kinematics(joints_deg)
                if result is not None:
                    # 返回 [x,y,z,rx,ry,rz]，姿态单位为弧度
                    return result['position'] + result['euler_rad']
                return None
            except Exception as e:
                print(f'[Algo] FK异常: {e}')
                return None


class RealManSDKWrapper:
    """RealMan RM65 官方SDK封装 - 模式感知统一接口

    每臂一个实例，内部根据 mode 自动路由：
      sim:  RM65Robot IK/FK 解算 (支持自定义base坐标系)
      real: RM65Robot IK/FK 解算 + RoboticArm TCP 控制

    调用方无需关心 Algo 细节，只需 connect() + 调用接口。
    支持自定义机械臂 base 坐标系参数，提高 IK/FK 精度。
    """

    DOF = 6

    def __init__(self, ip: str, port: int = 8080, arm_id: str = 'A',
                 mode: str = 'real', algo: RealManAlgo = None,
                 base_position: List[float] = None,
                 base_orientation_deg: List[float] = None,
                 d6_mm: float = 172.5):
        """
        Args:
            ip: 机械臂IP地址
            port: 端口号 (默认8080)
            arm_id: 臂标识 (A/B/S)
            mode: 'sim' 或 'real'
            algo: 共享的Algo实例 (可选，不传则内部创建)
            base_position: [x, y, z] 单位 m，机械臂base相对转盘的位置
            base_orientation_deg: [rx, ry, rz] ZYX欧拉角，单位 deg，机械臂base相对转盘的姿态
            d6_mm: 末端 d6 参数，单位 mm，默认 172.5 (RM65-6F)
        """
        self._ip = ip
        self._port = port
        self._arm_id = arm_id
        self._mode = mode
        self._tag = f'[SDK:{arm_id}:{mode}]'
        self._lock = threading.Lock()

        self._robot = None
        self._tcp_connected = False

        # Algo: 如果未传入则创建新实例（使用 base 参数）
        if algo is not None:
            self._algo = algo
        else:
            self._algo = RealManAlgo(
                base_position=base_position,
                base_orientation_deg=base_orientation_deg,
                d6_mm=d6_mm
            )

    @property
    def mode(self) -> str:
        return self._mode

    @property
    def is_ready(self) -> bool:
        """SDK是否就绪 (根据模式判断)
        sim:  Algo 已初始化即可
        real: TCP 已连接 + Algo 已初始化
        """
        if self._mode == 'sim':
            return self._algo.is_ready
        else:
            return self._tcp_connected and self._algo.is_ready

    @property
    def is_connected(self) -> bool:
        """TCP连接状态 (real模式有意义)"""
        return self._tcp_connected

    # ─── 连接管理 (模式感知) ───

    def connect(self) -> bool:
        """连接SDK - 根据模式自动初始化

        sim模式:  初始化 Algo (纯本地IK/FK)
        real模式: 初始化 Algo + TCP连接真实机械臂
        """
        # 1. 初始化 Algo (两种模式都需要)
        algo_ok = self._ensure_algo()

        if self._mode == 'sim':
            if algo_ok:
                print(f'{self._tag} sim模式就绪 (Algo IK/FK可用)')
            else:
                print(f'{self._tag} sim模式Algo初始化失败!')
            return algo_ok

        # 2. real模式: 额外建立TCP连接
        tcp_ok = self._connect_tcp()
        if not tcp_ok:
            print(f'{self._tag} real模式TCP连接失败!')
            return False

        if not algo_ok:
            print(f'{self._tag} real模式TCP已连接，但Algo初始化失败 (IK不可用)')

        print(f'{self._tag} real模式就绪 '
              f'(TCP={tcp_ok}, Algo={algo_ok})')
        return tcp_ok

    def disconnect(self):
        """断开连接"""
        if self._robot and self._tcp_connected:
            try:
                self._robot.rm_delete_robot_arm()
            except Exception:
                pass
            self._tcp_connected = False
            print(f'{self._tag} TCP已断开')

    def _ensure_algo(self) -> bool:
        """确保 Algo 已初始化"""
        if self._algo.is_ready:
            return True
        return self._algo.initialize()

    def _connect_tcp(self) -> bool:
        """建立TCP连接到真实机械臂"""
        try:
            from Robotic_Arm.rm_robot_interface import (
                RoboticArm, rm_thread_mode_e
            )
            self._robot = RoboticArm(
                rm_thread_mode_e.RM_TRIPLE_MODE_E)
            handle = self._robot.rm_create_robot_arm(
                self._ip, self._port)
            if handle.id == -1:
                print(f'{self._tag} TCP连接失败: {self._ip}:{self._port}')
                return False

            self._tcp_connected = True
            print(f'{self._tag} TCP已连接 {self._ip}:{self._port}')
            return True
        except ImportError:
            print(f'{self._tag} 未安装 Robotic_Arm SDK')
            return False
        except Exception as e:
            print(f'{self._tag} TCP连接异常: {e}')
            return False

    def set_arm_run_mode(self, sim: bool = True) -> bool:
        """设置机械臂运行模式 (需要TCP连接)
        Args:
            sim: True=仿真模式(rm_set_arm_run_mode(1)),
                 False=真实模式(rm_set_arm_run_mode(0))
        """
        if not self._tcp_connected:
            return False
        with self._lock:
            try:
                mode_val = 1 if sim else 0
                ret = self._robot.rm_set_arm_run_mode(mode_val)
                print(f'{self._tag} 设置运行模式: '
                      f'{"仿真" if sim else "真实"}, ret={ret}')
                return ret == 0
            except Exception as e:
                print(f'{self._tag} 设置运行模式失败: {e}')
                return False

    # ─── IK/FK (通过SDK Algo解算) ───

    def inverse_kinematics(
        self, x: float, y: float, z: float,
        rx: float, ry: float, rz: float,
        q_ref: Optional[List[float]] = None
    ) -> Optional[List[float]]:
        """逆运动学解算 (SDK Algo)

        两种模式都通过SDK Algo解算，不区分sim/real。

        Args:
            x,y,z: 目标位置 (米)
            rx,ry,rz: 目标欧拉角 (弧度)
            q_ref: 参考关节角度 (弧度), None则使用当前角度或零位

        Returns:
            关节角度列表 (弧度), 失败返回 None
        """
        if not self._algo.is_ready:
            if not self._ensure_algo():
                return None

        # 参考角度: 传入值 > 真实臂当前值 > 零位
        if q_ref is not None:
            q_ref_deg = [math.degrees(q) for q in q_ref]
        elif self._tcp_connected:
            joints = self.get_joint_positions()
            if joints:
                q_ref_deg = [math.degrees(q) for q in joints]
            else:
                q_ref_deg = [0.0] * 6
        else:
            q_ref_deg = [0.0] * 6

        # SDK Algo 位置单位为米 (m)，姿态单位为弧度 (rad)
        target_pose = [x, y, z, rx, ry, rz]
        result_deg = self._algo.inverse_kinematics(
            q_ref_deg, target_pose, flag=1)

        if result_deg is None:
            return None
        return [math.radians(d) for d in result_deg]

    def forward_kinematics(
        self, joints: List[float]
    ) -> Optional[dict]:
        """正运动学解算 (SDK Algo)

        Args:
            joints: 关节角度 (弧度)
        Returns:
            {'x','y','z','rx','ry','rz'} 或 None
        """
        if not self._algo.is_ready:
            if not self._ensure_algo():
                return None

        joints_deg = [math.degrees(j) for j in joints]
        result = self._algo.forward_kinematics(joints_deg, flag=1)
        if result is None:
            return None
        return {
            'x': result[0], 'y': result[1], 'z': result[2],
            'rx': result[3], 'ry': result[4], 'rz': result[5],
        }

    # ─── 状态查询 (需要TCP) ───

    def get_joint_positions(self) -> Optional[List[float]]:
        """获取当前关节角度 (弧度)"""
        if not self._tcp_connected:
            return None
        with self._lock:
            try:
                ret = self._robot.rm_get_joint_degree()
                if ret[0] == 0:
                    return [math.radians(d) for d in ret[1]]
                return None
            except Exception as e:
                print(f'{self._tag} 获取关节角度失败: {e}')
                return None

    def get_current_pose(self) -> Optional[dict]:
        """获取当前末端位姿
        Returns: {'x','y','z','rx','ry','rz'} 位置(米)+欧拉角(弧度)
        """
        if not self._tcp_connected:
            return None
        with self._lock:
            try:
                ret = self._robot.rm_get_current_arm_state()
                if ret[0] == 0:
                    pose = ret[1]['pose']
                    return {
                        'x': pose[0], 'y': pose[1], 'z': pose[2],
                        'rx': pose[3], 'ry': pose[4], 'rz': pose[5],
                    }
                return None
            except Exception as e:
                print(f'{self._tag} 获取位姿失败: {e}')
                return None

    # ─── 运动控制 (需要TCP) ───

    def movej(self, joints: List[float], speed: int = 20,
              block: bool = True) -> SDKMotionResult:
        """关节运动
        Args:
            joints: 目标关节角度 (弧度)
            speed: 速度百分比 1-100
        """
        if not self._tcp_connected:
            return SDKMotionResult.NOT_CONNECTED
        with self._lock:
            try:
                joints_deg = [math.degrees(j) for j in joints]
                ret = self._robot.rm_movej(
                    joints_deg, speed, 0, 0, block)
                if ret == 0:
                    return SDKMotionResult.SUCCESS
                print(f'{self._tag} MoveJ失败: ret={ret}')
                return SDKMotionResult.FAILED
            except Exception as e:
                print(f'{self._tag} MoveJ异常: {e}')
                return SDKMotionResult.FAILED

    def movel(self, x: float, y: float, z: float,
              rx: float, ry: float, rz: float,
              speed: int = 20, block: bool = True) -> SDKMotionResult:
        """直线运动
        Args:
            x,y,z: 目标位置 (米)
            rx,ry,rz: 目标欧拉角 (弧度)
            speed: 速度百分比 1-100
        """
        if not self._tcp_connected:
            return SDKMotionResult.NOT_CONNECTED
        with self._lock:
            try:
                pose = [x, y, z, rx, ry, rz]
                ret = self._robot.rm_movel(
                    pose, speed, 0, 0, block)
                if ret == 0:
                    return SDKMotionResult.SUCCESS
                print(f'{self._tag} MoveL失败: ret={ret}')
                return SDKMotionResult.FAILED
            except Exception as e:
                print(f'{self._tag} MoveL异常: {e}')
                return SDKMotionResult.FAILED

    def movej_p(self, x: float, y: float, z: float,
                rx: float, ry: float, rz: float,
                speed: int = 20, block: bool = True) -> SDKMotionResult:
        """关节空间运动到目标位姿 (MoveJ_P)"""
        if not self._tcp_connected:
            return SDKMotionResult.NOT_CONNECTED
        with self._lock:
            try:
                pose = [x, y, z, rx, ry, rz]
                ret = self._robot.rm_movej_p(
                    pose, speed, 0, 0, block)
                if ret == 0:
                    return SDKMotionResult.SUCCESS
                print(f'{self._tag} MoveJP失败: ret={ret}')
                return SDKMotionResult.FAILED
            except Exception as e:
                print(f'{self._tag} MoveJP异常: {e}')
                return SDKMotionResult.FAILED

    def movep(self, x: float, y: float, z: float,
              rx: float, ry: float, rz: float,
              speed: int = 20, block: bool = True) -> SDKMotionResult:
        """点到点运动 (MoveP)"""
        if not self._tcp_connected:
            return SDKMotionResult.NOT_CONNECTED
        with self._lock:
            try:
                pose = [x, y, z, rx, ry, rz]
                ret = self._robot.rm_movep_canfd(pose, 0, 0, 0)
                if ret == 0:
                    return SDKMotionResult.SUCCESS
                print(f'{self._tag} MoveP失败: ret={ret}')
                return SDKMotionResult.FAILED
            except Exception as e:
                print(f'{self._tag} MoveP异常: {e}')
                return SDKMotionResult.FAILED

    def stop(self) -> bool:
        """急停"""
        if not self._tcp_connected:
            return False
        with self._lock:
            try:
                ret = self._robot.rm_set_arm_stop()
                return ret == 0
            except Exception:
                return False

    # ─── 夹爪控制 (需要TCP) ───

    def gripper_pick_on(self, speed: int = 500,
                        force: int = 200,
                        block: bool = True,
                        timeout: int = 5000) -> SDKMotionResult:
        """力控持续夹取"""
        if not self._tcp_connected:
            return SDKMotionResult.NOT_CONNECTED
        with self._lock:
            try:
                ret = self._robot.rm_set_gripper_pick_on(
                    speed, force, block, timeout)
                if ret == 0:
                    return SDKMotionResult.SUCCESS
                return SDKMotionResult.FAILED
            except Exception as e:
                print(f'{self._tag} 夹爪pick_on失败: {e}')
                return SDKMotionResult.FAILED

    def gripper_pick(self, speed: int = 500,
                     force: int = 200,
                     block: bool = True,
                     timeout: int = 5000) -> SDKMotionResult:
        """力控夹取 (非持续)"""
        if not self._tcp_connected:
            return SDKMotionResult.NOT_CONNECTED
        with self._lock:
            try:
                ret = self._robot.rm_set_gripper_pick(
                    speed, force, block, timeout)
                if ret == 0:
                    return SDKMotionResult.SUCCESS
                return SDKMotionResult.FAILED
            except Exception as e:
                print(f'{self._tag} 夹爪pick失败: {e}')
                return SDKMotionResult.FAILED

    def gripper_set_position(self, position: int,
                             block: bool = True,
                             timeout: int = 5000) -> SDKMotionResult:
        """夹爪位置控制
        Args:
            position: 0(全闭) ~ 1000(全开)
        """
        if not self._tcp_connected:
            return SDKMotionResult.NOT_CONNECTED
        with self._lock:
            try:
                ret = self._robot.rm_set_gripper_position(
                    position, block, timeout)
                if ret == 0:
                    return SDKMotionResult.SUCCESS
                return SDKMotionResult.FAILED
            except Exception as e:
                print(f'{self._tag} 夹爪位置控制失败: {e}')
                return SDKMotionResult.FAILED
