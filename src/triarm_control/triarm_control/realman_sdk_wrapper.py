#!/usr/bin/env python3
"""
RealMan RM65 官方SDK封装

封装 Robotic_Arm 官方SDK，提供：
- TCP 连接管理
- IK/FK 解算 (通过官方 Algo)
- 关节运动 (MoveJ)
- 直线运动 (MoveL)
- 关节空间到位姿运动 (MoveJ_P)
- 夹爪控制
- 每臂独立实例

依赖: pip install Robotic_Arm
"""

import math
import threading
from enum import Enum
from typing import List, Optional, Tuple


class SDKMotionResult(Enum):
    SUCCESS = 0
    FAILED = 1
    TIMEOUT = 2
    NOT_CONNECTED = 3
    IK_FAILED = 4


class RealManSDKWrapper:
    """RealMan RM65 官方SDK封装 - 每臂一个实例"""

    DOF = 6
    # RM_65 标准版 (RM_B)
    ARM_MODEL = 'RM_B'

    def __init__(self, ip: str, port: int = 8080, arm_id: str = 'A'):
        """
        Args:
            ip: 机械臂IP地址
            port: 端口号 (默认8080)
            arm_id: 臂标识 (A/B/S)
        """
        self._ip = ip
        self._port = port
        self._arm_id = arm_id
        self._tag = f'[SDK:{arm_id}]'
        self._lock = threading.Lock()

        self._robot = None
        self._algo = None
        self._connected = False

    # ─── 连接管理 ───

    def connect(self) -> bool:
        """连接到机械臂"""
        try:
            from Robotic_Arm.rm_robot_interface import (
                RoboticArm, rm_thread_mode_e
            )
            self._robot = RoboticArm(
                rm_thread_mode_e.RM_TRIPLE_MODE_E)
            handle = self._robot.rm_create_robot_arm(
                self._ip, self._port)
            if handle.id == -1:
                print(f'{self._tag} 连接失败: {self._ip}:{self._port}')
                return False

            self._connected = True
            # 获取 Algo 句柄用于 IK/FK
            self._algo = self._robot
            print(f'{self._tag} 已连接 {self._ip}:{self._port}')
            return True
        except ImportError:
            print(f'{self._tag} 未安装 Robotic_Arm SDK')
            return False
        except Exception as e:
            print(f'{self._tag} 连接异常: {e}')
            return False

    def disconnect(self):
        """断开连接"""
        if self._robot and self._connected:
            try:
                self._robot.rm_delete_robot_arm()
            except Exception:
                pass
            self._connected = False
            print(f'{self._tag} 已断开')

    @property
    def is_connected(self) -> bool:
        return self._connected

    # ─── 状态查询 ───

    def get_joint_positions(self) -> Optional[List[float]]:
        """获取当前关节角度 (弧度)"""
        if not self._connected:
            return None
        with self._lock:
            try:
                ret = self._robot.rm_get_joint_degree()
                if ret[0] == 0:
                    # SDK返回角度制，转弧度
                    return [math.radians(d) for d in ret[1]]
                return None
            except Exception as e:
                print(f'{self._tag} 获取关节角度失败: {e}')
                return None

    def get_current_pose(self) -> Optional[dict]:
        """获取当前末端位姿
        Returns: {'x','y','z','rx','ry','rz'} 位置(米)+欧拉角(弧度)
        """
        if not self._connected:
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

    # ─── IK/FK 解算 ───

    def inverse_kinematics(
        self, x: float, y: float, z: float,
        rx: float, ry: float, rz: float,
        q_ref: Optional[List[float]] = None
    ) -> Optional[List[float]]:
        """逆运动学解算
        Args:
            x,y,z: 目标位置 (米)
            rx,ry,rz: 目标欧拉角 (弧度)
            q_ref: 参考关节角度 (弧度), None则使用当前角度
        Returns:
            关节角度列表 (弧度), 失败返回 None
        """
        if not self._connected:
            return None

        if q_ref is None:
            q_ref = self.get_joint_positions()
            if q_ref is None:
                return None

        with self._lock:
            try:
                # SDK 接口: 角度制输入
                q_ref_deg = [math.degrees(q) for q in q_ref]
                pose = [x, y, z, rx, ry, rz]
                ret = self._robot.rm_algo_inverse_kinematics(
                    q_ref_deg, pose, 1)  # flag=1: 欧拉角
                if ret[0] == 0:
                    return [math.radians(d) for d in ret[1]]
                return None
            except Exception as e:
                print(f'{self._tag} IK解算失败: {e}')
                return None

    def forward_kinematics(
        self, joints: List[float]
    ) -> Optional[dict]:
        """正运动学解算
        Args:
            joints: 关节角度 (弧度)
        Returns:
            {'x','y','z','rx','ry','rz'} 或 None
        """
        if not self._connected:
            return None
        with self._lock:
            try:
                joints_deg = [math.degrees(j) for j in joints]
                ret = self._robot.rm_algo_forward_kinematics(
                    joints_deg, 1)
                if ret[0] == 0:
                    p = ret[1]
                    return {
                        'x': p[0], 'y': p[1], 'z': p[2],
                        'rx': p[3], 'ry': p[4], 'rz': p[5],
                    }
                return None
            except Exception as e:
                print(f'{self._tag} FK解算失败: {e}')
                return None

    # ─── 运动控制 ───

    def movej(self, joints: List[float], speed: int = 20,
              block: bool = True) -> SDKMotionResult:
        """关节运动
        Args:
            joints: 目标关节角度 (弧度)
            speed: 速度百分比 1-100
            block: 是否阻塞
        """
        if not self._connected:
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
            block: 是否阻塞
        """
        if not self._connected:
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
        if not self._connected:
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

    def stop(self) -> bool:
        """急停"""
        if not self._connected:
            return False
        with self._lock:
            try:
                ret = self._robot.rm_set_arm_stop()
                return ret == 0
            except Exception:
                return False

    # ─── 夹爪控制 ───

    def gripper_pick_on(self, speed: int = 500,
                        force: int = 200,
                        block: bool = True,
                        timeout: int = 5000) -> SDKMotionResult:
        """力控持续夹取"""
        if not self._connected:
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
        if not self._connected:
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
        if not self._connected:
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
