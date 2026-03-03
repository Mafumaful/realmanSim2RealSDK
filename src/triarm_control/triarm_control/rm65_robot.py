"""
RM65 机械臂算法库
支持自定义 base 参数的正/逆运动学计算
"""

import math
import numpy as np
from Robotic_Arm.rm_robot_interface import (
    Algo,
    rm_robot_arm_model_e,
    rm_force_type_e,
    rm_dh_t,
    rm_inverse_kinematics_params_t,
)


class RM65Robot:
    """RM65 机械臂算法库，支持自定义 base 坐标系"""

    def __init__(
        self,
        base_position=None,
        base_orientation_deg=None,
        d6_mm=172.5,
    ):
        """
        初始化 RM65 机械臂

        参数
        ----
        base_position        : [x, y, z] 单位 m，机械臂base相对转盘的位置，默认 [0, 0, 0]
        base_orientation_deg : [rx, ry, rz] ZYX欧拉角，单位 deg，机械臂base相对转盘的姿态，默认 [0, 0, 0]
        d6_mm                : 末端 d6 参数，单位 mm，默认 172.5 (RM65-6F)

        注：base_position 和 base_orientation_deg 是机械臂base相对转盘的固定位姿
        """
        self.base_position = base_position or [0.0, 0.0, 0.0]
        self.base_orientation_deg = base_orientation_deg or [0.0, 0.0, 0.0]
        self.d6_mm = d6_mm
        self.turntable_angle_deg = 0.0  # 转盘旋转角度（绕z轴）

        # 构建 DH 参数
        mm = 1e-3
        dh = rm_dh_t(
            a     = [0, 0, 256*mm, 0, 0, 0],
            alpha = [0, 90, 0, 90, -90, 90],
            d     = [240.5*mm, 0, 0, 210*mm, 0, d6_mm*mm],
            offset= [0, 90, 90, 0, 0, 0],
        )

        # 初始化官方算法库
        self._algo = Algo(
            arm_model = rm_robot_arm_model_e.RM_MODEL_UNIVERSAL_E,
            force_type = rm_force_type_e.RM_MODEL_RM_B_E,
            arm_dof   = 6,
            dh        = dh,
        )

        # 设置逆解求解为遍历模式（提高求解成功率）
        self._algo.rm_algo_set_redundant_parameter_traversal_mode(True)

    def set_turntable_angle(self, angle_deg):
        """
        设置转盘旋转角度

        参数
        ----
        angle_deg : float, 转盘旋转角度（绕z轴），单位 deg
        """
        self.turntable_angle_deg = angle_deg

    def forward_kinematics(self, joint_angles_deg):
        """
        正运动学：关节角度 → 末端位姿（世界坐标系）

        参数
        ----
        joint_angles_deg : list[float], 6个关节角度，单位 deg

        返回
        ----
        dict:
            'position'   : [x, y, z] 单位 m
            'euler_deg'  : [rx, ry, rz] ZYX欧拉角，单位 deg
            'euler_rad'  : [rx, ry, rz] ZYX欧拉角，单位 rad
        """
        # 官方库计算（相对 base）
        pose_base = self._algo.rm_algo_forward_kinematics(joint_angles_deg, flag=1)

        # 转换到世界坐标系：世界 = 转盘 @ base相对转盘 @ 末端相对base
        T_turntable = self._build_turntable_transform()
        T_base_on_turntable = self._build_base_transform()
        T_end_base = self._pose_to_transform(pose_base)
        T_world = T_turntable @ T_base_on_turntable @ T_end_base

        x, y, z = T_world[0:3, 3]
        rx_r, ry_r, rz_r = self._rot_to_euler_zyx(T_world[0:3, 0:3])

        return {
            'position': [x, y, z],
            'euler_deg': [math.degrees(rx_r), math.degrees(ry_r), math.degrees(rz_r)],
            'euler_rad': [rx_r, ry_r, rz_r],
        }

    def inverse_kinematics(
        self,
        target_position,
        target_orientation_deg=None,
        current_joint_angles_deg=None,
    ):
        """
        逆运动学：末端位姿（世界坐标系）→ 关节角度

        参数
        ----
        target_position          : [x, y, z] 目标位置，单位 m
        target_orientation_deg   : [rx, ry, rz] 目标姿态，单位 deg
                                   若为 None，则使用当前姿态
        current_joint_angles_deg : list[float], 当前关节角度，单位 deg
                                   用于选择逆解分支，默认 [0]*6

        返回
        ----
        dict:
            'joint_angles_deg' : list[float], 6个关节角度，单位 deg
            'success'          : bool, 是否求解成功
        """
        if current_joint_angles_deg is None:
            current_joint_angles_deg = [0.0] * 6

        # 如果未指定姿态，使用当前关节角度对应的姿态
        if target_orientation_deg is None:
            current_pose = self.forward_kinematics(current_joint_angles_deg)
            target_orientation_deg = current_pose['euler_deg']

        # 构建世界坐标系下的目标位姿
        target_pose_world = list(target_position) + [
            math.radians(target_orientation_deg[0]),
            math.radians(target_orientation_deg[1]),
            math.radians(target_orientation_deg[2]),
        ]

        # 转换到 base 坐标系：base = (转盘 @ base相对转盘)^-1 @ 世界
        T_turntable = self._build_turntable_transform()
        T_base_on_turntable = self._build_base_transform()
        T_world_to_base = np.linalg.inv(T_turntable @ T_base_on_turntable)
        T_target_world = self._pose_to_transform(target_pose_world)
        T_target_base = T_world_to_base @ T_target_world

        # 提取 base 坐标系下的位姿
        x, y, z = T_target_base[0:3, 3]
        rx_r, ry_r, rz_r = self._rot_to_euler_zyx(T_target_base[0:3, 0:3])
        target_pose_base = [x, y, z, rx_r, ry_r, rz_r]

        # 构建逆解参数（当前关节角度、目标位姿、flag）
        ik_params = rm_inverse_kinematics_params_t(
            current_joint_angles_deg,
            target_pose_base,
            1
        )

        # 调用官方逆解
        result = self._algo.rm_algo_inverse_kinematics(ik_params)

        # 返回值是元组 (状态码, 关节角度列表)
        if result is not None and isinstance(result, tuple) and len(result) == 2:
            status_code, joint_angles = result
            # 状态码 0 表示成功
            if status_code == 0 and joint_angles and len(joint_angles) == 6:
                return {
                    'joint_angles_deg': list(joint_angles),
                    'success': True,
                }

        return {
            'joint_angles_deg': None,
            'success': False,
            'error': f'逆解失败，返回值: {result}',
        }

    # ========== 内部辅助方法 ==========

    def _build_turntable_transform(self):
        """构建转盘坐标系的齐次变换矩阵（绕z轴旋转）"""
        theta = math.radians(self.turntable_angle_deg)
        c, s = math.cos(theta), math.sin(theta)
        return np.array([
            [c, -s, 0, 0],
            [s,  c, 0, 0],
            [0,  0, 1, 0],
            [0,  0, 0, 1]
        ])

    def _build_base_transform(self):
        """构建 base 坐标系的齐次变换矩阵"""
        T = np.eye(4)
        T[0:3, 0:3] = self._euler_zyx_to_mat(*self.base_orientation_deg)
        T[0:3, 3] = self.base_position
        return T

    def _euler_zyx_to_mat(self, rx_d, ry_d, rz_d):
        """ZYX欧拉角(deg) → 旋转矩阵"""
        rx, ry, rz = math.radians(rx_d), math.radians(ry_d), math.radians(rz_d)
        Rz = np.array([[math.cos(rz),-math.sin(rz),0],
                       [math.sin(rz), math.cos(rz),0],[0,0,1]])
        Ry = np.array([[math.cos(ry),0,math.sin(ry)],
                       [0,1,0],[-math.sin(ry),0,math.cos(ry)]])
        Rx = np.array([[1,0,0],[0,math.cos(rx),-math.sin(rx)],
                       [0,math.sin(rx),math.cos(rx)]])
        return Rz @ Ry @ Rx

    def _rot_to_euler_zyx(self, R):
        """旋转矩阵 → ZYX欧拉角(rad)"""
        sy = math.sqrt(R[0,0]**2 + R[1,0]**2)
        if sy > 1e-6:
            rx = math.atan2(R[2,1], R[2,2])
            ry = math.atan2(-R[2,0], sy)
            rz = math.atan2(R[1,0], R[0,0])
        else:
            rx = math.atan2(-R[1,2], R[1,1])
            ry = math.atan2(-R[2,0], sy)
            rz = 0.0
        return rx, ry, rz

    def _pose_to_transform(self, pose):
        """位姿 [x,y,z,rx,ry,rz] → 齐次变换矩阵（角度单位 rad）"""
        T = np.eye(4)
        T[0:3, 0:3] = self._euler_zyx_to_mat(
            math.degrees(pose[3]),
            math.degrees(pose[4]),
            math.degrees(pose[5])
        )
        T[0:3, 3] = pose[0:3]
        return T
