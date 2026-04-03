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
        base_orientation_deg : [roll_x, pitch_y, yaw_z] 固定轴 XYZ 外旋欧拉角，单位 deg
                               描述机械臂base相对转盘的姿态，默认 [0, 0, 0]
                               旋转矩阵 = Rz(yaw) @ Ry(pitch) @ Rx(roll)
        d6_mm                : 末端 d6 参数，单位 mm，默认 172.5 (RM65-6F)

        注：base_position 和 base_orientation_deg 是机械臂base相对转盘的固定安装位姿
        """
        self.base_position = base_position or [0.0, 0.0, 0.0]
        self.base_orientation_deg = base_orientation_deg or [0.0, 0.0, 0.0]
        self.d6_mm = d6_mm
        self.turntable_angle_deg = 0.0  # 转盘旋转角度（绕z轴）

        # 构建 DH 参数（同时保存为实例变量，供 joint_transforms 复用）
        mm = 1e-3
        self._dh_a     = [0, 0, 256*mm, 0, 0, 0]
        self._dh_alpha = [0, 90, 0, 90, -90, 90]
        self._dh_d     = [240.5*mm, 0, 0, 210*mm, 0, d6_mm*mm]
        self._dh_offset= [0, 90, 90, 0, 0, 0]

        dh = rm_dh_t(
            a     = self._dh_a,
            alpha = self._dh_alpha,
            d     = self._dh_d,
            offset= self._dh_offset,
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
        roll_x, pitch_y, yaw_z = self._rot_to_euler_xyz(T_world[0:3, 0:3])

        return {
            'position': [x, y, z],
            'euler_deg': [math.degrees(roll_x), math.degrees(pitch_y), math.degrees(yaw_z)],
            'euler_rad': [roll_x, pitch_y, yaw_z],
        }

    def joint_transforms(self, joint_angles_deg):
        """计算每个关节的相对变换（用于 TF 树发布）

        参数
        ----
        joint_angles_deg : list[float], 6个关节角度，单位 deg

        返回
        ----
        list of (xyz, quat_wxyz): 6个变换，每个表示 link_i → link_{i+1}
            xyz: [x, y, z] 单位 m
            quat_wxyz: [w, x, y, z] 四元数
        """
        transforms = []
        for i in range(6):
            theta = math.radians(joint_angles_deg[i] + self._dh_offset[i])
            c, s = math.cos(theta), math.sin(theta)
            ca, sa = math.cos(math.radians(self._dh_alpha[i])), math.sin(math.radians(self._dh_alpha[i]))

            # DH 变换矩阵
            T = np.array([
                [c, -s*ca,  s*sa, self._dh_a[i]*c],
                [s,  c*ca, -c*sa, self._dh_a[i]*s],
                [0,  sa,    ca,   self._dh_d[i]  ],
                [0,  0,     0,    1               ]
            ])

            xyz = T[0:3, 3]
            R = T[0:3, 0:3]

            # 旋转矩阵转四元数
            trace = R[0,0] + R[1,1] + R[2,2]
            if trace > 0:
                s = 0.5 / math.sqrt(trace + 1.0)
                w = 0.25 / s
                x = (R[2,1] - R[1,2]) * s
                y = (R[0,2] - R[2,0]) * s
                z = (R[1,0] - R[0,1]) * s
            else:
                if R[0,0] > R[1,1] and R[0,0] > R[2,2]:
                    s = 2.0 * math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
                    w = (R[2,1] - R[1,2]) / s
                    x = 0.25 * s
                    y = (R[0,1] + R[1,0]) / s
                    z = (R[0,2] + R[2,0]) / s
                elif R[1,1] > R[2,2]:
                    s = 2.0 * math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
                    w = (R[0,2] - R[2,0]) / s
                    x = (R[0,1] + R[1,0]) / s
                    y = 0.25 * s
                    z = (R[1,2] + R[2,1]) / s
                else:
                    s = 2.0 * math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
                    w = (R[1,0] - R[0,1]) / s
                    x = (R[0,2] + R[2,0]) / s
                    y = (R[1,2] + R[2,1]) / s
                    z = 0.25 * s

            transforms.append((xyz, np.array([w, x, y, z])))

        return transforms

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
        roll_x, pitch_y, yaw_z = self._rot_to_euler_xyz(T_target_base[0:3, 0:3])
        target_pose_base = [x, y, z, roll_x, pitch_y, yaw_z]

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
        """构建 base 坐标系的齐次变换矩阵（相对转盘）

        base_orientation_deg 为固定轴 XYZ 外旋欧拉角 [roll_x, pitch_y, yaw_z]，
        与 config 中的定义完全对应，R = Rz @ Ry @ Rx。
        """
        T = np.eye(4)
        roll_x, pitch_y, yaw_z = self.base_orientation_deg
        T[0:3, 0:3] = self._euler_zyx_to_mat(roll_x, pitch_y, yaw_z)
        T[0:3, 3] = self.base_position
        return T

    def _euler_zyx_to_mat(self, roll_x_deg, pitch_y_deg, yaw_z_deg):
        """固定轴 XYZ 外旋欧拉角(deg) → 旋转矩阵

        参数顺序：[roll_x, pitch_y, yaw_z]（与 config base_orientation_deg 完全对应）
        旋转矩阵：R = Rz(yaw_z) @ Ry(pitch_y) @ Rx(roll_x)
        即先绕固定 X 轴转 roll，再绕固定 Y 轴转 pitch，最后绕固定 Z 轴转 yaw。
        """
        rx = math.radians(roll_x_deg)
        ry = math.radians(pitch_y_deg)
        rz = math.radians(yaw_z_deg)
        Rz = np.array([[math.cos(rz), -math.sin(rz), 0],
                       [math.sin(rz),  math.cos(rz), 0],
                       [0,             0,             1]])
        Ry = np.array([[ math.cos(ry), 0, math.sin(ry)],
                       [ 0,            1, 0            ],
                       [-math.sin(ry), 0, math.cos(ry)]])
        Rx = np.array([[1, 0,            0           ],
                       [0, math.cos(rx), -math.sin(rx)],
                       [0, math.sin(rx),  math.cos(rx)]])
        return Rz @ Ry @ Rx

    def _rot_to_euler_xyz(self, R):
        """旋转矩阵 → 固定轴 XYZ 外旋欧拉角 (rad)

        与 _euler_zyx_to_mat / _pose_to_transform 约定完全一致：
        R = Rz(yaw) @ Ry(pitch) @ Rx(roll)，提取 (roll_x, pitch_y, yaw_z)。
        """
        sy = math.sqrt(R[0, 0] ** 2 + R[0, 1] ** 2)
        if sy > 1e-6:
            roll_x  = math.atan2( R[1, 2], R[2, 2])
            pitch_y = math.atan2(-R[0, 2], sy)
            yaw_z   = math.atan2( R[0, 1], R[0, 0])
        else:
            roll_x  = math.atan2(-R[2, 1], R[1, 1])
            pitch_y = math.atan2(-R[0, 2], sy)
            yaw_z   = 0.0
        return roll_x, pitch_y, yaw_z

    def _pose_to_transform(self, pose):
        """位姿 [x,y,z,rx,ry,rz] → 齐次变换矩阵

        pose 姿态单位为 rad，按固定轴 XYZ 外旋顺序 [roll_x, pitch_y, yaw_z] 解释，
        与 _euler_zyx_to_mat 约定完全一致。
        """
        T = np.eye(4)
        T[0:3, 0:3] = self._euler_zyx_to_mat(
            math.degrees(pose[3]),   # roll_x
            math.degrees(pose[4]),   # pitch_y
            math.degrees(pose[5]),   # yaw_z
        )
        T[0:3, 3] = pose[0:3]
        return T
