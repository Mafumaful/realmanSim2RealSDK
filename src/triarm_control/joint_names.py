"""
关节名称和限位配置
"""

# 19个关节名称列表 (按JointState消息顺序)
JOINT_NAMES_LIST = [
    # 平台旋转
    'joint_platform_D1',
    # 机械臂A
    'joint_platform_A1',
    'joint_platform_A2',
    'joint_platform_A3',
    'joint_platform_A4',
    'joint_platform_A5',
    'joint_platform_A6',
    # 机械臂B
    'joint_platform_B1',
    'joint_platform_B2',
    'joint_platform_B3',
    'joint_platform_B4',
    'joint_platform_B5',
    'joint_platform_B6',
    # 机械臂S
    'joint_platform_S1',
    'joint_platform_S2',
    'joint_platform_S3',
    'joint_platform_S4',
    'joint_platform_S5',
    'joint_platform_S6',
]

# 关节限位 (min, max) 单位：弧度
import math

def deg2rad(deg):
    return deg * math.pi / 180.0

JOINT_LIMITS = {
    'joint_platform_D1': (deg2rad(-180), deg2rad(180)),
    # 机械臂A
    'joint_platform_A1': (deg2rad(-180), deg2rad(180)),
    'joint_platform_A2': (deg2rad(-129.5), deg2rad(129.5)),
    'joint_platform_A3': (deg2rad(-129.5), deg2rad(129.5)),
    'joint_platform_A4': (deg2rad(-180), deg2rad(180)),
    'joint_platform_A5': (deg2rad(-126), deg2rad(126)),
    'joint_platform_A6': (deg2rad(-355), deg2rad(355)),
    # 机械臂B
    'joint_platform_B1': (deg2rad(-177.6), deg2rad(177.6)),
    'joint_platform_B2': (deg2rad(-129.5), deg2rad(129.5)),
    'joint_platform_B3': (deg2rad(-129.5), deg2rad(129.5)),
    'joint_platform_B4': (deg2rad(-180), deg2rad(180)),
    'joint_platform_B5': (deg2rad(-126), deg2rad(126)),
    'joint_platform_B6': (deg2rad(-355), deg2rad(355)),
    # 机械臂S
    'joint_platform_S1': (deg2rad(-153.6), deg2rad(153.6)),
    'joint_platform_S2': (deg2rad(0), deg2rad(189)),
    'joint_platform_S3': (deg2rad(0), deg2rad(180)),
    'joint_platform_S4': (deg2rad(-88.8), deg2rad(88.8)),
    'joint_platform_S5': (deg2rad(-88.8), deg2rad(88.8)),
    'joint_platform_S6': (deg2rad(-171.9), deg2rad(171.9)),
}
