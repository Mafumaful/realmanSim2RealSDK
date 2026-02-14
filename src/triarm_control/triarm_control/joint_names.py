"""
关节名称和限位配置

23 关节体系:
  [0]     D1          (底盘旋转)
  [1-6]   A1-A6       (左臂)
  [7-12]  B1-B6       (右臂)
  [13-18] S1-S6       (头部)
  [19]    joint_L1    (左夹爪指1)
  [20]    joint_L11   (左夹爪指2, 与L1反向)
  [21]    joint_R1    (右夹爪指1)
  [22]    joint_R11   (右夹爪指2, 与R1反向)
"""

import math


def deg2rad(deg):
    return deg * math.pi / 180.0


# 23个关节名称列表 (按JointState消息顺序)
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
    # 夹爪 (crt_ctag2f90c)
    'joint_L1',     # 左夹爪指1
    'joint_L11',    # 左夹爪指2 (与L1反向联动)
    'joint_R1',     # 右夹爪指1
    'joint_R11',    # 右夹爪指2 (与R1反向联动)
]

# 臂关节数 (不含夹爪)
ARM_JOINT_COUNT = 19
# 总关节数 (含夹爪)
TOTAL_JOINT_COUNT = 23

# 夹爪关节在23关节数组中的索引
GRIPPER_JOINT_INDICES = {
    'arm_a': [19, 20],   # joint_L1, joint_L11
    'arm_b': [21, 22],   # joint_R1, joint_R11
}

# 关节限位 (min, max) 单位：弧度
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
    'joint_platform_B1': (deg2rad(-180), deg2rad(180)),
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
    # 夹爪: 0~1 rad (0=全开, 1=全闭)
    'joint_L1':  (0.0, 1.0),
    'joint_L11': (-1.0, 0.0),   # 与L1反向
    'joint_R1':  (0.0, 1.0),
    'joint_R11': (-1.0, 0.0),   # 与R1反向
}
