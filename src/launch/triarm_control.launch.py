"""
三臂机器人控制启动文件

- 订阅 /joint_states (来自Isaac Sim) 显示实时关节角度
- 发布 /joint_commands 发送目标角度指令
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取配置文件路径
    pkg_share = get_package_share_directory('triarm_control')
    config_file = os.path.join(pkg_share, 'config', 'triarm_config.yaml')

    # GUI节点 (订阅/joint_states, 发布/joint_commands)
    joint_gui = Node(
        package='triarm_control',
        executable='joint_gui',
        name='triarm_joint_control_gui',
        parameters=[config_file]
    )

    return LaunchDescription([
        joint_gui,
    ])
