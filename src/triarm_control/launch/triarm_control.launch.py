"""
三臂机器人控制启动文件

启动模式：
- 默认: 仅启动控制节点（无GUI）
- with_gui:=true: 同时启动GUI
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('triarm_control')
    config_file = os.path.join(pkg_share, 'config', 'triarm_config.yaml')

    # 声明参数
    with_gui_arg = DeclareLaunchArgument(
        'with_gui', default_value='false',
        description='是否启动GUI界面'
    )

    # 控制节点（始终启动）
    controller_node = Node(
        package='triarm_control',
        executable='controller',
        name='triarm_controller',
        parameters=[config_file]
    )

    # GUI节点（可选）
    gui_node = Node(
        package='triarm_control',
        executable='gui',
        name='triarm_gui',
        parameters=[config_file],
        condition=IfCondition(
            PythonExpression([LaunchConfiguration('with_gui'), " == 'true'"])
        )
    )

    return LaunchDescription([
        with_gui_arg,
        controller_node,
        gui_node,
    ])
