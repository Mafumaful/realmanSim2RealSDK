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
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='sim',
        choices=['sim', 'real'],
        description='运行模式: sim (Isaac Sim) / real (真实机械臂)'
    )

    with_gui_arg = DeclareLaunchArgument(
        'with_gui',
        default_value='false',
        choices=['true', 'false'],
        description='是否启动GUI界面'
    )

    # 控制节点（始终启动，sim模式下负责插值）
    controller_node = Node(
        package='triarm_control',
        executable='controller',
        name='triarm_controller',
        parameters=[config_file, {'mode': LaunchConfiguration('mode')}]
    )

    # 统一机械臂节点（始终启动，桥接 rm_driver 话题）
    unified_arm_node = Node(
        package='triarm_control',
        executable='unified_arm',
        name='unified_arm_node',
        parameters=[config_file, {'mode': LaunchConfiguration('mode')}]
    )

    # 夹爪桥接节点（始终启动）
    gripper_bridge_node = Node(
        package='triarm_control',
        executable='gripper_bridge',
        name='gripper_bridge_node',
        parameters=[config_file, {'mode': LaunchConfiguration('mode')}]
    )

    # GUI节点（可选）
    gui_node = Node(
        package='triarm_control',
        executable='gui',
        name='triarm_gui',
        parameters=[config_file],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('with_gui'), "' == 'true'"])
        )
    )

    return LaunchDescription([
        mode_arg,
        with_gui_arg,
        controller_node,
        unified_arm_node,
        gripper_bridge_node,
        gui_node,
    ])
