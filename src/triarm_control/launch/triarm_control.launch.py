"""
三臂机器人控制启动文件

启动模式：
- 默认: 仅启动控制节点（无GUI）
- with_gui:=true: 同时启动GUI
- 默认同时启动 `robot_state_publisher`，为抓取感知提供 `platform_link`
  与各机械臂链路 TF。
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

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot',
        description='三臂系统命名空间，用于 joint_states/joint_command 等话题'
    )

    # 控制节点（始终启动，sim模式下负责插值）
    controller_node = Node(
        package='triarm_control',
        executable='controller',
        name='triarm_controller',
        parameters=[config_file, {
            'mode': LaunchConfiguration('mode'),
            'namespace': LaunchConfiguration('namespace'),
        }]
    )

    # 统一机械臂节点（始终启动，桥接 rm_driver 话题）
    unified_arm_node = Node(
        package='triarm_control',
        executable='unified_arm',
        name='unified_arm_node',
        parameters=[config_file, {
            'mode': LaunchConfiguration('mode'),
            'namespace': LaunchConfiguration('namespace'),
        }]
    )

    # 夹爪桥接节点（始终启动）
    gripper_bridge_node = Node(
        package='triarm_control',
        executable='gripper_bridge',
        name='gripper_bridge_node',
        parameters=[config_file, {
            'mode': LaunchConfiguration('mode'),
            'namespace': LaunchConfiguration('namespace'),
        }]
    )

    # TF 发布节点（统一管理所有坐标变换，替代 unified_arm_node 中分散的 TF 发布）
    tf_publisher_node = Node(
        package='triarm_control',
        executable='tf_publisher',
        name='tf_publisher_node',
        parameters=[config_file, {
            'mode': LaunchConfiguration('mode'),
            'namespace': LaunchConfiguration('namespace'),
        }]
    )

    # GUI节点（可选）
    gui_node = Node(
        package='triarm_control',
        executable='gui',
        name='triarm_gui',
        parameters=[config_file, {
            'namespace': LaunchConfiguration('namespace'),
            'mode': LaunchConfiguration('mode'),
        }],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('with_gui'), "' == 'true'"])
        )
    )

    return LaunchDescription([
        mode_arg,
        with_gui_arg,
        namespace_arg,
        controller_node,
        unified_arm_node,
        gripper_bridge_node,
        tf_publisher_node,
        gui_node,
    ])
