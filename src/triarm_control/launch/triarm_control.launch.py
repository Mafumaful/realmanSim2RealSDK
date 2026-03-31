"""
三臂机器人控制启动文件

启动模式：
- 默认: 仅启动控制节点（无GUI）
- with_gui:=true: 同时启动GUI
- publish_robot_model:=true: 启动 robot_state_publisher + RViz2 模型显示
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share  = get_package_share_directory('triarm_control')
    config_file = os.path.join(pkg_share, 'config', 'triarm_config.yaml')
    urdf_file   = os.path.join(pkg_share, 'urdf', 'triarm.urdf.xacro')

    # ── Launch 参数声明 ──
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
    publish_robot_model_arg = DeclareLaunchArgument(
        'publish_robot_model',
        default_value='false',
        choices=['true', 'false'],
        description='是否启动 robot_state_publisher 在 RViz2 中显示 URDF 模型'
    )

    # ── 控制节点 ──
    controller_node = Node(
        package='triarm_control',
        executable='controller',
        name='triarm_controller',
        parameters=[config_file, {
            'mode':      LaunchConfiguration('mode'),
            'namespace': LaunchConfiguration('namespace'),
        }]
    )

    # ── 统一机械臂节点 ──
    unified_arm_node = Node(
        package='triarm_control',
        executable='unified_arm',
        name='unified_arm_node',
        parameters=[config_file, {
            'mode':      LaunchConfiguration('mode'),
            'namespace': LaunchConfiguration('namespace'),
        }]
    )

    # ── 夹爪桥接节点 ──
    gripper_bridge_node = Node(
        package='triarm_control',
        executable='gripper_bridge',
        name='gripper_bridge_node',
        parameters=[config_file, {
            'mode':      LaunchConfiguration('mode'),
            'namespace': LaunchConfiguration('namespace'),
        }]
    )

    # ── TF 发布节点 ──
    tf_publisher_node = Node(
        package='triarm_control',
        executable='tf_publisher',
        name='tf_publisher_node',
        parameters=[config_file, {
            'mode':                LaunchConfiguration('mode'),
            'namespace':           LaunchConfiguration('namespace'),
            'publish_robot_model': LaunchConfiguration('publish_robot_model'),
        }]
    )

    # ── robot_state_publisher（条件启动，publish_robot_model=true 时启用）──
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration('publish_robot_model'), "' == 'true'"])
        )
    )

    # ── GUI 节点（可选）──
    gui_node = Node(
        package='triarm_control',
        executable='gui',
        name='triarm_gui',
        parameters=[config_file, {
            'namespace': LaunchConfiguration('namespace'),
            'mode':      LaunchConfiguration('mode'),
        }],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('with_gui'), "' == 'true'"])
        )
    )

    return LaunchDescription([
        mode_arg,
        with_gui_arg,
        namespace_arg,
        publish_robot_model_arg,
        controller_node,
        unified_arm_node,
        gripper_bridge_node,
        tf_publisher_node,
        robot_state_publisher_node,
        gui_node,
    ])
