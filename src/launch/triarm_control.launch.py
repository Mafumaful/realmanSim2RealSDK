"""
三臂机器人控制启动文件

- 订阅 /joint_states (来自Isaac Sim) 显示实时关节角度
- 发布 /joint_commands 发送目标角度指令
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # GUI节点 (订阅/joint_states, 发布/joint_commands)
    joint_gui = Node(
        package='triarm_control',
        executable='joint_gui',
        name='triarm_joint_control_gui'
    )

    return LaunchDescription([
        joint_gui,
    ])
