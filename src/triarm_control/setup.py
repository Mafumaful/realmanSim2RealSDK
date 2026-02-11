from setuptools import setup
import os
from glob import glob

package_name = 'triarm_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='三臂机器人关节控制ROS2功能包',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = triarm_control.controller_node:main',
            'gui = triarm_control.gui_node:main',
            'unified_arm = triarm_control.unified_arm_node:main',
            'gripper_bridge = triarm_control.gripper_bridge:main',
        ],
    },
)
