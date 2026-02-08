"""
realmanSim2RealSDK - 三臂机器人关节控制模块

支持仿真环境和真实机械臂的关节角度控制。
"""

from .joint_controller import JointController
from .config import JOINT_LIMITS

__all__ = ['JointController', 'JOINT_LIMITS']
__version__ = '0.1.0'
