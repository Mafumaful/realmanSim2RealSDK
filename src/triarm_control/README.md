# triarm_control

三臂机器人关节控制ROS2功能包

## 概述

`triarm_control` 是一个用于控制三臂机器人平台的ROS2功能包。支持与Isaac Sim仿真环境交互，提供：
- 核心控制器（与ROS解耦）
- 可选的GUI界面
- 矩阵形式的目标位置输入接口

## 系统架构

```
┌─────────────────┐     /target_joints      ┌──────────────────┐
│   外部程序       │ ──────────────────────> │                  │
│ (Python/C++等)  │   Float64MultiArray     │  controller_node │
└─────────────────┘                         │                  │
                                            │  (ArmController) │
┌─────────────────┐     /target_joints      │                  │
│    gui_node     │ ──────────────────────> │                  │
│   (可选GUI)     │                         └────────┬─────────┘
└─────────────────┘                                  │
        ▲                                            │ /joint_command
        │ /joint_states                              ▼
        │ /joint_command                      ┌──────────────────┐
        └──────────────────────────────────── │   Isaac Sim      │
                                              └──────────────────┘
```

## 话题接口

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/joint_states` | sensor_msgs/JointState | 订阅 | 来自Isaac Sim的实时状态 |
| `/joint_command` | sensor_msgs/JointState | 发布 | 发送给Isaac Sim的指令 |
| `/target_joints` | std_msgs/Float64MultiArray | 订阅 | **矩阵输入接口** |

## 安装

```bash
# 安装依赖
sudo apt-get install python3-tk ros-${ROS_DISTRO}-sensor-msgs

# 编译
cd ~/Code/realmanSim2RealSDK
colcon build --packages-select triarm_control
source install/setup.bash
```

## 使用方法

### 1. 启动控制节点（无GUI）

```bash
ros2 launch triarm_control triarm_control.launch.py
```

### 2. 启动控制节点 + GUI

```bash
ros2 launch triarm_control triarm_control.launch.py with_gui:=true
```

### 3. 单独启动各节点

```bash
# 仅控制器
ros2 run triarm_control controller

# 仅GUI
ros2 run triarm_control gui
```

## 矩阵输入接口

通过 `/target_joints` 话题发送19个关节的目标角度（单位：度）。

### 关节顺序

```
索引 0:     D1  (平台旋转)
索引 1-6:   A1-A6 (机械臂A)
索引 7-12:  B1-B6 (机械臂B)
索引 13-18: S1-S6 (机械臂S)
```

### 命令行发送

```bash
# 所有关节归零
ros2 topic pub --once /target_joints std_msgs/Float64MultiArray \
  "data: [0,0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0]"

# 平台旋转45度，其他不动
ros2 topic pub --once /target_joints std_msgs/Float64MultiArray \
  "data: [45, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0]"

# 机械臂A的A2关节转30度
ros2 topic pub --once /target_joints std_msgs/Float64MultiArray \
  "data: [0, 0,30,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0]"
```

### Python代码发送

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

rclpy.init()
node = Node('target_sender')
pub = node.create_publisher(Float64MultiArray, '/target_joints', 10)

# 等待连接
import time
time.sleep(0.5)

# 构建目标位置 (19个关节，单位：度)
msg = Float64MultiArray()
msg.data = [
    0.0,                              # D1 平台
    0.0, 30.0, 0.0, 0.0, 0.0, 0.0,    # A1-A6
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,     # B1-B6
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,     # S1-S6
]

pub.publish(msg)
print("目标位置已发送")

node.destroy_node()
rclpy.shutdown()
```

## 文件结构

```
triarm_control/
├── config/
│   └── triarm_config.yaml      # 配置参数
├── launch/
│   └── triarm_control.launch.py
├── triarm_control/
│   ├── arm_controller.py       # 核心控制器（纯逻辑）
│   ├── controller_node.py      # ROS2控制节点
│   ├── gui_node.py             # GUI节点（可选）
│   └── joint_names.py          # 关节配置
├── package.xml
└── setup.py
```

## 配置参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `namespace` | `""` | 话题命名空间 |
| `joint_velocity` | `30.0` | 关节速度（度/秒） |
| `publish_rate` | `50.0` | 发布频率（Hz） |
| `gui_width` | `700` | GUI宽度 |
| `gui_height` | `500` | GUI高度 |
