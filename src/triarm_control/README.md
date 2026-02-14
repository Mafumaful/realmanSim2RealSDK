# triarm_control

三臂机器人关节控制 ROS2 功能包

## 概述

`triarm_control` 是一个用于控制三臂机器人平台的 ROS2 功能包，支持 23 关节体系（19 臂关节 + 4 夹爪关节），提供 sim（Isaac Sim 仿真）和 real（真实机械臂）双模式运行。

## 23 关节体系

### 关节索引与名称

| 索引 | 简称 | JointState 名称 | 说明 |
|------|------|-----------------|------|
| 0 | D1 | `joint_platform_D1` | 底盘旋转 |
| 1 | A1 | `joint_platform_A1` | 左臂关节1 |
| 2 | A2 | `joint_platform_A2` | 左臂关节2 |
| 3 | A3 | `joint_platform_A3` | 左臂关节3 |
| 4 | A4 | `joint_platform_A4` | 左臂关节4 |
| 5 | A5 | `joint_platform_A5` | 左臂关节5 |
| 6 | A6 | `joint_platform_A6` | 左臂关节6 |
| 7 | B1 | `joint_platform_B1` | 右臂关节1 |
| 8 | B2 | `joint_platform_B2` | 右臂关节2 |
| 9 | B3 | `joint_platform_B3` | 右臂关节3 |
| 10 | B4 | `joint_platform_B4` | 右臂关节4 |
| 11 | B5 | `joint_platform_B5` | 右臂关节5 |
| 12 | B6 | `joint_platform_B6` | 右臂关节6 |
| 13 | S1 | `joint_platform_S1` | 头部关节1 |
| 14 | S2 | `joint_platform_S2` | 头部关节2 |
| 15 | S3 | `joint_platform_S3` | 头部关节3 |
| 16 | S4 | `joint_platform_S4` | 头部关节4 |
| 17 | S5 | `joint_platform_S5` | 头部关节5 |
| 18 | S6 | `joint_platform_S6` | 头部关节6 |
| 19 | L1 | `joint_L1` | 左夹爪指1 (0\~1 rad) |
| 20 | L11 | `joint_L11` | 左夹爪指2 (与L1反向) |
| 21 | R1 | `joint_R1` | 右夹爪指1 (0\~1 rad) |
| 22 | R11 | `joint_R11` | 右夹爪指2 (与R1反向) |

### 臂关节索引映射

```
arm_a (左臂):  索引 [1-6]   → joint_platform_A1 ~ A6
arm_b (右臂):  索引 [7-12]  → joint_platform_B1 ~ B6
arm_s (头部):  索引 [13-18] → joint_platform_S1 ~ S6 (预留)
```

### 夹爪关节

- 角度范围：0 rad = 全开，1 rad = 全闭
- 反向联动：`L11 = -L1`，`R11 = -R1`
- 夹爪型号：crt_ctag2f90c

## 系统架构

### 核心设计：共享 23 关节目标数组

`unified_arm_node` 内部维护一个共享的 23 关节目标数组。各臂的 `ArmBridge` 独立进行 IK 解算，只更新自己对应的 6 个关节索引，其余关节保持不变，然后发布完整 23 关节数组到 `/target_joints`。

```
共享目标数组 _shared_target[23]:
  [0]     ← D1 底盘旋转 (rotate_cmd 更新)
  [1-6]   ← ArmBridge(arm_a) 独立更新
  [7-12]  ← ArmBridge(arm_b) 独立更新
  [13-18] ← ArmBridge(arm_s) 预留
  [19-22] ← /gripper_target 更新
```

### sim 模式（Isaac Sim 仿真）

```
                                        ┌──────────────────┐
                                        │  controller_node │  /joint_command   ┌────────────┐
                                        │  (23关节线性插值)  │ ────(23关节)────> │ Isaac Sim  │
                                        └────────▲─────────┘                    └─────┬──────┘
                                                 │ /target_joints (23关节)             │
                                                 │                                     │
┌──────────────────┐  arm_a/movel_cmd   ┌────────┴─────────┐                          │
│ contact_graspnet │ ────────────────> │ unified_arm_node  │                          │
│ _ros2            │  arm_b/movej_cmd   │                   │ <── /joint_states ───────┘
│                  │ ────────────────> │ ArmBridge(arm_a)  │      (23关节反馈)
└──────────────────┘                    │   IK → 6关节      │
                                        │   更新 [1-6]      │
┌──────────────────┐  /gripper_target   │                   │
│    gui_node      │ ────(4关节)─────> │ ArmBridge(arm_b)  │
│  (夹爪快捷按钮)   │                    │   IK → 6关节      │
│                  │  /target_joints    │   更新 [7-12]     │
│  (臂关节滑块)     │ ────(23关节)────> │                   │
└──────────────────┘                    │ 夹爪合并 [19-22]   │
                                        └───────────────────┘

                                        ┌──────────────────┐
                                        │  gripper_bridge   │  sim模式: 发布 gripper_result
                                        │  (结果反馈)        │ <── /gripper_target
                                        └──────────────────┘
```

sim 模式数据流：

1. 单臂运动（以 arm_a 为例）：
   - `arm_a/rm_driver/movel_cmd` → `ArmBridge(arm_a)` SDK IK 解算 → 得到 6 个关节角度
   - 仅更新共享数组 `[1-6]`，其余关节不变 → 发布完整 23 关节 `/target_joints`
   - `controller_node` 线性插值 → `/joint_command` → Isaac Sim

2. 夹爪控制：
   - `GripperController` 或 GUI 发布 `/gripper_target`（4关节弧度）
   - → `unified_arm_node` 更新共享数组 `[19-22]` → `/target_joints` → `controller_node` → Isaac Sim
   - → `gripper_bridge` 检测变化的臂，发布 `/{arm_name}/gripper_result` 反馈给 `GripperController`

3. D1 底盘旋转：
   - `/base_controller/rotate_cmd` → `unified_arm_node` 更新共享数组 `[0]`
   - → `/target_joints` → `controller_node` → `/joint_command` → Isaac Sim

4. 状态反馈：
   - Isaac Sim → `/joint_states`（23关节）→ 各 `ArmBridge` 提取自己的 6 关节更新内部状态

### real 模式（真实机械臂）

```
┌──────────────────┐  arm_a/movel_cmd   ┌──────────────────┐
│ contact_graspnet │ ────────────────> │ unified_arm_node  │
│ _ros2            │  arm_b/movej_cmd   │                   │
│                  │ ────────────────> │ ArmBridge(arm_a)  │ ── SDK TCP ──> RM65 左臂
└──────────────────┘                    │ ArmBridge(arm_b)  │ ── SDK TCP ──> RM65 右臂
                                        └──────────────────┘

┌──────────────────┐  /gripper_target   ┌──────────────────┐
│    gui_node      │ ────(4关节)─────> │  gripper_bridge   │ ── Modbus RTU ──> crt_ctag2f90c
│  (夹爪快捷按钮)   │                    │  (ModbusGripper)  │    左爪 /dev/ttyUSB0
└──────────────────┘                    └──────────────────┘    右爪 /dev/ttyUSB1
```

real 模式数据流：
1. 单臂运动：`arm_a/rm_driver/movel_cmd` → `ArmBridge(arm_a)` → RealMan SDK TCP 直连 → RM65（不经过 controller_node）
2. 夹爪控制：`/gripper_target`（4关节弧度）→ `gripper_bridge` → Modbus RTU 串口 → crt_ctag2f90c 夹爪

### 夹爪 Modbus 映射（crt_ctag2f90c）

基于 `crt_ctag2f90c_gripper_visualization/scripts/GUI2Robot_90.py`：

| 参数 | 值 |
|------|-----|
| 关节角度 → Modbus 位置 | `pos = int((1 - angle) * 9000)` |
| 0 rad（全开） | Modbus 9000 |
| 1 rad（全闭） | Modbus 0 |
| 防重复 | 新位置直接发送；相同位置不重复写入 |
| 寄存器: 使能 | 0x0100 |
| 寄存器: 位置 | 0x0102 (高8位) |
| 寄存器: 速度 | 0x0104 |
| 寄存器: 力矩 | 0x0105 |
| 寄存器: 触发 | 0x0108 |

## 话题接口

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/joint_states` | sensor_msgs/JointState | 订阅 | Isaac Sim 实时状态 (23关节) |
| `/joint_command` | sensor_msgs/JointState | 发布 | 发送给 Isaac Sim 的指令 (23关节) |
| `/target_joints` | std_msgs/Float64MultiArray | 订阅 | 目标位置输入 (23关节, 角度制, 兼容19) |
| `/gripper_target` | std_msgs/Float64MultiArray | 订阅/发布 | 夹爪目标 (4关节: L1,L11,R1,R11, 弧度制) |
| `{arm_name}/gripper_result` | std_msgs/Bool | 发布 | 夹爪动作结果 (仅目标变化的臂发布) |
| `arm_a/rm_driver/movel_cmd` | rm_ros_interfaces/Movel | 订阅 | 左臂直线运动命令 |
| `arm_a/rm_driver/movej_cmd` | rm_ros_interfaces/Movej | 订阅 | 左臂关节运动命令 |
| `arm_a/rm_driver/movej_p_cmd` | rm_ros_interfaces/Movejp | 订阅 | 左臂关节位置运动命令 |
| `arm_a/rm_driver/movel_result` | std_msgs/Bool | 发布 | 左臂运动结果 |
| `arm_b/rm_driver/movel_cmd` | rm_ros_interfaces/Movel | 订阅 | 右臂直线运动命令 |
| `arm_b/rm_driver/movej_cmd` | rm_ros_interfaces/Movej | 订阅 | 右臂关节运动命令 |
| `arm_b/rm_driver/movej_p_cmd` | rm_ros_interfaces/Movejp | 订阅 | 右臂关节位置运动命令 |
| `arm_b/rm_driver/movel_result` | std_msgs/Bool | 发布 | 右臂运动结果 |
| `/base_controller/rotate_cmd` | std_msgs/Float64 | 订阅 | D1 底盘旋转命令 (sim) |
| `/base_controller/rotate_result` | std_msgs/Bool | 发布 | D1 旋转结果 (sim) |

## 安装

```bash
# 安装依赖
sudo apt-get install python3-tk ros-${ROS_DISTRO}-sensor-msgs
pip install minimalmodbus   # real 模式夹爪 Modbus 控制

# 编译
cd ~/Code/realmanSim2RealSDK
colcon build --packages-select triarm_control
source install/setup.bash
```

## 使用方法

### 启动（sim 模式）

```bash
# 控制节点 + 统一臂节点 + 夹爪桥接
ros2 launch triarm_control triarm_control.launch.py mode:=sim

# 带 GUI
ros2 launch triarm_control triarm_control.launch.py mode:=sim with_gui:=true
```

### 启动（real 模式）

```bash
ros2 launch triarm_control triarm_control.launch.py mode:=real
```

### 单独启动各节点

```bash
ros2 run triarm_control controller        # 控制器 (sim 插值)
ros2 run triarm_control unified_arm       # 统一臂节点
ros2 run triarm_control gripper_bridge    # 夹爪桥接
ros2 run triarm_control gui               # GUI
```

### 命令行发送

```bash
# 23关节归零 (角度制)
ros2 topic pub --once /target_joints std_msgs/Float64MultiArray \
  "data: [0,0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0]"

# 平台旋转45度
ros2 topic pub --once /target_joints std_msgs/Float64MultiArray \
  "data: [45, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0]"

# 左夹爪闭合 (弧度制, /gripper_target)
ros2 topic pub --once /gripper_target std_msgs/Float64MultiArray \
  "data: [1.0, -1.0, 0.0, 0.0]"

# 右夹爪闭合
ros2 topic pub --once /gripper_target std_msgs/Float64MultiArray \
  "data: [0.0, 0.0, 1.0, -1.0]"
```

### Python 代码发送

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

rclpy.init()
node = Node('target_sender')
pub = node.create_publisher(Float64MultiArray, '/target_joints', 10)
gripper_pub = node.create_publisher(Float64MultiArray, '/gripper_target', 10)
time.sleep(0.5)

# 臂关节目标 (23关节, 角度制)
msg = Float64MultiArray()
msg.data = [
    0.0,                              # D1 平台
    0.0, 30.0, 0.0, 0.0, 0.0, 0.0,   # A1-A6 (左臂)
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    # B1-B6 (右臂)
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    # S1-S6 (头部)
    0.0, 0.0, 0.0, 0.0,              # L1, L11, R1, R11 (夹爪)
]
pub.publish(msg)

# 夹爪目标 (4关节, 弧度制: [L1, L11, R1, R11])
gripper_msg = Float64MultiArray()
gripper_msg.data = [1.0, -1.0, 0.0, 0.0]  # 左夹爪闭合
gripper_pub.publish(gripper_msg)

node.destroy_node()
rclpy.shutdown()
```

## 文件结构

```
triarm_control/
├── config/
│   └── triarm_config.yaml          # 配置参数 (含 Modbus 串口配置)
├── launch/
│   └── triarm_control.launch.py    # 启动文件 (4个节点)
├── triarm_control/
│   ├── joint_names.py              # 23关节定义 + 限位 + 夹爪索引
│   ├── arm_controller.py           # sim-only 线性插值控制器
│   ├── controller_node.py          # ROS2 控制节点 (插值 → /joint_command)
│   ├── unified_arm_node.py         # 统一臂节点 (rm_driver 兼容 + 单臂IK + 夹爪合并)
│   ├── gripper_bridge.py           # 夹爪桥接 (sim: 监控 / real: Modbus RTU)
│   ├── gui_node.py                 # GUI 节点 (含夹爪标签页)
│   └── realman_sdk_wrapper.py      # RealMan SDK 封装 (IK/FK + real TCP)
├── package.xml
└── setup.py
```

## 配置参数

### triarm_controller

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `namespace` | `""` | 话题命名空间 |
| `mode` | `"sim"` | 运行模式: sim / real |
| `joint_velocity` | `30.0` | 关节速度 (度/秒) |
| `publish_rate` | `50.0` | 发布频率 (Hz) |

### unified_arm_node

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `mode` | `"sim"` | 运行模式 |
| `publish_rate` | `20.0` | 状态发布频率 (Hz) |
| `sim_joint_tolerance` | `0.02` | sim 运动到位阈值 (rad) |
| `sim_motion_timeout` | `10.0` | sim 运动超时 (秒) |
| `arm_a.ip` / `arm_b.ip` | `192.168.1.18/19` | 臂 IP 地址 (real) |
| `arm_a.port` / `arm_b.port` | `8080` | 臂端口 (real) |

### gripper_bridge_node

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `mode` | `"sim"` | 运行模式 |
| `gripper_a.serial_port` | `/dev/ttyUSB0` | 左夹爪串口 (real) |
| `gripper_a.baud_rate` | `115200` | 波特率 |
| `gripper_a.slave_addr` | `1` | Modbus 从站地址 |
| `gripper_b.serial_port` | `/dev/ttyUSB1` | 右夹爪串口 (real) |
| `gripper_b.baud_rate` | `115200` | 波特率 |
| `gripper_b.slave_addr` | `1` | Modbus 从站地址 |

### triarm_gui

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `namespace` | `""` | 话题命名空间 |
| `gui_width` | `700` | GUI 宽度 |
| `gui_height` | `600` | GUI 高度 |

## 依赖

- `rclpy`, `sensor_msgs`, `std_msgs`, `geometry_msgs` — ROS2 基础
- `rm_ros_interfaces` — RealMan 臂运动消息 (Movel/Movej/Movejp)
- `Robotic_Arm` — RealMan SDK (IK/FK + real TCP 控制)
- `transforms3d` — 四元数/欧拉角转换
- `minimalmodbus` — real 模式夹爪 Modbus RTU 控制
- `tkinter` — GUI (可选)
