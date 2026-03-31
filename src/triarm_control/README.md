# triarm_control

三臂机器人关节控制 ROS2 功能包

## 概述

`triarm_control` 是一个用于控制三臂机器人平台的 ROS2 功能包，支持 23 关节体系（19 臂关节 + 4 夹爪关节），提供 sim（Isaac Sim 仿真）和 real（真实机械臂）双模式运行。

核心特性：
- 双模式透明切换：调用方通过相同的 rm_driver 话题接口控制，内部自动路由 sim/real
- 自定义 base 坐标系：每条臂可独立配置相对转盘的安装位姿，IK/FK 自动处理世界坐标变换
- S 型曲线平滑插值：sim 模式支持线性和 S 型加减速两种插值策略
- 夹爪双通道：sim 模式通过关节数组控制，real 模式通过 Modbus RTU 控制

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
| 19 | L1 | `joint_L1` | 左夹爪指1 |
| 20 | L11 | `joint_L11` | 左夹爪指2 (与L1反向) |
| 21 | R1 | `joint_R1` | 右夹爪指1 |
| 22 | R11 | `joint_R11` | 右夹爪指2 (与R1反向) |

### 臂关节索引映射

```
arm_a (左臂):  索引 [1-6]   → joint_platform_A1 ~ A6
arm_b (右臂):  索引 [7-12]  → joint_platform_B1 ~ B6
arm_s (头部):  索引 [13-18] → joint_platform_S1 ~ S6 (预留)
```

### 夹爪关节

- sim 模式角度范围：0 deg = 全开，-30 deg = 全闭 (可通过配置参数 `left_gripper`/`right_gripper` 分别设置 sim/real 角度)
- 反向联动：`L11 = -L1`，`R11 = -R1`
- 夹爪型号：crt_ctag2f90c

## 系统架构

### 核心设计

#### 1. 共享 23 关节目标数组

`unified_arm_node` 内部维护一个共享的 23 关节目标数组 `_shared_target[23]`。各臂的 `ArmBridge` 独立进行 IK 解算，只更新自己对应的 6 个关节索引，其余关节保持不变，然后发布完整 23 关节数组到 `/target_joints`。同时订阅 `/joint_command` 持续同步 `_shared_target`，确保与 `controller_node` 内部目标一致。

```
共享目标数组 _shared_target[23]:
  [0]     ← D1 底盘旋转 (rotate_cmd 更新)
  [1-6]   ← ArmBridge(arm_a) 独立更新
  [7-12]  ← ArmBridge(arm_b) 独立更新
  [13-18] ← ArmBridge(arm_s) 预留
  [19-22] ← /gripper_target 更新
  持续同步 ← /joint_command (controller_node 插值输出)
```

#### 2. 自定义 base 坐标系与运动学

每条臂通过 `RM65Robot` 算法库（`rm65_robot.py`）独立计算正/逆运动学，支持自定义 base 坐标系参数：

- `base_position`: 机械臂 base 相对转盘的位置 `[x, y, z]` (m)
- `base_orientation_deg`: 机械臂 base 相对转盘的姿态 `[rx, ry, rz]` ZYX 欧拉角 (deg)
- `d6_mm`: 末端 d6 参数 (mm)

坐标变换链：`世界坐标系 = 转盘旋转(D1) @ base安装位姿 @ 末端相对base`

`RealManAlgo` 封装了 `RM65Robot` 实例，`RealManSDKWrapper` 在此基础上提供模式感知的统一接口：
- sim 模式：仅使用 `RealManAlgo` 进行 IK/FK 纯本地计算
- real 模式：`RealManAlgo` IK/FK + `RoboticArm` TCP 直连真实机械臂

#### 3. S 型曲线插值

`controller_node` (sim-only) 支持两种插值策略：
- **线性插值**：恒定速度逼近目标
- **S 型曲线插值** (默认)：平滑加减速，自动判断加速/匀速/减速阶段

通过参数 `enable_smooth` 和话题 `/robot/enable_smooth` 动态切换，`velocity_mode` 支持 slow(0.5x)/normal(1.0x)/fast(2.0x) 速度倍率。

### sim 模式（Isaac Sim 仿真）

```
                                        ┌──────────────────────┐
                                        │   controller_node    │  /robot/joint_command  ┌────────────┐
                                        │ (23关节 S型曲线插值)  │ ────(23关节)─────────> │ Isaac Sim  │
                                        └────────▲──────┬──────┘                         └─────┬──────┘
                                                 │      │ /robot/joint_command                  │
                                                 │      │ (同步 _shared_target)                 │
                                                 │      ▼                                       │
                                  /target_joints │ ┌────────────────────┐                       │
                                   (23关节)      │ │  unified_arm_node  │                       │
                                                 │ │                    │ <── /robot/joint_states┘
┌──────────────────┐  arm_a/movel_cmd   ┌────────┴─┤  ArmBridge(arm_a) │      (23关节反馈)
│  上层应用节点     │ ────────────────> │           │   RM65Robot IK/FK │
│ (状态机+夹爪控制) │  arm_b/movej_cmd   │           │   世界坐标→关节角  │
│                  │ ────────────────> │           │   更新 [1-6]       │
│                  │                    │           │                    │
│                  │ ─┐ gripper_target  │           │  ArmBridge(arm_b) │
└──────────────────┘  │                 │           │   RM65Robot IK/FK │
                      ├──────────────> │           │   更新 [7-12]      │
┌──────────────────┐  │                 │           │                    │
│    gui_node      │ ─┘                 │           │ 夹爪合并 [19-22]   │
│ (夹爪/臂/Pose)   │                    │           │ D1旋转 [0]         │
│                  │ /robot/target_joints│           │                    │
│                  │ ────(23关节)─────> │           └────────────────────┘
└──────────────────┘                    │
                                        │           ┌────────────────────┐
                                        │           │   gripper_bridge   │
                                        └─────────> │ sim: 发布 result   │
                                     gripper_target └────────────────────┘
```

sim 模式数据流：

1. 单臂运动（以 arm_a 为例）：
   - `arm_a/rm_driver/movel_cmd` → `ArmBridge(arm_a)` 世界坐标 → `RM65Robot` IK（自动处理转盘角度 D1 和 base 安装位姿）→ 得到 6 个关节角度
   - 仅更新共享数组 `[1-6]`，其余关节不变 → 发布完整 23 关节 `/robot/target_joints`
   - `controller_node` S 型曲线插值 → `/robot/joint_command` → Isaac Sim
   - `/robot/joint_command` 同时反馈到 `unified_arm_node` 同步 `_shared_target`

2. 基座坐标系直接控制：
   - `{arm}/base_pose_cmd` (Float64MultiArray[6]) → `ArmBridge` 直接调用 `RealManSDKWrapper.inverse_kinematics`（跳过世界坐标变换）→ 关节角度 → `/target_joints`

3. 夹爪控制：
   - `GripperController` 或 GUI 发布 `/robot/gripper_target`（4关节弧度）
   - → `unified_arm_node` 更新共享数组 `[19-22]` → `/robot/target_joints` → `controller_node` → Isaac Sim
   - → `gripper_bridge` 检测变化的臂，发布 `/robot/{arm}/gripper_result` 反馈给 `GripperController`

4. D1 底盘旋转：
   - `/base_controller/rotate_cmd` → `unified_arm_node` 更新共享数组 `[0]`
   - → `/robot/target_joints` → `controller_node` → `/robot/joint_command` → Isaac Sim

5. 状态反馈：
   - Isaac Sim → `/robot/joint_states`（23关节）→ 各 `ArmBridge` 提取自己的 6 关节更新内部状态
   - 各 `ArmBridge` 通过 `RM65Robot` FK 计算末端位姿 → 发布 `{arm}/rm_driver/udp_arm_position`

### real 模式（真实机械臂）

```
┌──────────────────┐  arm_a/movel_cmd   ┌────────────────────┐
│  上层应用节点     │ ────────────────> │  unified_arm_node   │
│ (状态机+夹爪控制) │  arm_b/movej_cmd   │                     │
│                  │ ────────────────> │  ArmBridge(arm_a)   │ ── SDK TCP ──> RM65 左臂
│                  │                    │   RealManSDKWrapper │
│                  │                    │  ArmBridge(arm_b)   │ ── SDK TCP ──> RM65 右臂
│                  │ /robot/gripper_target│  RealManSDKWrapper │
│                  │ ────(4关节)──┐     └────────────────────┘
└───────▲──────────┘              │
        │                         ▼
        │ gripper_holding  ┌──────────────────┐
        │ gripper_result   │  gripper_bridge   │ ── Modbus RTU ──> crt_ctag2f90c
        │                  │  (ModbusGripper)  │    左爪 /dev/ttyUSB0
        └──────────────────│  per-arm Lock     │    右爪 /dev/ttyUSB1
                           │  线程化执行        │
┌──────────────────┐       └────────┬──────────┘
│    gui_node      │ ──────────────>│ gripper_target
│ (夹爪/臂/Pose)   │               │
└──────────────────┘               ▼
                          /robot/{arm}/gripper_result  → 动作完成反馈
                          /robot/{arm}/gripper_holding → 持有状态 (2Hz Modbus 轮询)
```

real 模式数据流：
1. 单臂运动：`arm_a/rm_driver/movel_cmd` → `ArmBridge(arm_a)` → `RealManSDKWrapper` SDK TCP 直连 → RM65（不经过 controller_node）
2. 夹爪控制：`GripperController` 或 GUI 发布 `/robot/gripper_target`（4关节弧度）→ `gripper_bridge` 线程化 Modbus RTU → crt_ctag2f90c 夹爪 → `/robot/{arm}/gripper_result` 反馈
3. 持有检测：`gripper_bridge` 2Hz 周期轮询 Modbus 力矩到达(0x0601) + 掉落报警(0x0612) → `/robot/{arm}/gripper_holding` → `GripperController.check_holding()` 三级判断
4. 状态反馈：`RealManSDKWrapper` 通过 TCP 查询关节角度和末端位姿 → 发布 `{arm}/joint_states` 和 `{arm}/rm_driver/udp_arm_position`

### 核心模块说明

#### `rm65_robot.py` — RM65 运动学算法库

基于 RealMan 官方 `Robotic_Arm` SDK 的 `Algo` 接口，封装了支持自定义 base 坐标系的正/逆运动学计算：

- **正运动学 (FK)**：关节角度 → 世界坐标系末端位姿。内部构建变换链 `T_turntable @ T_base @ T_end`
- **逆运动学 (IK)**：世界坐标系目标位姿 → 关节角度。先将目标变换到 base 坐标系，再调用 SDK 逆解（遍历模式）
- 支持动态设置转盘角度 `set_turntable_angle()`，每次 IK 调用自动考虑 D1 旋转

#### `realman_sdk_wrapper.py` — SDK 模式感知封装

两层封装：

| 类 | 职责 |
|----|------|
| `RealManAlgo` | 封装 `RM65Robot` 实例，提供 IK/FK + SDK Algo 辅助功能 (pos2matrix/matrix2pos/pose_move)，纯本地计算 |
| `RealManSDKWrapper` | 每臂一个实例，模式感知统一接口。sim 模式仅用 `RealManAlgo`；real 模式额外建立 TCP 连接，支持 movej/movel/movej_p/movep/stop + 夹爪 SDK 控制 + 状态查询 |

#### `unified_arm_node.py` — 统一臂节点

核心节点，为 A/B 臂各创建一个 `ArmBridge`，提供 rm_driver 兼容话题接口：

- 每个 `ArmBridge` 持有独立的 `RealManSDKWrapper` 实例，使用各自的 base 参数初始化 `RM65Robot`
- sim 模式：Pose 命令 → 世界坐标 IK（自动获取 D1 角度并传入 RM65Robot）→ 关节角度 → 更新共享目标 → 发布 `/target_joints` → 轮询等待到位
- real 模式：Pose/Joint 命令 → `RealManSDKWrapper` TCP 直发 → 阻塞等待结果
- 支持运动命令串行化锁 `_motion_lock` 和 sim 运动取消 `_stop_event`

#### `controller_node.py` — 控制节点 (sim-only)

sim 模式下的插值执行器：

- 订阅 `/target_joints` (23 关节, 角度制) → `ArmController` 插值 → 发布 `/joint_command` (弧度制) → Isaac Sim
- 支持 S 型曲线平滑插值 (默认) 和线性插值，通过 `enable_smooth` 参数/话题切换
- 速度模式 `velocity_mode`: slow(0.5x)/normal(1.0x)/fast(2.0x)
- 夹爪快捷控制：订阅 `/gripper_control` (String)，支持 `left_open`/`left_close`/`right_open`/`right_close` 指令
- 参数支持热加载（夹爪角度、速度等）

#### `arm_controller.py` — 插值核心 (sim-only)

与 ROS2 解耦的纯算法模块：

- 管理 23 关节的当前/目标/指令位置
- 线性插值：恒定速度 `joint_velocity` (度/秒) 逐步逼近
- S 型曲线插值：根据加速度 `acceleration` (度/秒²) 自动平滑加减速，计算减速距离判断加速/匀速/减速阶段

#### `gripper_bridge.py` — 夹爪桥接

- sim 模式：夹爪关节由 `unified_arm_node` 合并到 `/target_joints` 控制，本节点仅发布 `gripper_result` 反馈
- real 模式：`ModbusGripper` 封装 Modbus RTU 通信，per-arm Lock 保证同臂不并发，线程化执行不阻塞 ROS 回调
- real 模式 2Hz 周期轮询夹持状态，发布 `gripper_holding`

#### `gui_node.py` — GUI 节点

基于 tkinter 的调试界面，支持：
- 23 关节滑块直接控制 (`/target_joints`)
- 夹爪快捷按钮
- 臂 Pose 命令发送 (movel/movejp)
- IK/FK 辅助计算 (通过 `RealManAlgo`)

#### `joint_names.py` — 关节定义

23 关节名称列表、关节限位、夹爪关节索引映射等常量定义。

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
| 寄存器: 力矩到达 (只读) | 0x0601 |
| 寄存器: 位置到达 (只读) | 0x0602 |
| 寄存器: 准备完成 (只读) | 0x0604 (力矩到达 OR 位置到达) |
| 寄存器: 反馈位置 (只读) | 0x0609 (32位) |
| 寄存器: 报警信息 (只读) | 0x0612 (bit flags: 过温/堵转/超速/掉落等) |

## 话题接口

> 全局话题带 namespace 前缀（默认 `robot`，即 `/robot/...`）。臂专用话题不带 namespace，`{arm}` = `arm_a` 或 `arm_b`。底盘话题硬编码 `/base_controller/...`。

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/robot/joint_states` | sensor_msgs/JointState | 订阅 | Isaac Sim 实时状态 (23关节) |
| `/robot/joint_command` | sensor_msgs/JointState | 发布 | 发送给 Isaac Sim 的指令 (23关节) |
| `/robot/target_joints` | std_msgs/Float64MultiArray | 订阅 | 目标位置输入 (23关节, 角度制, 兼容19) |
| `/robot/gripper_target` | std_msgs/Float64MultiArray | 订阅/发布 | 夹爪目标 (4关节: L1,L11,R1,R11, 弧度制) |
| `/robot/gripper_control` | std_msgs/String | 订阅 | 夹爪快捷指令 (left_open/left_close/right_open/right_close) |
| `/robot/enable_smooth` | std_msgs/Bool | 订阅 | 动态切换平滑插值开关 |
| `/robot/velocity_mode` | std_msgs/String | 订阅 | 速度模式切换 (slow/normal/fast) |
| `/robot/{arm}/gripper_result` | std_msgs/Bool | 发布 | 夹爪动作结果 (仅目标变化的臂发布) |
| `{arm}/rm_driver/movel_cmd` | rm_ros_interfaces/Movel | 订阅 | 直线运动命令 |
| `{arm}/rm_driver/movej_cmd` | rm_ros_interfaces/Movej | 订阅 | 关节运动命令 |
| `{arm}/rm_driver/movej_p_cmd` | rm_ros_interfaces/Movejp | 订阅 | 关节位置运动命令 |
| `{arm}/rm_driver/movep_cmd` | rm_ros_interfaces/Movel | 订阅 | 点到点运动命令 |
| `{arm}/rm_driver/move_stop_cmd` | std_msgs/Empty | 订阅 | 停止运动命令 |
| `{arm}/rm_driver/movel_result` | std_msgs/Bool | 发布 | 直线运动结果 |
| `{arm}/rm_driver/movej_result` | std_msgs/Bool | 发布 | 关节运动结果 |
| `{arm}/rm_driver/movej_p_result` | std_msgs/Bool | 发布 | 关节位置运动结果 |
| `{arm}/rm_driver/movep_result` | std_msgs/Bool | 发布 | 点到点运动结果 |
| `{arm}/rm_driver/udp_arm_position` | geometry_msgs/Pose | 发布 | 臂末端位姿反馈 |
| `{arm}/joint_states` | sensor_msgs/JointState | 发布 | 臂6关节状态反馈 |
| `{arm}/base_pose_cmd` | std_msgs/Float64MultiArray | 订阅 | 基座坐标系位姿命令 [x,y,z,rx,ry,rz] (跳过世界坐标变换) |
| `/base_controller/rotate_cmd` | std_msgs/Float64 | 订阅 | D1 底盘旋转命令 (sim) |
| `/base_controller/rotate_result` | std_msgs/Bool | 发布 | D1 旋转结果 (sim) |
| `/base_controller/current_angle` | std_msgs/Float64 | 发布 | D1 当前角度反馈 (sim) |
| `/robot/{arm}/gripper_holding` | std_msgs/Bool | 发布 | 夹爪持有状态 (real 模式, 2Hz Modbus 轮询) |

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
ros2 topic pub --once /robot/target_joints std_msgs/Float64MultiArray \
  "data: [0,0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0]"

# 平台旋转45度
ros2 topic pub --once /robot/target_joints std_msgs/Float64MultiArray \
  "data: [45, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0]"

# 左夹爪闭合 (弧度制, /robot/gripper_target)
ros2 topic pub --once /robot/gripper_target std_msgs/Float64MultiArray \
  "data: [1.0, -1.0, 0.0, 0.0]"

# 右夹爪闭合
ros2 topic pub --once /robot/gripper_target std_msgs/Float64MultiArray \
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
pub = node.create_publisher(Float64MultiArray, '/robot/target_joints', 10)
gripper_pub = node.create_publisher(Float64MultiArray, '/robot/gripper_target', 10)
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
│   └── triarm_config.yaml          # 配置参数 (含 base 坐标系、Modbus 串口、夹爪角度)
├── launch/
│   └── triarm_control.launch.py    # 启动文件 (4个节点)
├── triarm_control/
│   ├── joint_names.py              # 23关节定义 + 限位 + 夹爪索引
│   ├── rm65_robot.py               # RM65 运动学算法库 (自定义base坐标系 IK/FK)
│   ├── realman_sdk_wrapper.py      # SDK 模式感知封装 (RealManAlgo + RealManSDKWrapper)
│   ├── arm_controller.py           # sim-only 插值核心 (线性 + S型曲线)
│   ├── controller_node.py          # ROS2 控制节点 (插值 → /joint_command)
│   ├── unified_arm_node.py         # 统一臂节点 (ArmBridge + rm_driver兼容 + IK/FK + 夹爪合并)
│   ├── gripper_bridge.py           # 夹爪桥接 (sim: result反馈 / real: Modbus RTU)
│   └── gui_node.py                 # GUI 节点 (关节滑块 + 夹爪 + Pose + IK/FK)
├── package.xml
└── setup.py
```

## 配置参数

### triarm_controller

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `namespace` | `"robot"` | 话题命名空间 |
| `mode` | `"sim"` | 运行模式: sim / real |
| `joint_velocity` | `30.0` | 关节速度 (度/秒) |
| `publish_rate` | `50.0` | 发布频率 (Hz) |
| `enable_smooth` | `true` | 是否启用 S 型曲线平滑插值 |
| `acceleration` | `100.0` | 加速度限制 (度/秒²) |
| `velocity_mode` | `"normal"` | 速度模式: slow(0.5x) / normal(1.0x) / fast(2.0x) |
| `left_gripper.sim_open_angle` | `0.0` | sim 模式左夹爪打开角度 (度) |
| `left_gripper.sim_close_angle` | `-30.0` | sim 模式左夹爪闭合角度 (度) |
| `left_gripper.real_open_angle` | `0.0` | real 模式左夹爪打开角度 (度) |
| `left_gripper.real_close_angle` | `-30.0` | real 模式左夹爪闭合角度 (度) |
| `right_gripper.*` | 同上 | 右夹爪参数，结构同左夹爪 |

### unified_arm_node

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `namespace` | `"robot"` | 话题命名空间 |
| `mode` | `"sim"` | 运行模式 |
| `publish_rate` | `20.0` | 状态发布频率 (Hz) |
| `sim_joint_tolerance` | `0.02` | sim 运动到位阈值 (rad) |
| `sim_motion_timeout` | `10.0` | sim 运动超时 (秒) |
| `arm_a.ip` / `arm_b.ip` | `192.168.1.18/19` | 臂 IP 地址 (real) |
| `arm_a.port` / `arm_b.port` | `8080` | 臂端口 (real) |
| `arm_a.base_position` | `[0.05457, -0.04863, 0.2273]` | A 臂 base 相对转盘位置 [x,y,z] (m) |
| `arm_a.base_orientation_deg` | `[45.0, 90.0, 0.0]` | A 臂 base 相对转盘姿态 [rx,ry,rz] ZYX (deg) |
| `arm_a.d6_mm` | `144` | A 臂末端 d6 参数 (mm) |
| `arm_b.base_position` | `[0.0, 0.0, 0.0]` | B 臂 base 参数 (待校准) |
| `arm_b.base_orientation_deg` | `[0.0, 0.0, 0.0]` | B 臂 base 参数 (待校准) |
| `arm_b.d6_mm` | `144` | B 臂末端 d6 参数 (mm) |

### gripper_bridge_node

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `namespace` | `"robot"` | 话题命名空间 |
| `mode` | `"sim"` | 运行模式 |
| `gripper_a.serial_port` | `/dev/ttyUSB0` | 左夹爪串口 (real) |
| `gripper_a.baud_rate` | `115200` | 波特率 |
| `gripper_a.slave_addr` | `1` | Modbus 从站地址 |
| `gripper_b.serial_port` | `/dev/ttyUSB1` | 右夹爪串口 (real) |
| `gripper_b.baud_rate` | `115200` | 波特率 |
| `gripper_b.slave_addr` | `1` | Modbus 从站地址 |
| `motion_timeout` | `5.0` | Modbus 运动完成等待超时 (秒, real 模式) |

### triarm_gui

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `namespace` | `"robot"` | 话题命名空间 |
| `gui_width` | `700` | GUI 宽度 |
| `gui_height` | `600` | GUI 高度 |

## 依赖

- `rclpy`, `sensor_msgs`, `std_msgs`, `geometry_msgs` — ROS2 基础
- `rm_ros_interfaces` — RealMan 臂运动消息 (Movel/Movej/Movejp)
- `Robotic_Arm` — RealMan SDK (Algo DH参数初始化 + real TCP 控制)
- `numpy` — 齐次变换矩阵运算 (rm65_robot.py)
- `transforms3d` — 四元数/欧拉角转换
- `minimalmodbus` — real 模式夹爪 Modbus RTU 控制
- `tkinter` — GUI (可选)
