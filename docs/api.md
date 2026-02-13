# API 接口文档

## 话题总览

本 SDK 提供三类话题接口：

1. **rm_driver 兼容话题** — 上层状态机通过这些话题控制臂和夹爪
2. **底盘控制话题** — D1 旋转控制
3. **Isaac Sim 接口话题** — 与仿真环境的数据交换

---

## 1. 臂控制话题 (rm_driver 兼容)

每臂独立命名空间：`{arm}` = `arm_a` (左臂) 或 `arm_b` (右臂)

### 命令话题 (状态机 → unified_arm_node)

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `{arm}/rm_driver/movel_cmd` | `rm_ros_interfaces/Movel` | 直线运动 (末端位姿) |
| `{arm}/rm_driver/movej_cmd` | `rm_ros_interfaces/Movej` | 关节空间运动 (关节角度) |
| `{arm}/rm_driver/movej_p_cmd` | `rm_ros_interfaces/Movejp` | 关节空间到位姿运动 |

### 结果话题 (unified_arm_node → 状态机)

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `{arm}/rm_driver/movel_result` | `std_msgs/Bool` | 直线运动结果 |
| `{arm}/rm_driver/movej_result` | `std_msgs/Bool` | 关节运动结果 |
| `{arm}/rm_driver/movej_p_result` | `std_msgs/Bool` | 关节到位姿运动结果 |

### 反馈话题 (unified_arm_node → 状态机)

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `{arm}/rm_driver/udp_arm_position` | `geometry_msgs/Pose` | 末端位姿反馈 (FK 回算) |
| `{arm}/joint_states` | `sensor_msgs/JointState` | 单臂 6 关节状态 (弧度) |

### Sim 模式行为

```
movel_cmd → SDK Algo IK → 关节角度 → 更新 _shared_target → /target_joints
         → 轮询 _current_joints 直到到位 (< 0.02 rad)
         → 发布 movel_result = True/False
```

### Real 模式行为

```
movel_cmd → SDK TCP movel() → RM65 执行
         → SDK 查询关节状态 → 发布 joint_states + udp_arm_position
         → 发布 movel_result = True/False
```

---

## 2. 夹爪控制话题

每臂独立命名空间：`{arm}` = `arm_a` 或 `arm_b`

### 命令话题 (状态机 → gripper_bridge)

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `{arm}/rm_driver/set_gripper_pick_on_cmd` | `rm_ros_interfaces/Gripperpick` | 力控持续夹取 |
| `{arm}/rm_driver/set_gripper_pick_cmd` | `rm_ros_interfaces/Gripperpick` | 力控夹取 |
| `{arm}/rm_driver/set_gripper_position_cmd` | `rm_ros_interfaces/Gripperset` | 位置控制 |

### 结果话题 (gripper_bridge → 状态机)

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `{arm}/rm_driver/set_gripper_pick_on_result` | `std_msgs/Bool` | 力控持续夹取结果 |
| `{arm}/rm_driver/set_gripper_pick_result` | `std_msgs/Bool` | 力控夹取结果 |
| `{arm}/rm_driver/set_gripper_position_result` | `std_msgs/Bool` | 位置控制结果 |

### Sim 模式行为

```
gripper_cmd → sleep(sim_gripper_delay) → 发布 result=True (mock)
```

### Real 模式行为

```
gripper_cmd → SDK gripper_pick_on() / gripper_set_position() → 发布 result
```

---

## 3. 底盘控制话题

| 话题 | 消息类型 | 方向 | 说明 |
|------|----------|------|------|
| `/base_controller/rotate_cmd` | `std_msgs/Float64` | → unified_arm_node(sim) / 外部驱动(real) | 旋转命令 (目标角度°) |
| `/base_controller/rotate_result` | `std_msgs/Bool` | unified_arm_node(sim) / 外部驱动(real) → | 旋转结果 |
| `/base_controller/current_angle` | `std_msgs/Float64` | unified_arm_node(sim) → | 当前角度反馈 (°) |

### Sim 模式行为

```
rotate_cmd → 更新 _shared_target[0] (D1) → /target_joints
          → 轮询 _current_d1_angle 直到到位 (< 0.5°)
          → 发布 rotate_result = True/False
```

角度反馈来源：Isaac Sim `/joint_states` 的 positions[0] → 弧度转度 → 发布 `current_angle`

### Real 模式行为

底盘由外部独立控制器处理，不走 unified_arm_node。

---

## 4. Isaac Sim 接口话题

| 话题 | 消息类型 | 方向 | 单位 | 说明 |
|------|----------|------|------|------|
| `/robot/target_joints` | `Float64MultiArray` | unified_arm_node → controller_node | 度 | 19 关节目标 |
| `/robot/joint_command` | `Float64MultiArray` | controller_node → Isaac Sim | 度 | 插值后指令 |
| `/robot/joint_states` | `JointState` | Isaac Sim → unified_arm_node | 弧度 | 19 关节反馈 |

### 关节顺序 (19 关节)

```
索引 0:     D1      (底盘旋转)
索引 1-6:   A1-A6   (左臂)
索引 7-12:  B1-B6   (右臂)
索引 13-18: S1-S6   (头部, 固定)
```

### 单位注意

- `/target_joints` 和 `/joint_command`：角度制 (度)
- `/joint_states`：弧度制 (rad)
- unified_arm_node 内部自动转换

---

## 5. 插值控制器话题

| 话题 | 消息类型 | 方向 | 说明 |
|------|----------|------|------|
| `/enable_smooth` | `std_msgs/Bool` | → controller_node | 平滑插值开关 |
| `/velocity_mode` | `std_msgs/String` | → controller_node | 速度模式 (slow/normal/fast) |

速度映射：
- `slow`: 0.5x (15 度/秒)
- `normal`: 1.0x (30 度/秒)
- `fast`: 2.0x (60 度/秒)

---

## 6. 状态机控制话题

这些话题由上层 `grasp_state_machine` 使用，不属于本 SDK，但列出以便理解完整系统：

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `/state_machine/start` | `std_msgs/Empty` | 启动作业 |
| `/state_machine/reset` | `std_msgs/Empty` | 复位 |
| `/state_machine/fault_clear` | `std_msgs/Empty` | 清除故障 |
| `/state_machine/state` | `std_msgs/String` | 状态发布 |

---

## 配置参数

### triarm_config.yaml

```yaml
# 运行模式
mode: sim                    # sim | real

# 插值控制器参数
joint_velocity: 30.0         # 基准速度 (度/秒)
publish_rate: 50.0           # 控制频率 (Hz)
enable_smooth: true          # 启用 S 型曲线插值
acceleration: 100.0          # 加速度 (度/秒²)
velocity_mode: normal        # slow | normal | fast

# Sim 模式专用参数
sim_joint_tolerance: 0.02    # 到位检测阈值 (弧度)
sim_motion_timeout: 10.0     # 运动超时 (秒)
sim_gripper_delay: 0.5       # 夹爪模拟延时 (秒)

# GUI 参数 (可选)
gui_width: 700
gui_height: 500
```

### Launch 参数

```bash
ros2 launch triarm_control triarm_control.launch.py \
  mode:=sim \
  with_gui:=false
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `mode` | `sim` | 运行模式 |
| `with_gui` | `false` | 是否启动 GUI |

---

## 关节限位表

| 关节 | 最小值 | 最大值 |
|------|--------|--------|
| D1 | -180° | +180° |
| A1 | -180° | +180° |
| A2 | -129.5° | +129.5° |
| A3 | -129.5° | +129.5° |
| A4 | -180° | +180° |
| A5 | -126° | +126° |
| A6 | -355° | +355° |
| B1 | -180° | +180° |
| B2 | -129.5° | +129.5° |
| B3 | -129.5° | +129.5° |
| B4 | -180° | +180° |
| B5 | -126° | +126° |
| B6 | -355° | +355° |
| S1 | -153.6° | +153.6° |
| S2 | 0° | +189° |
| S3 | 0° | +180° |
| S4 | -88.8° | +88.8° |
| S5 | -88.8° | +88.8° |
| S6 | -171.9° | +171.9° |

---

## SDK Python API

### RealManAlgo

```python
from triarm_control.realman_sdk_wrapper import RealManAlgo

algo = RealManAlgo()

# 逆运动学: 末端位姿 → 关节角度 (弧度)
joints = algo.inverse_kinematics(
    x=0.3, y=0.0, z=0.4,
    rx=3.14, ry=0.0, rz=0.0,
    q_ref=[0.0] * 6  # 参考关节角度
)

# 正运动学: 关节角度 → 末端位姿
pose = algo.forward_kinematics([0.0, 0.5, -0.3, 0.0, 0.8, 0.0])
# → {'x': ..., 'y': ..., 'z': ..., 'rx': ..., 'ry': ..., 'rz': ...}
```

### RealManSDKWrapper

```python
from triarm_control.realman_sdk_wrapper import RealManSDKWrapper, SDKMotionResult

# Sim 模式 (仅 IK/FK, 无 TCP)
sdk = RealManSDKWrapper(mode='sim')

# Real 模式 (IK/FK + TCP)
sdk = RealManSDKWrapper(mode='real', ip='192.168.1.18', port=8080)

# 运动指令 (real 模式)
result = sdk.movel(x, y, z, rx, ry, rz, speed=20)
# → SDKMotionResult.SUCCESS / FAILED / TIMEOUT / NOT_CONNECTED / IK_FAILED
```
