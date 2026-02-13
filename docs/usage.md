# 使用指南

## 系统依赖

```bash
# ROS 2 Humble
sudo apt-get install ros-humble-desktop

# Python 依赖
pip install Robotic_Arm transforms3d
sudo apt-get install python3-tk   # GUI 可选

# ROS 2 消息包
# rm_ros_interfaces (RealMan 官方 ROS 2 消息包，需单独编译)
```

---

## 编译

```bash
cd realman_sim2real_sdk
make build
```

---

## 启动方式

### 仿真模式 (与 Isaac Sim 配合)

```bash
# 无 GUI
ros2 launch triarm_control triarm_control.launch.py mode:=sim

# 带 GUI
ros2 launch triarm_control triarm_control.launch.py mode:=sim with_gui:=true
```

启动后会运行以下节点：
- `unified_arm_node` — 臂桥接 + D1 旋转 + sim 反馈闭环
- `gripper_bridge_node` — 夹爪桥接 (sim mock)
- `controller_node` — 平滑插值控制器
- `gui_node` — 可选 GUI

### 真机模式

```bash
ros2 launch triarm_control triarm_control.launch.py mode:=real
```

Real 模式下 unified_arm_node 会通过 TCP 连接 RM65 机械臂。

### 单独启动节点

```bash
ros2 run triarm_control unified_arm
ros2 run triarm_control gripper_bridge
ros2 run triarm_control controller
ros2 run triarm_control gui
```

---

## 与上层状态机集成

本 SDK 设计为被 `contact_graspnet_ros2` 状态机层调用。状态机通过 rm_driver 兼容话题控制臂和夹爪：

```
状态机 (ArmController)     → arm_a/rm_driver/movel_cmd    → unified_arm_node
状态机 (GripperController) → arm_a/rm_driver/set_gripper_* → gripper_bridge
状态机 (BaseController)    → /base_controller/rotate_cmd   → unified_arm_node (sim)
```

### 完整系统启动顺序

1. 启动 Isaac Sim（发布 `/robot/joint_states`，消费 `/robot/joint_command`）
2. 启动本 SDK：`ros2 launch triarm_control triarm_control.launch.py mode:=sim`
3. 启动视觉服务：`ros2 launch contact_graspnet_ros2 grasp_hybrid_server.launch.py`
4. 启动状态机：`ros2 run contact_graspnet_ros2 grasp_state_machine`
5. 发送启动指令：`ros2 topic pub --once /state_machine/start std_msgs/Empty`

---

## Sim 模式工作原理

### 正向数据流

```
状态机发送 movel_cmd
  → unified_arm_node 收到 Pose
  → SDK Algo IK 计算关节角度
  → 更新 _shared_target[19] 对应索引
  → 发布 /robot/target_joints (19关节, 度)
  → controller_node 平滑插值
  → 发布 /robot/joint_command (19关节, 度)
  → Isaac Sim 执行
```

### 反馈数据流

```
Isaac Sim 发布 /robot/joint_states (19关节, 弧度)
  → unified_arm_node._sim_state_cb 收到
  → 拆分: positions[1:7] → ArmBridge:A, positions[7:13] → ArmBridge:B
  → 到位检测: max(|current - target|) < 0.02 rad → 发布 movel_result=True
  → FK 回算: 关节角度 → 末端 Pose → 发布 udp_arm_position
  → D1 角度: positions[0] → 度 → 发布 /base_controller/current_angle
```

### 到位检测参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `sim_joint_tolerance` | 0.02 rad (≈1.15°) | 臂关节到位阈值 |
| `sim_motion_timeout` | 10.0 s | 运动超时 |
| D1 到位容差 | 0.5° | 底盘旋转到位阈值 |

---

## Real 模式工作原理

### 臂控制

```
状态机发送 movel_cmd
  → unified_arm_node 收到 Pose
  → SDK TCP movel() 发送到 RM65
  → RM65 执行运动
  → SDK 查询关节状态 → 发布 joint_states + udp_arm_position
  → 发布 movel_result
```

### 底盘控制

Real 模式下底盘由外部独立控制器处理，不走 unified_arm_node。
BaseController 仍通过 `/base_controller/rotate_cmd` 发送命令，由外部驱动响应。

### 夹爪控制

```
状态机发送 set_gripper_pick_on_cmd
  → gripper_bridge 收到
  → SDK gripper_pick_on() 发送到 RM65
  → 发布 result
```

---

## GUI 使用

GUI 分为 4 个标签页：平台 (D1)、机械臂 A (A1-A6)、机械臂 B (B1-B6)、机械臂 S (S1-S6)

每个关节行包含：

| 列 | 说明 |
|----|------|
| 关节 | 关节编号 |
| 实时值 (蓝色) | 来自 Isaac Sim / 真机的当前位置 |
| 指令值 (绿色) | 正在发送的控制指令 |
| 滑块 | 拖动调节目标角度 |
| 输入框 | 精确输入目标角度 |

操作流程：
1. 启动 GUI：`ros2 launch triarm_control triarm_control.launch.py with_gui:=true`
2. 等待实时值显示数据
3. 使用滑块或输入框设置目标角度
4. 点击"执行"按钮
5. 观察指令值逐渐接近目标

---

## 调试技巧

### 查看话题

```bash
# 列出所有话题
ros2 topic list

# 监控臂控制命令
ros2 topic echo arm_a/rm_driver/movel_cmd

# 监控臂运动结果
ros2 topic echo arm_a/rm_driver/movel_result

# 监控末端位姿反馈
ros2 topic echo arm_a/rm_driver/udp_arm_position

# 监控底盘角度
ros2 topic echo /base_controller/current_angle

# 监控 Isaac Sim 关节状态
ros2 topic echo /robot/joint_states
```

### 查看节点

```bash
ros2 node list
# 应该看到:
# /unified_arm_node
# /gripper_bridge_node
# /triarm_controller
```

### 手动发送命令

```bash
# 底盘旋转到 90°
ros2 topic pub --once /base_controller/rotate_cmd std_msgs/Float64 "data: 90.0"

# 直接发送 19 关节目标 (度)
ros2 topic pub --once /robot/target_joints std_msgs/Float64MultiArray \
  "data: [0, 0,30,-20,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0]"
```

### 日志级别

```bash
ros2 run triarm_control unified_arm --ros-args --log-level debug
```

---

## 常见问题

### Q1: sim 模式下臂不动

检查：
1. Isaac Sim 是否运行并发布 `/robot/joint_states`
2. controller_node 是否运行：`ros2 node list | grep controller`
3. 话题是否连通：`ros2 topic hz /robot/joint_command`

### Q2: movel_result 一直不返回

可能原因：
- Isaac Sim 未发布 `/robot/joint_states`（sim 反馈闭环断开）
- 目标位姿 IK 无解
- 运动超时（默认 10s）

### Q3: 底盘旋转后角度不对

检查 `/base_controller/current_angle` 是否有数据：
```bash
ros2 topic echo /base_controller/current_angle
```
sim 模式下该话题由 unified_arm_node 从 Isaac Sim joint_states 的 D1 分量桥接。

### Q4: real 模式连接失败

确认：
1. RM65 机械臂已上电
2. 网络连通：`ping 192.168.1.18`
3. `Robotic_Arm` SDK 已安装：`pip install Robotic_Arm`
