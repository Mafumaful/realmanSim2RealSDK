# 系统架构

## 整体架构图

```
┌─────────────────────────────────────────────────────────────────┐
│                         用户层                                    │
├─────────────────────────────────────────────────────────────────┤
│  GUI界面 (可选)          │  外部程序 (Python/C++)                │
│  - 滑块控制              │  - 矩阵输入                           │
│  - 实时监控              │  - 编程控制                           │
└──────────┬───────────────┴──────────┬────────────────────────────┘
           │                          │
           │ /target_joints           │ /target_joints
           │ (Float64MultiArray)      │ (Float64MultiArray)
           ▼                          ▼
┌─────────────────────────────────────────────────────────────────┐
│                      控制层                                       │
├─────────────────────────────────────────────────────────────────┤
│                   controller_node                                │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │              ArmController (核心控制器)                    │   │
│  │  - 插值算法 (线性/S型曲线)                                │   │
│  │  - 速度控制 (slow/normal/fast)                           │   │
│  │  - 关节限位保护                                           │   │
│  └──────────────────────────────────────────────────────────┘   │
└──────────┬──────────────────────────────────────────────────────┘
           │
           │ /joint_command (JointState)
           ▼
┌─────────────────────────────────────────────────────────────────┐
│                      执行层                                       │
├─────────────────────────────────────────────────────────────────┤
│  ┌──────────────────┐              ┌──────────────────┐         │
│  │  unified_arm     │              │  gripper_bridge  │         │
│  │  (机械臂统一接口) │              │  (夹爪桥接)       │         │
│  └────────┬─────────┘              └────────┬─────────┘         │
│           │                                 │                   │
│           ▼                                 ▼                   │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │              模式选择 (mode参数)                          │   │
│  │  ┌──────────────┐          ┌──────────────┐             │   │
│  │  │  sim模式     │          │  real模式    │             │   │
│  │  │  Isaac Sim   │          │  真实机械臂  │             │   │
│  │  └──────────────┘          └──────────────┘             │   │
│  └─────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

## 核心模块

### 1. ArmController (核心控制器)

**职责**：
- 关节运动插值计算
- 速度和加速度控制
- 关节限位检查
- 与ROS2解耦的纯逻辑层

**特性**：
- 支持线性插值和S型曲线插值
- 动态速度调节 (0.5x / 1.0x / 2.0x)
- 可开关的平滑处理
- 回调机制与外部通信

**文件位置**：`triarm_control/arm_controller.py`

---

### 2. controller_node (控制节点)

**职责**：
- ROS2话题通信
- 调用ArmController执行控制
- 定时发布关节指令

**订阅话题**：
- `/joint_states` - 实时关节状态
- `/target_joints` - 目标位置输入
- `/enable_smooth` - 平滑开关
- `/velocity_mode` - 速度模式

**发布话题**：
- `/joint_command` - 关节控制指令

**文件位置**：`triarm_control/controller_node.py`

---

### 3. gui_node (GUI节点)

**职责**：
- 提供可视化控制界面
- 实时显示关节状态
- 发送目标位置到controller_node

**特性**：
- 分页显示19个关节
- 滑块和输入框双向同步
- 实时值(蓝色) / 指令值(绿色) 对比显示
- 平滑处理和速度模式控制

**文件位置**：`triarm_control/gui_node.py`

---

### 4. unified_arm_node (统一机械臂接口)

**职责**：
- 桥接不同模式下的机械臂控制
- sim模式：直接转发到Isaac Sim
- real模式：调用真实机械臂SDK

**文件位置**：`triarm_control/unified_arm_node.py`

---

### 5. gripper_bridge (夹爪桥接)

**职责**：
- 夹爪控制接口
- 支持sim/real模式切换

**文件位置**：`triarm_control/gripper_bridge.py`

---

## 数据流

### 仿真模式 (mode=sim)

```
用户输入 → /target_joints → controller_node → /joint_command → Isaac Sim
                                    ↑
                              /joint_states ← Isaac Sim
```

### 真机模式 (mode=real)

```
用户输入 → /target_joints → controller_node → /joint_command → unified_arm → 真实机械臂
                                    ↑
                              /joint_states ← unified_arm ← 真实机械臂
```

## 插值算法

### 线性插值 (enable_smooth=false)

```python
max_delta = velocity * dt
if abs(diff) > max_delta:
    diff = max_delta * sign(diff)
position += diff
```

**特点**：
- 简单快速
- 恒定速度
- 可能有抖动

---

### S型曲线插值 (enable_smooth=true)

```python
# 加速阶段
if velocity < max_velocity:
    velocity += acceleration * dt

# 匀速阶段
velocity = max_velocity

# 减速阶段
if distance < decel_distance:
    velocity -= acceleration * dt

position += velocity * dt
```

**特点**：
- 平滑加减速
- 无抖动
- 更符合真实机械臂特性

---

## 关节配置

### 19个关节分布

| 组 | 关节 | 数量 | 说明 |
|----|------|------|------|
| 平台 | D1 | 1 | 平台旋转 |
| 机械臂A | A1-A6 | 6 | 6自由度机械臂 |
| 机械臂B | B1-B6 | 6 | 6自由度机械臂 |
| 机械臂S | S1-S6 | 6 | 6自由度机械臂 |

### 关节限位示例

```python
JOINT_LIMITS = {
    'joint_platform_D1': (-180°, 180°),
    'joint_platform_A1': (-180°, 180°),
    'joint_platform_A2': (-129.5°, 129.5°),
    # ... 更多限位配置
}
```

**文件位置**：`triarm_control/joint_names.py`

---

## 配置参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `mode` | `sim` | 运行模式 (sim/real) |
| `joint_velocity` | `30.0` | 基准速度 (度/秒) |
| `publish_rate` | `50.0` | 发布频率 (Hz) |
| `enable_smooth` | `true` | 启用平滑插值 |
| `acceleration` | `100.0` | 加速度 (度/秒²) |
| `velocity_mode` | `normal` | 速度模式 (slow/normal/fast) |

**配置文件**：`config/triarm_config.yaml`
