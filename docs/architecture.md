# 系统架构

## 整体定位

本 SDK 是三臂机器人平台的 ROS 2 桥接层，位于上层状态机 (`contact_graspnet_ros2`) 和底层执行器 (Isaac Sim / 真实 RM65) 之间。

核心职责：将 rm_driver 兼容话题翻译为 sim 模式的 `/target_joints` 或 real 模式的 SDK TCP 指令，并提供完整的反馈闭环。

---

## 软件节点架构

```
┌─────────────────────────────────────────────────────────────────────┐
│                contact_graspnet_ros2 (状态机层)                       │
│                                                                     │
│  ┌──────────────────┐  ┌──────────────┐  ┌──────────────────────┐  │
│  │ grasp_state_     │  │ grasp_hybrid │  │ interfaces/          │  │
│  │ machine          │  │ _server      │  │  arm_controller      │  │
│  │ (主控状态机)      │  │ (视觉感知)   │  │  gripper_controller  │  │
│  │                  │  │              │  │  base_controller     │  │
│  └───────┬──────────┘  └──────┬───────┘  └──────────┬───────────┘  │
│          │                    │                      │              │
│          │  GetGrasps srv     │   rm_driver 兼容话题  │              │
│          ├────────────────────┘                      │              │
└──────────┼──────────────────────────────────────────┼──────────────┘
           │                                          │
           │  arm_x/rm_driver/movel_cmd               │
           │  arm_x/rm_driver/movej_cmd               │
           │  arm_x/rm_driver/set_gripper_*_cmd       │
           │  /base_controller/rotate_cmd             │
           │                                          │
┌──────────┼──────────────────────────────────────────┼──────────────┐
│          │     realman_sim2real_sdk (桥接层)          │              │
│          ▼                                          ▼              │
│  ┌──────────────────┐  ┌──────────────┐  ┌──────────────────┐     │
│  │ unified_arm_node │  │ gripper_     │  │ controller_node  │     │
│  │ (臂桥接+D1旋转)  │  │ bridge       │  │ (插值控制器)      │     │
│  │                  │  │ (夹爪桥接)    │  │                  │     │
│  │ sim: SDK IK →    │  │ sim: mock    │  │ /target_joints   │     │
│  │   /target_joints │  │   + delay    │  │   → 平滑插值     │     │
│  │ real: SDK TCP    │  │ real: SDK    │  │   → /joint_cmd   │     │
│  └──────────────────┘  └──────────────┘  └──────────────────┘     │
│          │                                          │              │
└──────────┼──────────────────────────────────────────┼──────────────┘
           │                                          │
           │  /robot/target_joints (19关节,度)         │
           │  /robot/joint_command (19关节,度)         │
           │  /robot/joint_states  (19关节,弧度)       │
           ▼                                          ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    Isaac Sim / 真实机械臂 RM65                        │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 核心模块

### 1. UnifiedArmNode — 臂桥接 + D1 旋转 + sim 反馈闭环

文件：`triarm_control/unified_arm_node.py`

这是 SDK 的核心节点，内部包含：

- **ArmBridge (每臂一个实例)**：管理单臂的 rm_driver 兼容话题和 SDK 实例
- **共享目标数组 `_shared_target[19]`**：19 关节的统一目标（角度制），防止多臂发布冲突
- **D1 旋转处理**：订阅 `/base_controller/rotate_cmd`，更新共享目标的 index 0
- **sim 反馈回路**：从 Isaac Sim `/joint_states` 拆分各臂关节 + FK 回算末端位姿

#### ArmBridge 内部结构

```
ArmBridge (per arm: arm_a / arm_b)
  ├─ 订阅 rm_driver 兼容话题:
  │    movel_cmd / movej_cmd / movej_p_cmd
  ├─ 发布结果话题:
  │    movel_result / movej_result / movej_p_result
  ├─ 发布反馈话题:
  │    joint_states / udp_arm_position
  ├─ SDK 实例 (RealManSDKWrapper):
  │    sim: Algo IK/FK only
  │    real: Algo + TCP
  └─ 关节索引映射:
       arm_a → [1:7], arm_b → [7:13]
```

#### 共享目标机制

```
_shared_target[19] (角度制, 线程安全)
  [D1, A1,A2,A3,A4,A5,A6, B1,B2,B3,B4,B5,B6, S1..S6]
   ↑        ↑                  ↑
  D1旋转  ArmBridge:A       ArmBridge:B
  更新idx0 更新idx1-6        更新idx7-12

→ 每次更新后发布完整 19 关节到 /robot/target_joints
```

---

### 2. RealManSDKWrapper — RM65 SDK 封装

文件：`triarm_control/realman_sdk_wrapper.py`

两个类：

- **RealManAlgo**：纯算法层（IK/FK），不需要网络连接
  - `inverse_kinematics(x, y, z, rx, ry, rz, q_ref)` → 关节角度（弧度）
  - `forward_kinematics(joints)` → `{x, y, z, rx, ry, rz}`

- **RealManSDKWrapper**：模式感知的统一接口
  - sim 模式：仅使用 Algo（IK/FK），不建立 TCP 连接
  - real 模式：Algo + TCP 控制（`movel` / `movej` / `movej_p`）

---

### 3. GripperBridge — 夹爪桥接

文件：`triarm_control/gripper_bridge.py`

每臂一个实例，订阅 rm_driver 夹爪话题：

- sim 模式：`time.sleep(sim_gripper_delay)` + 返回 `result=True`（Isaac Sim 夹爪未配置）
- real 模式：调用 SDK `gripper_pick_on` / `gripper_pick` / `gripper_set_position`

---

### 4. TriarmControllerNode — 平滑插值控制器

文件：`triarm_control/controller_node.py`

- 订阅 `/robot/target_joints`（19 关节，角度制）
- 应用 S 型曲线 / 线性插值
- 发布 `/robot/joint_command`（19 关节，角度制）→ Isaac Sim

---

## Sim 模式数据流（核心）

```
                          ┌─────────────────────┐
                          │   grasp_state_machine│
                          │   (状态机主控)        │
                          └──┬───┬───┬───┬──────┘
                             │   │   │   │
              ┌──────────────┘   │   │   └──────────────┐
              ▼                  ▼   ▼                  ▼
     arm_a/rm_driver/     arm_b/rm_driver/    /base_controller/
     movel_cmd            movel_cmd           rotate_cmd (Float64)
              │                  │                      │
              ▼                  ▼                      ▼
┌─────────────────────────────────────────────────────────────────┐
│                     unified_arm_node                             │
│                                                                 │
│  ┌─────────────┐  ┌─────────────┐  ┌────────────────────────┐  │
│  │ ArmBridge:A │  │ ArmBridge:B │  │ D1旋转处理             │  │
│  │             │  │             │  │                        │  │
│  │ Pose → SDK  │  │ Pose → SDK  │  │ rotate_cmd → 更新D1   │  │
│  │ Algo IK →  │  │ Algo IK →  │  │ 到共享目标数组         │  │
│  │ 关节角度    │  │ 关节角度    │  │                        │  │
│  └──────┬──────┘  └──────┬──────┘  └───────────┬────────────┘  │
│         │                │                      │               │
│         ▼                ▼                      ▼               │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │          _shared_target[19] (共享目标数组, 角度制)         │   │
│  │                                                          │   │
│  │  [D1, A1,A2,A3,A4,A5,A6, B1,B2,B3,B4,B5,B6, S1..S6]   │   │
│  └──────────────────────────┬───────────────────────────────┘   │
│                             │                                   │
│                    /robot/target_joints                          │
│                    (Float64MultiArray, 19关节)                    │
└─────────────────────────────┬───────────────────────────────────┘
                              │
                              ▼
                   ┌──────────────────────┐
                   │   controller_node     │
                   │   (平滑插值控制器)     │
                   │                      │
                   │ target_joints →      │
                   │   梯形速度规划 →      │
                   │   /robot/joint_command│
                   └──────────┬───────────┘
                              │
                              ▼
                   ┌──────────────────────┐
                   │     Isaac Sim         │
                   │                      │
                   │ joint_command → 仿真  │
                   │ → /robot/joint_states │
                   │   (19关节, 弧度)      │
                   └──────────┬───────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                  unified_arm_node (反馈回路)                      │
│                                                                 │
│  _sim_state_cb(joint_states):                                   │
│    ├─ positions[0] → D1角度(度) → /base_controller/current_angle│
│    ├─ positions[1:7]  → ArmBridge:A._current_joints (到位检测)  │
│    ├─ positions[7:13] → ArmBridge:B._current_joints (到位检测)  │
│    └─ FK(关节角度) → arm_x/rm_driver/udp_arm_position (末端Pose)│
│                                                                 │
│  到位检测: max(|current - target|) < 0.02 rad → 发布 result=True│
│  D1到位:   |current_d1 - target| < 0.5° → 发布 rotate_result    │
└─────────────────────────────────────────────────────────────────┘
```

---

## Real 模式数据流

```
状态机 → rm_driver话题 → unified_arm_node → SDK TCP → RM65
                                              ↑ SDK查询 → joint_states + udp_arm_position
底盘: BaseController → /base_controller/rotate_cmd → 独立底盘控制器
```

Real 模式下：
- ArmBridge 收到 movel_cmd → 调用 `RealManSDKWrapper.movel()` → SDK TCP 发送到 RM65
- SDK 查询关节状态 → 发布 `joint_states` + FK 回算 `udp_arm_position`
- 底盘旋转由外部独立控制器处理（不走 unified_arm_node）

---

## 坐标系关系

```
world (固定)
  └─ base_link (固定, 与world重合)
       └─ joint_platform_D1 (旋转, 0°/90°/180°)
            └─ platform_link ← 【臂基座坐标系, 抓取位姿参考系】
                 ├─ A臂 (joint_a1 ~ joint_a6)
                 │    └─ Link_A6 (末端) + 夹爪
                 ├─ B臂 (joint_b1 ~ joint_b6)
                 │    └─ Link_B6 (末端) + 夹爪
                 └─ S臂 (joint_s1 ~ joint_s6, 固定不动)
                      └─ Link_S6 ← 【相机安装位】
                           └─ Gemini 355 RGBD

关键: 相机和臂都在 platform_link 下，
      camera→platform_link 变换不随 D1 旋转变化 (静态)
      Waypoints 定义在 platform_link 坐标系下
```

---

## 关节编号

```
19关节:
  D1(底盘) | A1-A6(左臂) | B1-B6(右臂) | S1-S6(头部,固定)
  index 0  | index 1-6   | index 7-12  | index 13-18
```

| 组 | 关节 | 数量 | 索引 | 说明 |
|----|------|------|------|------|
| 平台 | D1 | 1 | 0 | 底盘旋转 (0°/90°/180°) |
| A臂 | A1-A6 | 6 | 1-6 | 左臂 6DOF + 夹爪 |
| B臂 | B1-B6 | 6 | 7-12 | 右臂 6DOF + 夹爪 |
| S臂 | S1-S6 | 6 | 13-18 | 头部固定，搭载 Gemini 355 |

---

## 插值算法

controller_node 提供两种插值模式：

### S 型曲线插值 (enable_smooth=true, 默认)

平滑加减速，无抖动，更符合真实机械臂特性。

### 线性插值 (enable_smooth=false)

恒定速度，简单快速，可能有抖动。

---

## 配置参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `mode` | `sim` | 运行模式 (sim/real) |
| `joint_velocity` | `30.0` | 基准速度 (度/秒) |
| `publish_rate` | `50.0` | 控制频率 (Hz) |
| `enable_smooth` | `true` | 启用平滑插值 |
| `acceleration` | `100.0` | 加速度 (度/秒²) |
| `sim_joint_tolerance` | `0.02` | sim 到位检测阈值 (弧度) |
| `sim_motion_timeout` | `10.0` | sim 运动超时 (秒) |
| `sim_gripper_delay` | `0.5` | sim 夹爪模拟延时 (秒) |
