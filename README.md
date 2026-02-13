# realman_sim2real_sdk

三臂抓取平台 — Sim/Real 双模式桥接层

## 项目简介

本项目是三臂机器人平台的 ROS 2 桥接层，位于上层状态机 (`contact_graspnet_ros2`) 和底层执行器 (Isaac Sim / 真实 RM65) 之间。

核心职责：将 rm_driver 兼容话题翻译为 sim 模式的 `/target_joints` 或 real 模式的 SDK TCP 指令，并提供完整的反馈闭环。

```
状态机 → rm_driver 兼容话题 → unified_arm_node ─┬─ sim: SDK Algo IK → /target_joints → 插值 → Isaac Sim
                                                 └─ real: SDK TCP → RM65
                                                    ↑ /joint_states ← Isaac Sim (反馈闭环)
```

## 硬件拓扑

```
base_link (世界固定)
  └─ joint_platform_D1 (旋转关节 0°/90°/180°)
       └─ platform_link
            ├─ A臂 (左臂, A1-A6 + 夹爪)
            ├─ B臂 (右臂, B1-B6 + 夹爪)
            └─ S臂 (头部, S1-S6, 固定不动, 搭载 Gemini 355 RGBD)

关节编号 (19关节):
  D1(底盘) | A1-A6(左臂) | B1-B6(右臂) | S1-S6(头部)
  index 0  | index 1-6   | index 7-12  | index 13-18
```

## 快速开始

```bash
# 编译
make build

# 仿真模式启动 (无GUI)
ros2 launch triarm_control triarm_control.launch.py mode:=sim

# 真机模式启动
ros2 launch triarm_control triarm_control.launch.py mode:=real

# 带 GUI
ros2 launch triarm_control triarm_control.launch.py mode:=sim with_gui:=true
```

## 节点清单

| 节点 | 可执行文件 | 职责 |
|------|-----------|------|
| `unified_arm_node` | `unified_arm` | A/B 臂桥接 + D1 底盘旋转 + sim 反馈闭环 |
| `gripper_bridge_node` | `gripper_bridge` | A/B 臂夹爪桥接 (real: SDK, sim: mock+延时) |
| `triarm_controller` | `controller` | 平滑插值控制器 (/target_joints → /joint_command) |
| `triarm_gui` | `gui` | 可选 GUI 界面 |

## 项目结构

```
realman_sim2real_sdk/
├── src/triarm_control/
│   ├── triarm_control/
│   │   ├── realman_sdk_wrapper.py   # RM65 SDK 封装 (Algo IK/FK + TCP)
│   │   ├── unified_arm_node.py      # 臂桥接 + D1旋转 + sim反馈闭环
│   │   ├── gripper_bridge.py        # 夹爪桥接 (sim mock / real SDK)
│   │   ├── controller_node.py       # 平滑插值控制器
│   │   ├── arm_controller.py        # 插值算法核心 (纯逻辑)
│   │   ├── gui_node.py              # GUI 界面
│   │   └── joint_names.py           # 19关节名称 + 限位
│   ├── launch/
│   │   └── triarm_control.launch.py
│   └── config/
│       └── triarm_config.yaml
├── docs/
│   ├── architecture.md   # 架构设计
│   ├── api.md             # 话题接口
│   ├── usage.md           # 使用指南
│   └── development.md     # 开发指南
├── Makefile
└── README.md
```

## 文档

- [架构设计](./docs/architecture.md) — 双模式数据流、反馈闭环、共享目标机制
- [API 接口](./docs/api.md) — rm_driver 兼容话题、底盘话题、Isaac Sim 接口
- [使用指南](./docs/usage.md) — 启动方式、模式切换、调试技巧
- [开发指南](./docs/development.md) — 代码结构、扩展方法、测试

## 系统要求

- ROS 2 Humble
- Python 3.8+
- `Robotic_Arm` SDK (`pip install Robotic_Arm`)
- `rm_ros_interfaces` (RealMan ROS 2 消息包)
- `transforms3d` (`pip install transforms3d`)
- tkinter (GUI 可选)

## 与上层系统的关系

本 SDK 是三臂抓取平台的桥接层，完整系统还包括：

- `contact_graspnet_ros2` — 状态机层 (grasp_state_machine + grasp_hybrid_server + interfaces)
- Isaac Sim — 仿真环境 (发布 /joint_states, 消费 /joint_command)
- Gemini 355/305 RGBD — 视觉感知 (全局 + 腕部相机)

详细系统流程参见 `docs/system_flow.md`。
