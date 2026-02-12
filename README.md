# realmanSim2RealSDK

三臂机器人仿真到真机的控制SDK

## 项目简介

本项目是一个用于控制三臂机器人平台的ROS2功能包，支持Isaac Sim仿真环境和真实机械臂的无缝切换。集成了：
- 关节运动控制
- 平滑插值算法
- GUI可视化界面
- 矩阵输入接口
- 真机/仿真模式切换

## 快速开始

```bash
# 编译
make build

# 运行（无GUI）
make run

# 运行（带GUI）
make run-gui
```

## 文档

详细文档请查看 [docs](./docs) 目录：

- [架构设计](./docs/architecture.md) - 系统架构和模块说明
- [使用指南](./docs/usage.md) - 详细使用方法
- [API接口](./docs/api.md) - 话题接口和数据格式
- [开发指南](./docs/development.md) - 二次开发说明

## 项目结构

```
realmanSim2RealSDK/
├── src/triarm_control/          # 主功能包
│   ├── triarm_control/          # Python模块
│   │   ├── arm_controller.py    # 核心控制器
│   │   ├── controller_node.py   # ROS2控制节点
│   │   ├── gui_node.py          # GUI节点
│   │   ├── unified_arm_node.py  # 统一机械臂接口
│   │   └── gripper_bridge.py    # 夹爪桥接
│   ├── launch/                  # 启动文件
│   └── config/                  # 配置文件
├── docs/                        # 文档目录
├── Makefile                     # 快捷命令
└── README.md                    # 本文件
```

## 主要特性

- ✅ 仿真/真机模式切换
- ✅ 平滑S型曲线插值
- ✅ 可选GUI界面
- ✅ 矩阵输入接口
- ✅ 速度动态调节
- ✅ 关节限位保护

## 系统要求

- ROS2 Humble
- Python 3.8+
- tkinter (GUI可选)

## 许可证

MIT License
