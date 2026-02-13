# 开发指南

## 项目结构

```
realman_sim2real_sdk/
├── src/triarm_control/              # ROS 2 功能包
│   ├── triarm_control/              # Python 模块
│   │   ├── __init__.py
│   │   ├── realman_sdk_wrapper.py   # RM65 SDK 封装 (Algo IK/FK + TCP)
│   │   ├── unified_arm_node.py      # 臂桥接 + D1旋转 + sim反馈闭环
│   │   ├── gripper_bridge.py        # 夹爪桥接 (sim mock / real SDK)
│   │   ├── controller_node.py       # 平滑插值控制器
│   │   ├── arm_controller.py        # 插值算法核心 (纯逻辑, 不依赖 ROS)
│   │   ├── gui_node.py              # GUI 界面 (可选)
│   │   └── joint_names.py           # 19关节名称 + 限位
│   ├── launch/
│   │   └── triarm_control.launch.py
│   ├── config/
│   │   └── triarm_config.yaml
│   ├── resource/
│   ├── package.xml
│   └── setup.py
├── docs/
│   ├── architecture.md   # 架构设计
│   ├── api.md             # 话题接口
│   ├── usage.md           # 使用指南
│   └── development.md     # 本文件
├── Makefile
└── README.md
```

---

## 核心模块说明

### realman_sdk_wrapper.py

两个类：

- `RealManAlgo` — 纯算法层，封装 `Robotic_Arm` SDK 的 IK/FK 功能
  - 不需要网络连接，sim/real 都用
  - `inverse_kinematics()` 返回弧度制关节角度
  - `forward_kinematics()` 返回末端位姿 dict

- `RealManSDKWrapper` — 模式感知的统一接口
  - sim 模式：仅实例化 Algo，不建立 TCP
  - real 模式：Algo + TCP 连接到 RM65
  - 返回 `SDKMotionResult` 枚举

### unified_arm_node.py

核心节点，包含：

- `ArmBridge` 类（每臂一个实例）
  - 订阅 rm_driver 兼容话题 (movel_cmd, movej_cmd, movej_p_cmd)
  - 发布结果话题 (movel_result, movej_result, movej_p_result)
  - 发布反馈话题 (joint_states, udp_arm_position)
  - 持有 `RealManSDKWrapper` 实例
  - 关节索引映射：`ARM_JOINT_INDICES = {'arm_a': slice(1,7), 'arm_b': slice(7,13)}`

- `UnifiedArmNode` 类
  - 管理所有 ArmBridge 实例
  - 维护 `_shared_target[19]`（角度制，线程安全）
  - 处理 D1 旋转命令
  - sim 模式下订阅 Isaac Sim `/joint_states` 并拆分反馈

关键方法：

```python
# 更新共享目标并发布
def _update_and_publish_target(self, indices, joints_rad):
    """将指定索引的关节角度(弧度)写入共享目标(度)并发布"""

# sim 反馈回调
def _sim_state_cb(self, msg):
    """从 Isaac Sim /joint_states 拆分各臂关节 + D1 角度"""

# sim 底盘旋转
def _exec_sim_rotate(self, target_angle):
    """更新 D1 到共享目标，轮询 _current_d1_angle 直到到位"""
```

### gripper_bridge.py

`GripperBridge` 类（每臂一个实例）：

- 订阅 3 种夹爪命令话题
- sim 模式：`time.sleep(sim_gripper_delay)` + result=True
- real 模式：调用 SDK 对应方法

### controller_node.py

`TriarmControllerNode` 类：

- 订阅 `/robot/target_joints`（19 关节，度）
- 内部使用 `ArmController` 做平滑插值
- 发布 `/robot/joint_command`（19 关节，度）
- 支持动态切换插值模式和速度

### arm_controller.py

纯逻辑层，不依赖 ROS 2：

- S 型曲线插值 / 线性插值
- 速度和加速度控制
- 关节限位检查
- 回调机制与外部通信

### joint_names.py

19 关节的名称和限位定义：

```python
JOINT_NAMES_LIST = [
    'joint_platform_D1',       # index 0
    'joint_platform_A1', ...,  # index 1-6
    'joint_platform_B1', ...,  # index 7-12
    'joint_platform_S1', ...,  # index 13-18
]

JOINT_LIMITS = {
    'joint_platform_D1': (deg2rad(-180), deg2rad(180)),
    # ...
}
```

---

## 开发环境搭建

```bash
# ROS 2 Humble
sudo apt-get install ros-humble-desktop

# Python 依赖
pip install Robotic_Arm transforms3d
sudo apt-get install python3-tk  # GUI 可选

# rm_ros_interfaces (RealMan 官方消息包)
# 需单独编译，参考 RealMan 文档

# 编译
cd realman_sim2real_sdk
make build
```

---

## 代码规范

遵循 PEP 8，关键约定：

- 类名：大驼峰 (`ArmBridge`, `UnifiedArmNode`)
- 方法名：小写+下划线 (`_sim_move_to_pose`)
- 私有方法：前缀下划线
- 常量：全大写 (`ARM_JOINT_INDICES`, `JOINT_NAMES_LIST`)
- 类型注解：推荐使用

---

## 扩展指南

### 添加新臂

1. 在 `joint_names.py` 中添加关节名称和限位
2. 在 `unified_arm_node.py` 的 `ARM_JOINT_INDICES` 中添加索引映射
3. 在 launch 文件中添加新臂的参数
4. 扩展 `_shared_target` 数组长度

### 添加新的运动类型

在 `ArmBridge` 中：

1. 添加新的订阅者（命令话题）
2. 添加新的发布者（结果话题）
3. 实现 sim/real 两种模式的处理逻辑
4. sim 模式：SDK IK → 更新共享目标 → 轮询到位
5. real 模式：SDK TCP 调用

### 修改插值算法

在 `arm_controller.py` 中添加新的插值方法：

```python
class ArmController:
    def step(self) -> bool:
        if self.interpolation_mode == 'custom':
            return self._step_custom()
        # ...

    def _step_custom(self) -> bool:
        """自定义插值算法"""
        pass
```

---

## 调试

### ROS 2 日志

```python
self.get_logger().debug('调试信息')
self.get_logger().info('普通信息')
self.get_logger().warn('警告信息')
self.get_logger().error('错误信息')
```

启动时设置日志级别：

```bash
ros2 run triarm_control unified_arm --ros-args --log-level debug
```

### 可视化工具

```bash
rqt_graph          # 查看话题连接图
rqt_plot           # 绘制数据曲线
ros2 topic hz ...  # 查看话题频率
```

### 常见调试场景

| 场景 | 检查方法 |
|------|----------|
| sim 反馈闭环断开 | `ros2 topic hz /robot/joint_states` |
| IK 无解 | 查看 unified_arm_node 日志中的 IK_FAILED |
| 共享目标冲突 | 检查 `_shared_target_lock` 是否正确使用 |
| 到位检测不触发 | 调整 `sim_joint_tolerance` 参数 |

---

## 测试

### 手动测试

```bash
# 启动 sim 模式
ros2 launch triarm_control triarm_control.launch.py mode:=sim

# 发送底盘旋转
ros2 topic pub --once /base_controller/rotate_cmd std_msgs/Float64 "data: 90.0"

# 发送 19 关节目标
ros2 topic pub --once /robot/target_joints std_msgs/Float64MultiArray \
  "data: [0, 0,30,-20,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0]"

# 检查反馈
ros2 topic echo arm_a/rm_driver/udp_arm_position
ros2 topic echo /base_controller/current_angle
```

---

## 提交规范

使用 Conventional Commits：

```
feat: 添加新功能
fix: 修复bug
docs: 更新文档
refactor: 重构代码
test: 添加测试
```
