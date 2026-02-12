# 使用指南

## 安装

### 系统依赖

```bash
# ROS2 Humble
sudo apt-get install ros-humble-desktop

# Python依赖
sudo apt-get install python3-tk python3-pip

# ROS2消息类型
sudo apt-get install ros-humble-sensor-msgs ros-humble-std-msgs
```

### 编译项目

```bash
cd ~/Code/realmanSim2RealSDK
make build
```

编译完成后会在 `install/` 目录生成安装文件。

---

## 启动方式

### 方式1: 使用Makefile（推荐）

```bash
# 仅控制节点（无GUI）
make run

# 控制节点 + GUI
make run-gui
```

### 方式2: 使用launch文件

```bash
# 仿真模式，无GUI
ros2 launch triarm_control triarm_control.launch.py

# 仿真模式，带GUI
ros2 launch triarm_control triarm_control.launch.py with_gui:=true

# 真机模式，带GUI
ros2 launch triarm_control triarm_control.launch.py mode:=real with_gui:=true
```

### 方式3: 单独启动节点

```bash
# 控制节点
ros2 run triarm_control controller

# GUI节点
ros2 run triarm_control gui

# 统一机械臂节点
ros2 run triarm_control unified_arm

# 夹爪桥接节点
ros2 run triarm_control gripper_bridge
```

---

## 矩阵输入接口

### 关节顺序

19个关节按以下顺序排列：

```
索引 0:     D1      (平台旋转)
索引 1-6:   A1-A6   (机械臂A)
索引 7-12:  B1-B6   (机械臂B)
索引 13-18: S1-S6   (机械臂S)
```

### 命令行发送

```bash
# 所有关节归零
ros2 topic pub --once /target_joints std_msgs/Float64MultiArray \
  "data: [0,0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0]"

# 平台旋转45度
ros2 topic pub --once /target_joints std_msgs/Float64MultiArray \
  "data: [45, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0]"

# 机械臂A的A2关节转30度，A3转-20度
ros2 topic pub --once /target_joints std_msgs/Float64MultiArray \
  "data: [0, 0,30,-20,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0]"

# 多个机械臂同时运动
ros2 topic pub --once /target_joints std_msgs/Float64MultiArray \
  "data: [0, 10,20,0,0,0,0, 15,25,0,0,0,0, 20,30,0,0,0,0]"
```

### Python代码发送

#### 简单示例

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

rclpy.init()
node = Node('target_sender')
pub = node.create_publisher(Float64MultiArray, '/target_joints', 10)

# 等待连接建立
time.sleep(0.5)

# 构建目标位置 (19个关节，单位：度)
msg = Float64MultiArray()
msg.data = [
    0.0,                              # D1 平台
    0.0, 30.0, -20.0, 0.0, 0.0, 0.0,  # A1-A6
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,     # B1-B6
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,     # S1-S6
]

pub.publish(msg)
print("目标位置已发送")

node.destroy_node()
rclpy.shutdown()
```

#### 完整示例（带状态监控）

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import math

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')

        # 发布目标位置
        self.target_pub = self.create_publisher(
            Float64MultiArray, '/target_joints', 10)

        # 订阅实时状态
        self.state_sub = self.create_subscription(
            JointState, '/joint_states', self.state_callback, 10)

        self.current_positions = [0.0] * 19

    def state_callback(self, msg):
        """接收实时关节状态"""
        if msg.position:
            # 转换为角度
            self.current_positions = [math.degrees(p) for p in msg.position]
            print(f"当前位置: {[f'{p:.1f}' for p in self.current_positions[:3]]}")

    def send_target(self, positions):
        """发送目标位置"""
        msg = Float64MultiArray()
        msg.data = positions
        self.target_pub.publish(msg)
        self.get_logger().info(f'发送目标: {positions[:3]}...')

def main():
    rclpy.init()
    controller = ArmController()

    # 等待连接
    import time
    time.sleep(1.0)

    # 发送目标位置
    target = [0.0] * 19
    target[1] = 30.0  # A1转30度
    target[2] = -20.0 # A2转-20度
    controller.send_target(target)

    # 持续监控状态
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## GUI使用

### 界面说明

GUI分为4个标签页：
- **平台**：控制D1关节
- **机械臂A**：控制A1-A6
- **机械臂B**：控制B1-B6
- **机械臂S**：控制S1-S6

### 控制元素

每个关节行包含：

| 列 | 说明 |
|----|------|
| **关节** | 关节编号 |
| **实时值** (蓝色) | 来自机械臂的当前位置 |
| **指令值** (绿色) | 正在发送的控制指令 |
| **滑块** | 拖动调节目标角度 |
| **输入框** | 精确输入目标角度 |

### 按钮功能

- **执行**：开始运动到设定的目标位置
- **归零**：将所有目标角度设为0°
- **平滑处理**：勾选启用S型曲线插值
- **速度选择**：slow (0.5x) / normal (1.0x) / fast (2.0x)

### 操作流程

1. 启动GUI：`make run-gui`
2. 等待"实时值"显示数据（蓝色数字）
3. 使用滑块或输入框设置目标角度
4. 点击"执行"按钮
5. 观察"指令值"（绿色）逐渐接近目标

---

## 速度控制

### 动态调节速度

#### 通过GUI

在GUI界面选择速度模式：
- **slow**: 15度/秒 (0.5x)
- **normal**: 30度/秒 (1.0x)
- **fast**: 60度/秒 (2.0x)

#### 通过话题

```bash
# 设置为快速模式
ros2 topic pub --once /velocity_mode std_msgs/String "data: 'fast'"

# 设置为慢速模式
ros2 topic pub --once /velocity_mode std_msgs/String "data: 'slow'"

# 恢复正常速度
ros2 topic pub --once /velocity_mode std_msgs/String "data: 'normal'"
```

### 平滑处理开关

#### 通过GUI

勾选/取消勾选"平滑处理"复选框

#### 通过话题

```bash
# 启用平滑处理
ros2 topic pub --once /enable_smooth std_msgs/Bool "data: true"

# 禁用平滑处理
ros2 topic pub --once /enable_smooth std_msgs/Bool "data: false"
```

---

## 模式切换

### 仿真模式 (sim)

```bash
ros2 launch triarm_control triarm_control.launch.py mode:=sim
```

**特点**：
- 与Isaac Sim通信
- 无需真实硬件
- 用于算法验证

### 真机模式 (real)

```bash
ros2 launch triarm_control triarm_control.launch.py mode:=real
```

**特点**：
- 控制真实机械臂
- 需要硬件连接
- 生产环境使用

---

## 调试技巧

### 查看话题列表

```bash
ros2 topic list
```

### 监控话题数据

```bash
# 查看实时状态
ros2 topic echo /joint_states

# 查看控制指令
ros2 topic echo /joint_command

# 查看目标输入
ros2 topic echo /target_joints
```

### 查看话题频率

```bash
# 查看发布频率
ros2 topic hz /joint_command

# 应该显示约50Hz
```

### 查看节点信息

```bash
# 列出所有节点
ros2 node list

# 查看节点详情
ros2 node info /triarm_controller
```

### 日志级别

```bash
# 启动时设置日志级别
ros2 run triarm_control controller --ros-args --log-level debug
```

---

## 常见问题

### Q1: GUI显示"等待数据..."

**原因**：未收到 `/joint_states` 消息

**解决**：
1. 检查Isaac Sim是否运行
2. 检查话题：`ros2 topic list | grep joint_states`
3. 检查数据：`ros2 topic echo /joint_states`

### Q2: 机械臂不动

**原因**：
- 未启动controller_node
- 未启动unified_arm_node
- 目标位置与当前位置相同

**解决**：
```bash
# 检查节点是否运行
ros2 node list

# 应该看到：
# /triarm_controller
# /unified_arm_node
```

### Q3: 运动不平滑

**解决**：
1. 启用平滑处理
2. 降低速度模式
3. 检查发布频率是否为50Hz

### Q4: 关节超限

**现象**：输入的角度被自动限制

**说明**：这是正常的安全保护机制，每个关节都有限位

**查看限位**：参考 `triarm_control/joint_names.py`

---

## 性能优化

### 提高响应速度

```yaml
# config/triarm_config.yaml
publish_rate: 100.0  # 提高到100Hz
```

### 调整运动速度

```yaml
joint_velocity: 45.0  # 提高基准速度
acceleration: 150.0   # 提高加速度
```

### 禁用平滑处理

```yaml
enable_smooth: false  # 使用线性插值
```

**注意**：禁用平滑可能导致抖动
