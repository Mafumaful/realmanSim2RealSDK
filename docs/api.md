# API接口文档

## ROS2话题接口

### 输入话题（订阅）

#### 1. `/joint_states`

**消息类型**：`sensor_msgs/JointState`

**功能**：接收机械臂的实时关节状态

**数据格式**：
```python
{
    "header": {
        "stamp": {...},
        "frame_id": ""
    },
    "name": [
        "joint_platform_D1",
        "joint_platform_A1", "joint_platform_A2", ...,
        "joint_platform_B1", "joint_platform_B2", ...,
        "joint_platform_S1", "joint_platform_S2", ...
    ],
    "position": [0.0, 0.0, ...],  # 19个关节位置（弧度）
    "velocity": [],
    "effort": []
}
```

**发布者**：
- sim模式：Isaac Sim
- real模式：unified_arm_node

**订阅者**：
- controller_node
- gui_node

---

#### 2. `/target_joints`

**消息类型**：`std_msgs/Float64MultiArray`

**功能**：接收目标关节位置（矩阵输入接口）

**数据格式**：
```python
{
    "data": [
        0.0,                              # D1 (度)
        0.0, 30.0, -20.0, 0.0, 0.0, 0.0,  # A1-A6 (度)
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,     # B1-B6 (度)
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,     # S1-S6 (度)
    ]
}
```

**要求**：
- 必须包含19个元素
- 单位：度（不是弧度）
- 会自动应用关节限位

**发布者**：
- gui_node
- 外部程序

**订阅者**：
- controller_node

**示例**：
```bash
ros2 topic pub --once /target_joints std_msgs/Float64MultiArray \
  "data: [0,0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0]"
```

---

#### 3. `/enable_smooth`

**消息类型**：`std_msgs/Bool`

**功能**：动态开关平滑插值

**数据格式**：
```python
{
    "data": true  # true=启用S型曲线, false=线性插值
}
```

**发布者**：
- gui_node
- 外部程序

**订阅者**：
- controller_node

**示例**：
```bash
# 启用平滑
ros2 topic pub --once /enable_smooth std_msgs/Bool "data: true"

# 禁用平滑
ros2 topic pub --once /enable_smooth std_msgs/Bool "data: false"
```

---

#### 4. `/velocity_mode`

**消息类型**：`std_msgs/String`

**功能**：动态调节运动速度

**数据格式**：
```python
{
    "data": "normal"  # "slow" | "normal" | "fast"
}
```

**速度映射**：
- `"slow"`: 0.5x (15度/秒)
- `"normal"`: 1.0x (30度/秒)
- `"fast"`: 2.0x (60度/秒)

**发布者**：
- gui_node
- 外部程序

**订阅者**：
- controller_node

**示例**：
```bash
ros2 topic pub --once /velocity_mode std_msgs/String "data: 'fast'"
```

---

### 输出话题（发布）

#### 1. `/joint_command`

**消息类型**：`sensor_msgs/JointState`

**功能**：发送关节控制指令

**数据格式**：
```python
{
    "header": {
        "stamp": {...},
        "frame_id": ""
    },
    "name": [
        "joint_platform_D1",
        "joint_platform_A1", ...,
        # 19个关节名称
    ],
    "position": [0.0, 0.0, ...],  # 19个目标位置（弧度）
    "velocity": [],
    "effort": []
}
```

**发布频率**：50Hz（可配置）

**发布者**：
- controller_node

**订阅者**：
- sim模式：Isaac Sim
- real模式：unified_arm_node
- gui_node（用于显示）

---

## Python API

### ArmController 类

核心控制器类，可独立使用（不依赖ROS2）。

#### 初始化

```python
from triarm_control.arm_controller import ArmController

controller = ArmController(
    joint_velocity=30.0,    # 基准速度（度/秒）
    publish_rate=50.0,      # 控制频率（Hz）
    enable_smooth=True,     # 启用平滑插值
    acceleration=100.0      # 加速度（度/秒²）
)
```

#### 设置回调函数

```python
def on_command(positions):
    """当有新指令时调用"""
    print(f"发送指令: {positions}")

def on_complete():
    """运动完成时调用"""
    print("运动完成")

controller.set_command_callback(on_command)
controller.set_motion_complete_callback(on_complete)
```

#### 更新当前状态

```python
# 从传感器获取当前位置
joint_names = ["joint_platform_D1", "joint_platform_A1", ...]
positions = [0.0, 0.1, 0.2, ...]  # 弧度

controller.update_current_state(joint_names, positions)
```

#### 设置目标位置

```python
# 方式1: 设置所有关节
targets = [0.0] * 19
targets[1] = 30.0  # A1转30度
controller.set_target_positions(targets, in_degrees=True)

# 方式2: 设置单个关节
controller.set_single_joint_target(1, 30.0, in_degrees=True)
```

#### 开始运动

```python
if controller.start_motion():
    print("开始运动")
else:
    print("等待状态数据")
```

#### 执行控制步进

```python
# 在定时器中调用
while controller.step():
    time.sleep(1.0 / 50.0)  # 50Hz
```

#### 停止运动

```python
controller.stop_motion()
```

#### 动态调节参数

```python
# 开关平滑处理
controller.set_smooth_enabled(True)

# 设置速度模式
controller.set_velocity_mode('fast')  # 'slow' | 'normal' | 'fast'
```

#### 获取状态

```python
# 获取当前位置（度）
current = controller.get_current_positions_deg()

# 获取指令位置（度）
cmd = controller.get_cmd_positions_deg()

# 获取目标位置（度）
target = controller.get_target_positions_deg()
```

---

### 完整示例

```python
import time
from triarm_control.arm_controller import ArmController

# 创建控制器
controller = ArmController(
    joint_velocity=30.0,
    publish_rate=50.0,
    enable_smooth=True
)

# 设置回调
def on_command(positions):
    print(f"指令: {[f'{p:.2f}' for p in positions[:3]]}")

controller.set_command_callback(on_command)

# 模拟接收状态
controller.update_current_state(
    ["joint_platform_D1"] + [f"joint_platform_A{i}" for i in range(1, 7)],
    [0.0] * 7
)

# 设置目标
targets = [0.0] * 19
targets[1] = 30.0  # A1转30度
controller.set_target_positions(targets, in_degrees=True)

# 开始运动
if controller.start_motion():
    # 执行控制循环
    while controller.step():
        time.sleep(1.0 / 50.0)
    print("运动完成")
```

---

## 关节配置

### 关节名称列表

```python
from triarm_control.joint_names import JOINT_NAMES_LIST

# 19个关节名称
JOINT_NAMES_LIST = [
    'joint_platform_D1',      # 索引0
    'joint_platform_A1',      # 索引1
    'joint_platform_A2',      # 索引2
    'joint_platform_A3',      # 索引3
    'joint_platform_A4',      # 索引4
    'joint_platform_A5',      # 索引5
    'joint_platform_A6',      # 索引6
    'joint_platform_B1',      # 索引7
    'joint_platform_B2',      # 索引8
    'joint_platform_B3',      # 索引9
    'joint_platform_B4',      # 索引10
    'joint_platform_B5',      # 索引11
    'joint_platform_B6',      # 索引12
    'joint_platform_S1',      # 索引13
    'joint_platform_S2',      # 索引14
    'joint_platform_S3',      # 索引15
    'joint_platform_S4',      # 索引16
    'joint_platform_S5',      # 索引17
    'joint_platform_S6',      # 索引18
]
```

### 关节限位

```python
from triarm_control.joint_names import JOINT_LIMITS

# 获取某个关节的限位
limits = JOINT_LIMITS['joint_platform_A1']
min_rad, max_rad = limits  # 弧度

# 转换为角度
import math
min_deg = math.degrees(min_rad)
max_deg = math.degrees(max_rad)
```

### 完整限位表

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

## 配置文件

### triarm_config.yaml

```yaml
# 运行模式
mode: sim  # sim | real

# 控制参数
joint_velocity: 30.0      # 基准速度（度/秒）
publish_rate: 50.0        # 发布频率（Hz）
enable_smooth: true       # 启用平滑插值
acceleration: 100.0       # 加速度（度/秒²）
velocity_mode: normal     # slow | normal | fast

# GUI参数
gui_width: 700
gui_height: 500

# 命名空间
namespace: ""
```

### 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `mode` | string | `sim` | 运行模式 |
| `joint_velocity` | float | `30.0` | 基准速度 |
| `publish_rate` | float | `50.0` | 控制频率 |
| `enable_smooth` | bool | `true` | 平滑插值 |
| `acceleration` | float | `100.0` | 加速度限制 |
| `velocity_mode` | string | `normal` | 速度模式 |
| `gui_width` | int | `700` | GUI宽度 |
| `gui_height` | int | `500` | GUI高度 |
| `namespace` | string | `""` | 话题命名空间 |

---

## 错误码

### 控制器错误

| 错误 | 说明 | 解决方法 |
|------|------|----------|
| `尚未收到关节状态数据` | 未接收到 `/joint_states` | 检查Isaac Sim或真机连接 |
| `目标位置数量错误` | 输入的关节数不是19个 | 确保发送19个关节位置 |
| `关节索引超出范围` | 关节索引不在0-18 | 检查索引值 |

### 日志级别

```bash
# INFO: 正常运行信息
[INFO] [triarm_controller]: 控制节点已启动

# WARN: 警告信息
[WARN] [triarm_controller]: 等待关节状态数据...

# ERROR: 错误信息
[ERROR] [triarm_controller]: 目标位置数量错误
```

---

## 性能指标

### 控制频率

- **默认**：50Hz
- **推荐范围**：30-100Hz
- **最大**：200Hz（需要高性能硬件）

### 延迟

- **仿真模式**：< 20ms
- **真机模式**：< 50ms

### 精度

- **位置精度**：0.001 rad (≈0.06°)
- **速度精度**：0.1 度/秒

---

## 单位转换

### 角度 ↔ 弧度

```python
import math

# 角度转弧度
rad = math.radians(deg)

# 弧度转角度
deg = math.degrees(rad)
```

### 速度单位

- **配置文件**：度/秒
- **内部计算**：弧度/秒
- **自动转换**：由控制器处理
