# 开发指南

## 项目结构

```
realmanSim2RealSDK/
├── src/triarm_control/              # 主功能包
│   ├── triarm_control/              # Python模块
│   │   ├── __init__.py
│   │   ├── arm_controller.py        # 核心控制器（纯逻辑）
│   │   ├── controller_node.py       # ROS2控制节点
│   │   ├── gui_node.py              # GUI节点
│   │   ├── unified_arm_node.py      # 统一机械臂接口
│   │   ├── gripper_bridge.py        # 夹爪桥接
│   │   ├── realman_sdk_wrapper.py   # 真机SDK封装
│   │   └── joint_names.py           # 关节配置
│   ├── launch/
│   │   └── triarm_control.launch.py # 启动文件
│   ├── config/
│   │   └── triarm_config.yaml       # 配置文件
│   ├── resource/
│   ├── package.xml                  # ROS2包清单
│   └── setup.py                     # Python包配置
├── docs/                            # 文档目录
│   ├── architecture.md              # 架构设计
│   ├── usage.md                     # 使用指南
│   ├── api.md                       # API接口
│   └── development.md               # 本文件
├── Makefile                         # 快捷命令
└── README.md                        # 项目说明
```

---

## 开发环境搭建

### 1. 安装依赖

```bash
# ROS2 Humble
sudo apt-get install ros-humble-desktop

# Python开发工具
sudo apt-get install python3-pip python3-venv

# 代码格式化工具
pip3 install black flake8 pylint
```

### 2. 克隆项目

```bash
git clone <repository_url>
cd realmanSim2RealSDK
```

### 3. 编译

```bash
make build
```

### 4. 运行测试

```bash
# 启动仿真模式测试
make run

# 在另一个终端发送测试指令
ros2 topic pub --once /target_joints std_msgs/Float64MultiArray \
  "data: [0,0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0]"
```

---

## 代码规范

### Python风格

遵循 PEP 8 规范：

```python
# 类名：大驼峰
class ArmController:
    pass

# 函数名：小写+下划线
def set_target_positions():
    pass

# 常量：全大写
MAX_VELOCITY = 100.0

# 私有方法：前缀下划线
def _internal_method():
    pass
```

### 文档字符串

```python
def set_target_positions(self, positions: List[float], in_degrees: bool = True):
    """
    设置目标位置

    Args:
        positions: 目标位置列表 (19个关节)
        in_degrees: 是否为角度制，默认True

    Raises:
        ValueError: 如果关节数量不正确

    Example:
        >>> controller.set_target_positions([0.0] * 19)
    """
    pass
```

### 类型注解

```python
from typing import List, Optional, Callable

def update_state(self, names: List[str], positions: List[float]) -> None:
    pass

def get_positions(self) -> List[float]:
    return self.positions
```

---

## 添加新功能

### 1. 添加新的控制算法

在 `arm_controller.py` 中添加新的插值方法：

```python
class ArmController:
    def __init__(self, interpolation_mode='smooth'):
        self.interpolation_mode = interpolation_mode

    def step(self) -> bool:
        if self.interpolation_mode == 'linear':
            return self._step_linear()
        elif self.interpolation_mode == 'smooth':
            return self._step_smooth()
        elif self.interpolation_mode == 'custom':
            return self._step_custom()

    def _step_custom(self) -> bool:
        """自定义插值算法"""
        # 实现你的算法
        pass
```

### 2. 添加新的ROS2话题

在 `controller_node.py` 中添加：

```python
class TriarmControllerNode(Node):
    def __init__(self):
        super().__init__('triarm_controller')

        # 添加新的订阅者
        self.custom_sub = self.create_subscription(
            CustomMsg, '/custom_topic', self._custom_callback, 10)

        # 添加新的发布者
        self.custom_pub = self.create_publisher(
            CustomMsg, '/custom_output', 10)

    def _custom_callback(self, msg):
        """处理自定义消息"""
        # 处理逻辑
        pass
```

### 3. 添加新的GUI控件

在 `gui_node.py` 中添加：

```python
class JointControlGUI(Node):
    def _create_buttons(self):
        # 现有按钮
        # ...

        # 添加新按钮
        ttk.Button(btn_frame, text='自定义功能',
                   command=self._custom_action).pack(side='left', padx=5)

    def _custom_action(self):
        """自定义按钮动作"""
        # 实现功能
        pass
```

---

## 扩展关节数量

### 1. 修改关节配置

编辑 `joint_names.py`：

```python
JOINT_NAMES_LIST = [
    'joint_platform_D1',
    # ... 现有关节
    'joint_platform_C1',  # 新增关节
    'joint_platform_C2',
    # ...
]

JOINT_LIMITS = {
    # ... 现有限位
    'joint_platform_C1': (deg2rad(-180), deg2rad(180)),
    'joint_platform_C2': (deg2rad(-90), deg2rad(90)),
}
```

### 2. 更新控制器

`arm_controller.py` 会自动适配新的关节数量：

```python
self.num_joints = len(JOINT_NAMES_LIST)  # 自动更新
```

### 3. 更新GUI

在 `gui_node.py` 中添加新的标签页：

```python
def _create_gui(self):
    # ... 现有标签页
    self._create_arm_tab(notebook, 'C', 19)  # 新机械臂C
```

---

## 添加新的运行模式

### 1. 定义新模式

在 `controller_node.py` 中：

```python
class TriarmControllerNode(Node):
    def __init__(self):
        self.declare_parameter('mode', 'sim')
        self._mode = self.get_parameter('mode').value

        if self._mode == 'sim':
            self._setup_sim_mode()
        elif self._mode == 'real':
            self._setup_real_mode()
        elif self._mode == 'custom':
            self._setup_custom_mode()

    def _setup_custom_mode(self):
        """自定义模式设置"""
        # 实现自定义模式
        pass
```

### 2. 更新launch文件

在 `triarm_control.launch.py` 中：

```python
mode_arg = DeclareLaunchArgument(
    'mode',
    default_value='sim',
    choices=['sim', 'real', 'custom'],  # 添加新模式
    description='运行模式'
)
```

---

## 调试技巧

### 1. 使用ROS2日志

```python
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        # 不同级别的日志
        self.get_logger().debug('调试信息')
        self.get_logger().info('普通信息')
        self.get_logger().warn('警告信息')
        self.get_logger().error('错误信息')
```

启动时设置日志级别：

```bash
ros2 run triarm_control controller --ros-args --log-level debug
```

### 2. 使用Python调试器

```python
import pdb

def problematic_function():
    # 设置断点
    pdb.set_trace()
    # 代码继续执行
```

### 3. 可视化调试

使用 `rqt` 工具：

```bash
# 查看话题图
rqt_graph

# 绘制数据曲线
rqt_plot /joint_states/position[0]

# 查看所有话题
rqt_topic
```

---

## 性能优化

### 1. 提高控制频率

```yaml
# config/triarm_config.yaml
publish_rate: 100.0  # 从50Hz提高到100Hz
```

**注意**：需要确保硬件能够支持更高频率。

### 2. 减少计算开销

```python
# 避免重复计算
class ArmController:
    def __init__(self):
        # 预计算常量
        self._dt = 1.0 / self.publish_rate
        self._max_delta = math.radians(self.joint_velocity) * self._dt

    def step(self):
        # 直接使用预计算值
        if abs(diff) > self._max_delta:
            diff = self._max_delta * sign(diff)
```

### 3. 使用NumPy加速

```python
import numpy as np

class ArmController:
    def __init__(self):
        # 使用NumPy数组
        self.current_positions = np.zeros(self.num_joints)
        self.target_positions = np.zeros(self.num_joints)

    def step(self):
        # 向量化计算
        diff = self.target_positions - self.cmd_positions
        mask = np.abs(diff) > self._max_delta
        diff[mask] = self._max_delta * np.sign(diff[mask])
        self.cmd_positions += diff
```

---

## 测试

### 1. 单元测试

创建 `test/test_arm_controller.py`：

```python
import unittest
from triarm_control.arm_controller import ArmController

class TestArmController(unittest.TestCase):
    def setUp(self):
        self.controller = ArmController()

    def test_initialization(self):
        """测试初始化"""
        self.assertEqual(self.controller.num_joints, 19)
        self.assertFalse(self.controller.is_moving)

    def test_set_target(self):
        """测试设置目标位置"""
        targets = [0.0] * 19
        targets[0] = 45.0
        self.controller.set_target_positions(targets)
        self.assertEqual(self.controller.target_positions[0],
                        math.radians(45.0))

    def test_joint_limits(self):
        """测试关节限位"""
        targets = [0.0] * 19
        targets[0] = 200.0  # 超出限位
        self.controller.set_target_positions(targets)
        # 应该被限制在180度
        self.assertLessEqual(
            math.degrees(self.controller.target_positions[0]), 180.0)

if __name__ == '__main__':
    unittest.main()
```

运行测试：

```bash
python3 -m pytest test/
```

### 2. 集成测试

创建测试脚本 `test/integration_test.py`：

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import time

class IntegrationTest(Node):
    def __init__(self):
        super().__init__('integration_test')
        self.pub = self.create_publisher(
            Float64MultiArray, '/target_joints', 10)
        self.sub = self.create_subscription(
            JointState, '/joint_command', self.callback, 10)
        self.received = False

    def callback(self, msg):
        self.received = True
        print(f"收到指令: {msg.position[:3]}")

    def run_test(self):
        # 发送目标
        msg = Float64MultiArray()
        msg.data = [0.0] * 19
        msg.data[0] = 45.0
        self.pub.publish(msg)

        # 等待响应
        timeout = 5.0
        start = time.time()
        while not self.received and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        assert self.received, "未收到响应"
        print("测试通过")

def main():
    rclpy.init()
    test = IntegrationTest()
    time.sleep(1.0)  # 等待连接
    test.run_test()
    test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 常见开发问题

### Q1: 如何添加新的消息类型？

**步骤**：

1. 创建消息定义文件 `msg/CustomMsg.msg`
2. 在 `package.xml` 中添加依赖
3. 在 `CMakeLists.txt` 中配置（如果使用C++）
4. 重新编译

### Q2: 如何调试ROS2节点？

**方法1**：使用日志

```python
self.get_logger().info(f'变量值: {value}')
```

**方法2**：使用VSCode调试

在 `.vscode/launch.json` 中配置：

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "ROS2 Debug",
            "type": "python",
            "request": "launch",
            "program": "${workspaceFolder}/install/triarm_control/lib/triarm_control/controller",
            "console": "integratedTerminal"
        }
    ]
}
```

### Q3: 如何处理ROS2参数？

```python
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        # 声明参数
        self.declare_parameter('my_param', 'default_value')

        # 获取参数
        value = self.get_parameter('my_param').value

        # 参数回调
        self.add_on_set_parameters_callback(self.param_callback)

    def param_callback(self, params):
        for param in params:
            if param.name == 'my_param':
                print(f'参数更新: {param.value}')
        return SetParametersResult(successful=True)
```

---

## 贡献指南

### 1. 代码提交流程

```bash
# 创建功能分支
git checkout -b feature/new-feature

# 开发并提交
git add .
git commit -m "feat: 添加新功能"

# 推送到远程
git push origin feature/new-feature

# 创建Pull Request
```

### 2. 提交信息规范

使用 Conventional Commits：

```
feat: 添加新功能
fix: 修复bug
docs: 更新文档
style: 代码格式调整
refactor: 重构代码
test: 添加测试
chore: 构建/工具链更新
```

### 3. 代码审查清单

- [ ] 代码符合PEP 8规范
- [ ] 添加了必要的文档字符串
- [ ] 添加了单元测试
- [ ] 通过所有测试
- [ ] 更新了相关文档

---

## 发布流程

### 1. 版本号管理

使用语义化版本：`MAJOR.MINOR.PATCH`

```
1.0.0 - 初始版本
1.1.0 - 添加新功能
1.1.1 - 修复bug
2.0.0 - 重大更新（不兼容旧版本）
```

### 2. 更新版本

在 `setup.py` 中：

```python
setup(
    name='triarm_control',
    version='1.1.0',  # 更新版本号
    # ...
)
```

### 3. 创建发布

```bash
# 打标签
git tag -a v1.1.0 -m "Release version 1.1.0"

# 推送标签
git push origin v1.1.0

# 创建Release（在GitHub上）
```

---

## 参考资源

### ROS2文档

- [ROS2官方文档](https://docs.ros.org/en/humble/)
- [rclpy API](https://docs.ros2.org/latest/api/rclpy/)
- [sensor_msgs](https://docs.ros2.org/latest/api/sensor_msgs/)

### Python资源

- [PEP 8风格指南](https://pep8.org/)
- [Python类型注解](https://docs.python.org/3/library/typing.html)
- [NumPy文档](https://numpy.org/doc/)

### 工具

- [VSCode ROS扩展](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros)
- [rqt工具集](http://wiki.ros.org/rqt)
- [Plotjuggler](https://github.com/facontidavide/PlotJuggler)

---

## 联系方式

如有问题或建议，请通过以下方式联系：

- 提交Issue
- 发送邮件
- 加入开发者社区
