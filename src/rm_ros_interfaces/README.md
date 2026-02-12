# rm_ros_interfaces

ROS2 interfaces for RealMan triarm robot control system.

## 概述

本功能包为睿尔曼三臂机器人系统提供标准化的ROS2接口定义，包括消息、服务和动作类型。

## 编译

```bash
cd /root/code/realmanSim2RealSDK
colcon build --packages-select rm_ros_interfaces
source install/setup.bash
```

## 消息类型 (msg)

### TriarmJointCommand
三臂机器人关节指令消息，包含19个关节的目标位置。

```python
from rm_ros_interfaces.msg import TriarmJointCommand

msg = TriarmJointCommand()
msg.header.stamp = self.get_clock().now().to_msg()
msg.joint_names = ['D1', 'A1', 'A2', ...]
msg.positions = [0.0, 0.1, 0.2, ...]  # 单位: 弧度
```

### GripperCommand
夹爪控制指令消息。

```python
from rm_ros_interfaces.msg import GripperCommand

msg = GripperCommand()
msg.gripper_id = "left"  # 或 "right"
msg.control_mode = "position"  # 或 "force"
msg.target_position = -30.0  # 单位: 度
msg.speed = 50.0
```

### ArmState
单个机械臂状态消息。

```python
from rm_ros_interfaces.msg import ArmState

# 订阅示例
self.create_subscription(
    ArmState,
    '/arm_a/state',
    self.arm_state_callback,
    10
)
```

### VelocityMode
速度模式消息。

```python
from rm_ros_interfaces.msg import VelocityMode

msg = VelocityMode()
msg.mode = "normal"  # "slow", "normal", "fast"
msg.velocity_scale = 1.0
```

### MotionResult
运动执行结果消息。

```python
from rm_ros_interfaces.msg import MotionResult

# 订阅示例
self.create_subscription(
    MotionResult,
    '/arm_a/motion_result',
    self.motion_result_callback,
    10
)
```

## 服务类型 (srv)

### SetVelocityMode
设置速度模式服务。

```python
from rm_ros_interfaces.srv import SetVelocityMode

# 客户端示例
client = self.create_client(SetVelocityMode, '/set_velocity_mode')
request = SetVelocityMode.Request()
request.mode = "fast"

try:
    future = client.call_async(request)
    rclpy.spin_until_future_complete(self, future)
    response = future.result()
    if response.success:
        self.get_logger().info(f'速度模式已设置: {response.velocity_scale}')
finally:
    if rclpy.ok():
        rclpy.shutdown()
```

### GetFK
正运动学求解服务。

```python
from rm_ros_interfaces.srv import GetFK

client = self.create_client(GetFK, '/get_fk')
request = GetFK.Request()
request.arm_id = "arm_a"
request.joint_positions = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]

try:
    future = client.call_async(request)
    rclpy.spin_until_future_complete(self, future)
    response = future.result()
    if response.success:
        pose = response.tcp_pose
        self.get_logger().info(f'TCP位姿: {pose}')
finally:
    if rclpy.ok():
        rclpy.shutdown()
```

### GetIK
逆运动学求解服务。

```python
from rm_ros_interfaces.srv import GetIK
from geometry_msgs.msg import Pose

client = self.create_client(GetIK, '/get_ik')
request = GetIK.Request()
request.arm_id = "arm_a"
request.target_pose = Pose()
request.target_pose.position.x = 0.3
request.target_pose.position.y = 0.0
request.target_pose.position.z = 0.5

try:
    future = client.call_async(request)
    rclpy.spin_until_future_complete(self, future)
    response = future.result()
    if response.success:
        joints = response.joint_positions
        self.get_logger().info(f'关节解: {joints}')
finally:
    if rclpy.ok():
        rclpy.shutdown()
```

### MoveL
笛卡尔空间直线运动服务。

```python
from rm_ros_interfaces.srv import MoveL
from geometry_msgs.msg import Pose

client = self.create_client(MoveL, '/arm_a/move_l')
request = MoveL.Request()
request.arm_id = "arm_a"
request.target_pose = Pose()
request.target_pose.position.x = 0.3
request.speed = 0.1  # m/s
request.blocking = True
request.timeout = 10.0

try:
    future = client.call_async(request)
    rclpy.spin_until_future_complete(self, future)
    response = future.result()
    self.get_logger().info(f'运动结果: {response.success}')
finally:
    if rclpy.ok():
        rclpy.shutdown()
```

### MoveJ
关节空间运动服务。

```python
from rm_ros_interfaces.srv import MoveJ

client = self.create_client(MoveJ, '/arm_a/move_j')
request = MoveJ.Request()
request.arm_id = "arm_a"
request.joint_positions = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
request.speed = 30.0  # 度/秒
request.blocking = True
request.timeout = 10.0

try:
    future = client.call_async(request)
    rclpy.spin_until_future_complete(self, future)
    response = future.result()
    self.get_logger().info(f'运动结果: {response.success}')
finally:
    if rclpy.ok():
        rclpy.shutdown()
```

### SetGripper
夹爪控制服务。

```python
from rm_ros_interfaces.srv import SetGripper

client = self.create_client(SetGripper, '/set_gripper')
request = SetGripper.Request()
request.gripper_id = "left"
request.command = "close"  # "open", "close", "pick", "set_position"
request.speed = 50.0
request.force = 100.0  # 用于pick命令
request.blocking = True
request.timeout = 5.0

try:
    future = client.call_async(request)
    rclpy.spin_until_future_complete(self, future)
    response = future.result()
    self.get_logger().info(f'夹爪控制结果: {response.success}')
finally:
    if rclpy.ok():
        rclpy.shutdown()
```

## 动作类型 (action)

### GripperAction
夹爪动作，支持实时反馈。

```python
from rm_ros_interfaces.action import GripperAction
from rclpy.action import ActionClient

# 创建动作客户端
action_client = ActionClient(self, GripperAction, '/gripper_action')

# 发送目标
goal_msg = GripperAction.Goal()
goal_msg.gripper_id = "left"
goal_msg.command = "pick"
goal_msg.speed = 50.0
goal_msg.force = 100.0

try:
    action_client.wait_for_server()
    send_goal_future = action_client.send_goal_async(
        goal_msg,
        feedback_callback=self.feedback_callback
    )
    rclpy.spin_until_future_complete(self, send_goal_future)

    goal_handle = send_goal_future.result()
    if goal_handle.accepted:
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(f'夹爪动作完成: {result.success}')
finally:
    if rclpy.ok():
        rclpy.shutdown()

def feedback_callback(self, feedback_msg):
    feedback = feedback_msg.feedback
    self.get_logger().info(f'当前位置: {feedback.current_position}')
```

### MoveToJoint
关节空间运动动作。

```python
from rm_ros_interfaces.action import MoveToJoint
from rclpy.action import ActionClient

action_client = ActionClient(self, MoveToJoint, '/move_to_joint')

goal_msg = MoveToJoint.Goal()
goal_msg.arm_id = "arm_a"
goal_msg.target_positions = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
goal_msg.speed = 30.0
goal_msg.enable_smooth = True

try:
    action_client.wait_for_server()
    send_goal_future = action_client.send_goal_async(
        goal_msg,
        feedback_callback=self.joint_feedback_callback
    )
    rclpy.spin_until_future_complete(self, send_goal_future)

    goal_handle = send_goal_future.result()
    if goal_handle.accepted:
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(f'运动完成，耗时: {result.execution_time}秒')
finally:
    if rclpy.ok():
        rclpy.shutdown()

def joint_feedback_callback(self, feedback_msg):
    feedback = feedback_msg.feedback
    self.get_logger().info(f'进度: {feedback.progress * 100:.1f}%')
```

### MoveToCartesian
笛卡尔空间运动动作。

```python
from rm_ros_interfaces.action import MoveToCartesian
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose

action_client = ActionClient(self, MoveToCartesian, '/move_to_cartesian')

goal_msg = MoveToCartesian.Goal()
goal_msg.arm_id = "arm_a"
goal_msg.target_pose = Pose()
goal_msg.target_pose.position.x = 0.3
goal_msg.target_pose.position.y = 0.0
goal_msg.target_pose.position.z = 0.5
goal_msg.speed = 0.1
goal_msg.motion_type = "linear"  # 或 "joint"

try:
    action_client.wait_for_server()
    send_goal_future = action_client.send_goal_async(
        goal_msg,
        feedback_callback=self.cartesian_feedback_callback
    )
    rclpy.spin_until_future_complete(self, send_goal_future)

    goal_handle = send_goal_future.result()
    if goal_handle.accepted:
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(f'运动完成: {result.success}')
finally:
    if rclpy.ok():
        rclpy.shutdown()

def cartesian_feedback_callback(self, feedback_msg):
    feedback = feedback_msg.feedback
    self.get_logger().info(f'进度: {feedback.progress * 100:.1f}%')
```

## 最佳实践

### 1. 使用 try-finally 确保资源清理

```python
import rclpy
from rclpy.node import Node

def main():
    rclpy.init()
    node = Node('my_node')

    try:
        # 你的代码逻辑
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
```

### 2. 异步服务调用

```python
def call_service_async(self, request):
    try:
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    except Exception as e:
        self.get_logger().error(f'服务调用失败: {e}')
        return None
    finally:
        if rclpy.ok():
            rclpy.shutdown()
```

### 3. 动作取消处理

```python
def cancel_action(self, goal_handle):
    try:
        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future)
        self.get_logger().info('动作已取消')
    finally:
        if rclpy.ok():
            rclpy.shutdown()
```

## 许可证

Apache-2.0

## 维护者

请在 package.xml 中更新维护者信息。
