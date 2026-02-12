# 单个机械臂状态消息

std_msgs/Header header

# 机械臂标识: "arm_a", "arm_b", "arm_s"
string arm_id

# 当前关节位置 (单位: 弧度)
float64[] joint_positions

# 当前关节速度 (单位: 弧度/秒)
float64[] joint_velocities

# TCP位姿 (x, y, z, rx, ry, rz)
geometry_msgs/Pose tcp_pose

# 机械臂状态: "idle", "moving", "error"
string state

# 错误代码 (0表示无错误)
int32 error_code

# 错误描述
string error_message
