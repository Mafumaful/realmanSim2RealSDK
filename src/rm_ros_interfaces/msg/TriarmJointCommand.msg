# 三臂机器人关节指令消息
# 包含19个关节的目标位置

std_msgs/Header header

# 关节名称列表
string[] joint_names

# 关节目标位置 (单位: 弧度)
float64[] positions

# 可选: 关节速度 (单位: 弧度/秒)
float64[] velocities

# 可选: 关节加速度 (单位: 弧度/秒²)
float64[] accelerations
