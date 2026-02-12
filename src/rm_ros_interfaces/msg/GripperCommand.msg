# 夹爪控制指令消息

std_msgs/Header header

# 夹爪标识: "left" 或 "right"
string gripper_id

# 控制模式: "position" 或 "force"
string control_mode

# 目标位置 (单位: 度, 用于position模式)
float64 target_position

# 目标力 (单位: N, 用于force模式)
float64 target_force

# 速度 (单位: 度/秒)
float64 speed

# 是否阻塞等待完成
bool blocking

# 超时时间 (单位: 秒)
float64 timeout
