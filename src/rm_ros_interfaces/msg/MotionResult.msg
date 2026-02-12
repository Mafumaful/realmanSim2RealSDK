# 运动执行结果消息

std_msgs/Header header

# 机械臂标识
string arm_id

# 运动类型: "movej", "movel", "movej_p"
string motion_type

# 执行结果: true表示成功, false表示失败
bool success

# 执行时间 (单位: 秒)
float64 execution_time

# 结果描述
string message
