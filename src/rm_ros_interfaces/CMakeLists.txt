cmake_minimum_required(VERSION 3.8)
project(rm_ros_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 定义消息、服务和动作文件
rosidl_generate_interfaces(${PROJECT_NAME}
  # 消息文件
  "msg/TriarmJointCommand.msg"
  "msg/GripperCommand.msg"
  "msg/ArmState.msg"
  "msg/VelocityMode.msg"
  "msg/MotionResult.msg"

  # 服务文件
  "srv/SetVelocityMode.srv"
  "srv/GetFK.srv"
  "srv/GetIK.srv"
  "srv/MoveL.srv"
  "srv/MoveJ.srv"
  "srv/SetGripper.srv"

  # 动作文件
  "action/GripperAction.action"
  "action/MoveToJoint.action"
  "action/MoveToCartesian.action"

  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
