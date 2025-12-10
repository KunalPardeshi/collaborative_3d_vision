cmake_minimum_required(VERSION 3.5)
project(collab_vision_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generate message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HumanKeypoint.msg"
  "msg/AdvancedObject.msg"
  "msg/AdvancedDetections.msg"
  DEPENDENCIES std_msgs
)

# Install msg directory (REQUIRED for ROS2 Humble)
install(
  DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
