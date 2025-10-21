cmake_minimum_required(VERSION 3.8)
project(mecabot_bt_exp2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(bt_exp2 src/main.cpp)
ament_target_dependencies(bt_exp2 rclcpp behaviortree_cpp_v3 sensor_msgs geometry_msgs std_msgs)

install(TARGETS
  bt_exp2
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY trees/
  DESTINATION share/${PROJECT_NAME}/trees)

ament_package()

