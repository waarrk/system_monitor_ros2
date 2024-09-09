cmake_minimum_required(VERSION 3.8)
project(system_monitor_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SystemMonitor.msg"
)

ament_export_dependencies(rosidl_default_runtime)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(system_monitor_node src/system_monitor_node.cpp)

ament_target_dependencies(system_monitor_node rclcpp std_msgs)
target_link_libraries(system_monitor_node ${cpp_typesupport_target})

install(TARGETS
  system_monitor_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
