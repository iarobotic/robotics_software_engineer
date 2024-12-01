cmake_minimum_required(VERSION 3.8)
project(module_2_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(pattern src/patter_node.cpp)
ament_target_dependencies(pattern rclcpp geometry_msgs)

add_executable(move_turtle src/move_turtle.cpp)
ament_target_dependencies(move_turtle rclcpp geometry_msgs)

install(TARGETS
  pattern move_turtle
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
