cmake_minimum_required(VERSION 3.8)
project(module_4_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(mazeSolving src/maze_solving_with_lidar.cpp)
ament_target_dependencies(mazeSolving rclcpp sensor_msgs geometry_msgs)

add_executable(squareSolving src/square_solving_with_lidar.cpp)
ament_target_dependencies(squareSolving rclcpp sensor_msgs geometry_msgs)

add_executable(rangeLidarTb3 src/range_lidar_tb3.cpp)
ament_target_dependencies(rangeLidarTb3 rclcpp sensor_msgs geometry_msgs)


add_executable(showvelTb3 src/tb3_show_vel_ace.cpp)
ament_target_dependencies(showvelTb3 rclcpp sensor_msgs)

add_executable(lineFollowing src/line_following_with_camera.cpp)
ament_target_dependencies(lineFollowing rclcpp sensor_msgs cv_bridge geometry_msgs)
target_link_libraries(lineFollowing ${OpenCV_LIBS})


install(TARGETS mazeSolving lineFollowing rangeLidarTb3 squareSolving showvelTb3
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch worlds urdf
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
