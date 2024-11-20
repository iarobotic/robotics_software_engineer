cmake_minimum_required(VERSION 3.10)
project(Robot)

include_directories(include)

add_executable(robot_r src/robot_main.cpp src/robot_class.cpp)
add_executable(robot_sensor src/robot_sensors.cpp src/robot_class.cpp)
add_executable(sensor_r src/sensor_main.cpp)


