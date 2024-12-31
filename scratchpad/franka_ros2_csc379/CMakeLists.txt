cmake_minimum_required(VERSION 3.5)
project(franka_ros2_csc379)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# include_directories(include)

add_executable(talker src/publisher.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(franka_state_publisher src/franka_state_publisher.cpp)
target_include_directories(franka_state_publisher PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(franka_state_publisher rclcpp sensor_msgs)


install(TARGETS
  talker
  franka_state_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
