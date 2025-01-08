cmake_minimum_required(VERSION 3.5)
project(franka_ros2_csc379)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(Franka 0.9.0 QUIET)
if(NOT Franka_FOUND)
  find_package(Franka 0.8.0 REQUIRED)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# include_directories(include)

add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs)

add_executable(franka_state_publisher_ros2 src/franka_state_publisher_ros2.cpp)
target_include_directories(franka_state_publisher_ros2 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${Franka_INCLUDE_DIRS})
ament_target_dependencies(franka_state_publisher_ros2 rclcpp sensor_msgs Franka)


add_executable(run_franka_impedance_control 
    src/run_franka_impedance_control.cpp
    src/franka_impedance_control.cpp)
target_include_directories(run_franka_impedance_control PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${Franka_INCLUDE_DIRS})
ament_target_dependencies(run_franka_impedance_control rclcpp Franka) # why is rclcpp needed here?

add_executable(run_franka_impedance_control_ros2 
    src/run_franka_impedance_control_ros2.cpp
    src/franka_impedance_control_ros2.cpp
    src/franka_impedance_control.cpp)
target_include_directories(run_franka_impedance_control_ros2 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${Franka_INCLUDE_DIRS})
ament_target_dependencies(run_franka_impedance_control_ros2 rclcpp sensor_msgs Franka)

install(TARGETS
  talker
  subscriber
  franka_state_publisher_ros2
  run_franka_impedance_control
  run_franka_impedance_control_ros2
  DESTINATION lib/${PROJECT_NAME})

ament_package()
