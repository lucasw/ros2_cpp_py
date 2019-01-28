cmake_minimum_required(VERSION 3.5)

project(ros2_cpp_py)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosidl_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Test.msg"
  "srv/Test2.srv"
  # DEPENDENCIES std_msgs
)

add_executable(cpp_test
  src/cpp_test.cpp
)
ament_target_dependencies(cpp_test
  "rclcpp"
  "rcutils"
)

add_executable(ros2_pub_test
  src/pub_test.cpp
)
ament_target_dependencies(ros2_pub_test
  "cv_bridge"
  "rclcpp"
  "sensor_msgs"
)

add_executable(ros2_sub_test
  src/sub_test.cpp
)
ament_target_dependencies(ros2_sub_test
  "rclcpp"
  "sensor_msgs"
)

# Install cpp executables
install(TARGETS
  cpp_test
  ros2_pub_test
  ros2_sub_test
DESTINATION lib/${PROJECT_NAME}
)

# Install python scripts
install(PROGRAMS
  scripts/py_test.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
