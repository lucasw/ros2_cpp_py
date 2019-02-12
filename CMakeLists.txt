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
find_package(ament_index_cpp REQUIRED)
find_package(class_loader REQUIRED)
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

include_directories(include)

add_executable(cpp_test
  src/cpp_test.cpp
)
ament_target_dependencies(cpp_test
  "rclcpp"
  "rcutils"
)

add_executable(node_loader
  src/node_loader.cpp
)
ament_target_dependencies(node_loader
  "ament_index_cpp"
  "class_loader"
  "rclcpp"
  "rcutils"
)


add_library(ros2_cpp_nodes SHARED
  src/pub_test.cpp
  src/sub_test.cpp
)
ament_target_dependencies(ros2_cpp_nodes
  "class_loader"
  "cv_bridge"
  "rclcpp"
  "sensor_msgs"
)

add_executable(pub_sub_composed src/pub_sub_composed.cpp)
target_link_libraries(pub_sub_composed ros2_cpp_nodes)
ament_target_dependencies(pub_sub_composed rclcpp ros2_cpp_nodes)

add_executable(ros2_pub_test_standalone
  src/pub_test_standalone.cpp
)
target_link_libraries(ros2_pub_test_standalone ros2_cpp_nodes)
ament_target_dependencies(ros2_pub_test_standalone
  rclcpp
  ros2_cpp_nodes
)

add_executable(ros2_sub_test_standalone
  src/sub_test_standalone.cpp
)
target_link_libraries(ros2_sub_test_standalone ros2_cpp_nodes)
ament_target_dependencies(ros2_sub_test_standalone
  rclcpp
  ros2_cpp_nodes
)

add_executable(ros2_pub_sub_test
  src/pub_sub_test.cpp
)
target_link_libraries(ros2_pub_sub_test ros2_cpp_nodes)
ament_target_dependencies(ros2_pub_sub_test
  rclcpp
  ros2_cpp_nodes
)

install(TARGETS
  ros2_cpp_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install cpp executables
install(TARGETS
  cpp_test
  node_loader
  pub_sub_composed
  ros2_pub_test_standalone
  ros2_sub_test_standalone
  ros2_pub_sub_test
DESTINATION lib/${PROJECT_NAME}
)

# Install python scripts
install(PROGRAMS
  scripts/py_test.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
