cmake_minimum_required(VERSION 3.5)
project(ros2_cpp_py)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  std_msgs
)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(pub_test src/pub_test.cpp)
target_link_libraries(pub_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(sub_test src/sub_test.cpp)
target_link_libraries(sub_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
