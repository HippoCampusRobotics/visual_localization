# Guide to create mixed C++ and Python package
# https://answers.ros.org/question/298740/ros2-c-and-python-in-same-package-is-it-possible/

cmake_minimum_required(VERSION 3.5)
project(visual_localization)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CXX_FLAGS -Wall -Wextra -Wpedantic -Wshadow -std=c++17 -O2)
add_compile_options(${CXX_FLAGS})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_srvs REQUIRED)
find_package(hippo_msgs REQUIRED)
find_package(hippo_common REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

include_directories(include)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

################################################################################
# image_decoder
################################################################################
set(exec_name image_decoder)
add_executable(${exec_name}
  src/image_decoder.cpp
)
ament_target_dependencies(${exec_name}
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
)
install(TARGETS ${exec_name}
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# ranges_node
################################################################################
set(exec_name ranges)
add_executable(${exec_name}
  src/ranges.cpp
)
ament_target_dependencies(${exec_name}
  rclcpp
  rclcpp_components
  geometry_msgs
  hippo_msgs
  tf2_msgs
)
install(TARGETS ${exec_name}
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# px4_bridge_node
################################################################################
set(exec_name px4_bridge)
add_executable(${exec_name}
  src/px4_bridge.cpp
)
ament_target_dependencies(${exec_name}
rclcpp
rclcpp_components
geometry_msgs
hippo_common
nav_msgs
tf2_ros
tf2
tf2_geometry_msgs
px4_msgs
)
install(TARGETS ${exec_name}
  DESTINATION lib/${PROJECT_NAME}
)



# Install Python executables
install(PROGRAMS
  nodes/vision_ekf_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
