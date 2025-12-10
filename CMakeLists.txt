cmake_minimum_required(VERSION 3.20)
project(kalman_positioning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary ROS2 packages and external libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(include)

# Main node executable definition
# We include the UKF and LandmarkManager source files here to ensure they are compiled and linked into the node
add_executable(positioning_node
  src/positioning_node.cpp
  src/ukf.cpp
  src/landmark_manager.cpp
)

# Specify include directories for the target to find header files
target_include_directories(positioning_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Eigen3_INCLUDE_DIRS}
)

# Link the executable against required libraries
target_link_libraries(positioning_node
  ${rclcpp_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

# Define dependencies to ensure correct build order
ament_target_dependencies(positioning_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# Install rules for the executable
install(TARGETS
  positioning_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install rules for header files
install(DIRECTORY
  include/
  DESTINATION include
)

# Install rules for launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install rules for the landmarks data file
install(FILES
  landmarks.csv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()