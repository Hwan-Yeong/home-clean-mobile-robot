cmake_minimum_required(VERSION 3.8)
project(clean_robot_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(opennav_coverage_msgs REQUIRED)

include_directories(include)

# Add Executables
add_executable(map_to_polygon
  src/map_to_polygon/main.cpp
  src/map_to_polygon/map_to_polygon_node.cpp
)
ament_target_dependencies(map_to_polygon
  rclcpp
  cv_bridge
  OpenCV
  nav_msgs
  geometry_msgs
)

add_executable(coverage_client 
  src/coverage_client/main.cpp
  src/coverage_client/coverage_client_node.cpp
  src/coverage_client/coverage_path_manager.cpp
  src/coverage_client/robot_facade.cpp
  src/coverage_client/strategies/zigzag_strategy.cpp
)
ament_target_dependencies(coverage_client
  rclcpp
  cv_bridge
  OpenCV
  sensor_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  opennav_coverage_msgs
)

# Install
install(TARGETS
  map_to_polygon
  coverage_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION dreaming_include/${PROJECT_NAME} # Optional: if headers are needed by others
)

install(DIRECTORY
  launch
  config
  rviz
  maps
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
