cmake_minimum_required(VERSION 3.8)
project(kalman_filter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp         REQUIRED)
find_package(sensor_msgs    REQUIRED)
find_package(geometry_msgs  REQUIRED)
find_package(Eigen3         REQUIRED)

# ---------- library ----------
add_library(kalman_filter_lib
  src/KalmanFilter.cpp)

target_include_directories(kalman_filter_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(kalman_filter_lib Eigen3::Eigen)

ament_target_dependencies(kalman_filter_lib
  rclcpp sensor_msgs geometry_msgs)

# ---------- node ----------
add_executable(kalman_filter_node
  src/kalman_filter_node.cpp)

target_link_libraries(kalman_filter_node
  kalman_filter_lib
  rclcpp::rclcpp)

ament_target_dependencies(kalman_filter_node
  rclcpp sensor_msgs geometry_msgs)     
  
# ---------- install ----------
install(TARGETS
  kalman_filter_lib
  kalman_filter_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
