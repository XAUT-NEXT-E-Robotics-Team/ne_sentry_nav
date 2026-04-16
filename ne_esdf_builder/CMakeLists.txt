cmake_minimum_required(VERSION 3.8)
project(ne_esdf_builder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(esdf_builder_node
  src/esdf_builder_node.cpp
)

add_executable(esdf_visualizer_node
  src/esdf_visualizer_node.cpp
)

target_include_directories(esdf_builder_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(esdf_visualizer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(esdf_builder_node
  rclcpp
  nav_msgs
  nav_interfaces
)

ament_target_dependencies(esdf_visualizer_node
  geometry_msgs
  rclcpp
  nav_interfaces
  visualization_msgs
)

install(
  TARGETS esdf_builder_node esdf_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
