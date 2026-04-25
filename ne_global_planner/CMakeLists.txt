cmake_minimum_required(VERSION 3.8)
project(ne_global_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ne_esdf_builder REQUIRED)
find_package(nav_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(ne_grid_search SHARED
  src/ne_grid_search.cpp
)

target_include_directories(ne_grid_search PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /home/ei/ne_nav/src/ne_esdf_builder/include
  ${EIGEN3_INCLUDE_DIR}
  ${Eigen3_INCLUDE_DIRS}
)

ament_target_dependencies(ne_grid_search
  ne_esdf_builder
  nav_interfaces
  nav_msgs
  rclcpp
)

add_executable(jps_planner_node
  src/ne_jps_planner.cpp
)

target_include_directories(jps_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /home/ei/ne_nav/src/ne_esdf_builder/include
  ${EIGEN3_INCLUDE_DIR}
  ${Eigen3_INCLUDE_DIRS}
)

target_link_libraries(jps_planner_node
  ne_grid_search
)

ament_target_dependencies(jps_planner_node
  geometry_msgs
  ne_esdf_builder
  nav_interfaces
  nav_msgs
  rclcpp
  visualization_msgs
)

install(
  TARGETS ne_grid_search jps_planner_node
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
