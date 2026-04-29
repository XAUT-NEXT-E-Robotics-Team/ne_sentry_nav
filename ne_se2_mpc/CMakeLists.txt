cmake_minimum_required(VERSION 3.8)
project(ne_se2_mpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

pkg_check_modules(QPOASES_PC QUIET qpoases)

find_path(QPOASES_INCLUDE_DIR
  NAMES qpOASES.hpp
  HINTS ${QPOASES_PC_INCLUDE_DIRS}
  PATH_SUFFIXES qpOASES qpoases
)

find_library(QPOASES_LIBRARY
  NAMES qpOASES qpoases
  HINTS ${QPOASES_PC_LIBRARY_DIRS}
)

if(NOT QPOASES_INCLUDE_DIR OR NOT QPOASES_LIBRARY)
  message(FATAL_ERROR "qpOASES not found. Please install qpOASES and make sure qpOASES.hpp and the library are discoverable.")
endif()

add_library(${PROJECT_NAME}
  src/ne_mpc_controller.cpp
)

add_executable(occupancy_grid_sim_node
  src/occupancy_grid_sim_node.cpp
)

add_executable(mpc_tracking_sim_node
  src/mpc_tracking_sim_node.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../ne_global_planner/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../ne_esdf_builder/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${Eigen3_INCLUDE_DIRS}
  ${QPOASES_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${QPOASES_LIBRARY})
ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp nav_msgs nav_interfaces)

target_include_directories(occupancy_grid_sim_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(mpc_tracking_sim_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../ne_global_planner/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${Eigen3_INCLUDE_DIRS}
)

target_link_libraries(mpc_tracking_sim_node ${PROJECT_NAME})

ament_target_dependencies(occupancy_grid_sim_node
  rclcpp
  nav_msgs
)

ament_target_dependencies(mpc_tracking_sim_node
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_ros
  visualization_msgs
)

install(
  TARGETS ${PROJECT_NAME} occupancy_grid_sim_node mpc_tracking_sim_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 rclcpp nav_msgs nav_interfaces)

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
