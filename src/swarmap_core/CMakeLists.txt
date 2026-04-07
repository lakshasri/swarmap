cmake_minimum_required(VERSION 3.8)
project(swarmap_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(swarmap_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(DEPS
  rclcpp
  rclcpp_lifecycle
  swarmap_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  lifecycle_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ── robot_node ────────────────────────────────────────────────────────────────
add_executable(robot_node
  src/robot_node.cpp
  src/occupancy_grid.cpp
  src/frontier_explorer.cpp
  src/map_merger.cpp
  src/neighbour_tracker.cpp
)
ament_target_dependencies(robot_node ${DEPS})
target_include_directories(robot_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS robot_node
  DESTINATION lib/${PROJECT_NAME}
)

# ── map_aggregator_node ───────────────────────────────────────────────────────
add_executable(map_aggregator_node src/map_aggregator_node.cpp)
ament_target_dependencies(map_aggregator_node ${DEPS})
target_include_directories(map_aggregator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
install(TARGETS map_aggregator_node DESTINATION lib/${PROJECT_NAME})

# ── failure_injector_node ─────────────────────────────────────────────────────
add_executable(failure_injector_node src/failure_injector_node.cpp)
ament_target_dependencies(failure_injector_node ${DEPS})
target_include_directories(failure_injector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
install(TARGETS failure_injector_node DESTINATION lib/${PROJECT_NAME})

# ── Python nodes (swarm_monitor, param_service) ───────────────────────────────
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  src/swarm_monitor_node.py
  src/param_service_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── unit tests ────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_grid     test/test_grid.cpp
    src/occupancy_grid.cpp src/frontier_explorer.cpp src/map_merger.cpp src/neighbour_tracker.cpp)
  ament_add_gtest(test_frontier test/test_frontier.cpp
    src/occupancy_grid.cpp src/frontier_explorer.cpp)
  ament_add_gtest(test_merger   test/test_merger.cpp
    src/occupancy_grid.cpp src/map_merger.cpp)
  ament_add_gtest(test_auction  test/test_auction.cpp
    src/occupancy_grid.cpp src/frontier_explorer.cpp)

  foreach(t test_grid test_frontier test_merger test_auction)
    ament_target_dependencies(${t} ${DEPS})
    target_include_directories(${t} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
  endforeach()
endif()

ament_package()
