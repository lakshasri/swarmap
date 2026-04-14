cmake_minimum_required(VERSION 3.8)
project(swarmap_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config worlds urdf rviz models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_two_robots.py     TIMEOUT 180)
  add_launch_test(test/test_frontier_auction.py TIMEOUT 120)
  add_launch_test(test/test_map_merge.py      TIMEOUT 180)
  add_launch_test(test/test_failure_recovery.py TIMEOUT 240)
endif()

ament_package()
