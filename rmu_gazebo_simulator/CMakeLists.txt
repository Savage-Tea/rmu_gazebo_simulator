cmake_minimum_required(VERSION 3.8)
project(rmu_gazebo_simulator)

find_package(ament_cmake_auto REQUIRED)
find_package(ignition-gazebo6 REQUIRED)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo.dsv.in")

# Airy Link Follower plugin — keeps the Airy LiDAR sensor model attached to the robot
add_library(airy_link_follower SHARED src/airy_link_follower.cpp)
target_link_libraries(airy_link_follower
  ignition-gazebo6::ignition-gazebo6
)
install(TARGETS airy_link_follower
  LIBRARY DESTINATION lib
)

install(PROGRAMS
  scripts/referee_system/simple_competition_1v1.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  resource
  config
  launch
  rviz
)
