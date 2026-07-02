cmake_minimum_required(VERSION 3.10)
project(airy_gazebo_plugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-rendering6 REQUIRED)
find_package(ignition-transport11 REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/airy_gpu_lidar.cpp
)

target_link_libraries(${PROJECT_NAME}
  ignition-gazebo6::ignition-gazebo6
  ignition-rendering6::ignition-rendering6
  ignition-transport11::ignition-transport11
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

ament_auto_package()
