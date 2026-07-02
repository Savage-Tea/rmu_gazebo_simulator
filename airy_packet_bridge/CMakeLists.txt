cmake_minimum_required(VERSION 3.10)
project(airy_packet_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(ignition-transport11 REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/airy_packet_bridge.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN airy_packet_bridge::AiryPacketBridge
  EXECUTABLE airy_packet_bridge_node
)

target_link_libraries(${PROJECT_NAME}
  ignition-transport11::ignition-transport11
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
