cmake_minimum_required(VERSION 3.8)
project(oculus_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(oculus_bridge_node
  src/oculus_packet_decoder.cpp
  src/oculus_bridge_node.cpp
)
target_include_directories(oculus_bridge_node PRIVATE include)
target_compile_features(oculus_bridge_node PRIVATE cxx_std_17)
ament_target_dependencies(oculus_bridge_node rclcpp sensor_msgs std_msgs)

install(TARGETS
  oculus_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
