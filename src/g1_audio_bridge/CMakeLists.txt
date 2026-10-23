cmake_minimum_required(VERSION 3.8)
project(g1_audio_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(g1_task_interfaces REQUIRED)
find_package(unitree_api REQUIRED)
find_package(Threads REQUIRED)

set(UNITREE_ROS2_EXAMPLE_INCLUDE
  "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/unitree_ros2_example_include"
  CACHE PATH "Unitree ROS2 AudioClient headers (no native SDK library required)")
if(NOT EXISTS "${UNITREE_ROS2_EXAMPLE_INCLUDE}/g1/g1_audio_client.hpp")
  message(FATAL_ERROR "Bundled Unitree AudioClient headers are missing")
endif()

add_executable(g1_audio_bridge src/g1_audio_bridge.cpp)
target_include_directories(g1_audio_bridge SYSTEM PRIVATE
  ${UNITREE_ROS2_EXAMPLE_INCLUDE}
  ${UNITREE_ROS2_EXAMPLE_INCLUDE}/common
  ${UNITREE_ROS2_EXAMPLE_INCLUDE}/nlohmann)
ament_target_dependencies(g1_audio_bridge
  rclcpp rclcpp_action rcl_interfaces g1_task_interfaces unitree_api)
target_link_libraries(g1_audio_bridge Threads::Threads)

install(TARGETS g1_audio_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_test(audio_bridge
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_audio_bridge.py
      --bridge $<TARGET_FILE:g1_audio_bridge>
    TIMEOUT 100)
endif()

ament_package()
