cmake_minimum_required(VERSION 3.5)
project(g1_nav2_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(navigation_odometry src/navigation_odometry.cpp)
target_compile_features(navigation_odometry PUBLIC cxx_std_14)
target_include_directories(navigation_odometry PRIVATE include)
target_link_libraries(navigation_odometry Eigen3::Eigen)
ament_target_dependencies(navigation_odometry rclcpp nav_msgs)
install(TARGETS navigation_odometry DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config rviz behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS tools/generate_nav2_params.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(dwb_plugins REQUIRED)
  ament_add_gtest(test_navigation_odometry test/test_navigation_odometry.cpp)
  target_include_directories(test_navigation_odometry PRIVATE include)
  target_link_libraries(test_navigation_odometry Eigen3::Eigen)
  ament_target_dependencies(test_navigation_odometry nav_msgs dwb_plugins)
  find_package(ament_cmake_test REQUIRED)
  find_package(g1_loco_bridge REQUIRED)
  get_filename_component(G1_BRIDGE_PREFIX "${g1_loco_bridge_DIR}/../../.." ABSOLUTE)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_test(test_navigation_odometry_node
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_navigation_odometry_node.py
      --node $<TARGET_FILE:navigation_odometry>
    ENV ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1
      ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_ros_logs
    TIMEOUT 45)
  ament_add_test(test_navigation_config
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_navigation_config.py
    TIMEOUT 30)
  ament_add_test(test_navigation_readiness
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_navigation_readiness.py
      --adapter $<TARGET_FILE:navigation_odometry>
      --bridge "${G1_BRIDGE_PREFIX}/lib/g1_loco_bridge/g1_cmd_vel_dry_run"
    TIMEOUT 90)
endif()

ament_package()
