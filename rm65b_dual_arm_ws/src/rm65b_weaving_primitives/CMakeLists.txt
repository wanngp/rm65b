cmake_minimum_required(VERSION 3.8)
project(rm65b_weaving_primitives)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_executable(weaving_bt_runner src/weaving_bt_runner.cpp)
target_compile_features(weaving_bt_runner PRIVATE cxx_std_17)
ament_target_dependencies(
  weaving_bt_runner
  ament_index_cpp
  behaviortree_cpp_v3
  control_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  trajectory_msgs
)
if(TARGET yaml-cpp)
  target_link_libraries(weaving_bt_runner yaml-cpp)
elseif(TARGET YAML::yaml-cpp)
  target_link_libraries(weaving_bt_runner YAML::yaml-cpp)
else()
  target_link_libraries(weaving_bt_runner yaml-cpp)
endif()

install(TARGETS weaving_bt_runner DESTINATION lib/${PROJECT_NAME})

install(
  PROGRAMS rm65b_weaving_primitives/weaving_coordinator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME weaving_coordinator
)
install(
  PROGRAMS rm65b_weaving_primitives/trajectory_recorder.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME trajectory_recorder
)
install(
  PROGRAMS rm65b_weaving_primitives/tension_simulator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tension_simulator
)

install(DIRECTORY config launch trajectories DESTINATION share/${PROJECT_NAME})

ament_package()
