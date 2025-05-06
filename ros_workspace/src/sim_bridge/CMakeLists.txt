cmake_minimum_required(VERSION 3.8)
project(sim_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(sim_bridge src/SimBridge.cpp src/sim_bridge.cpp)
ament_target_dependencies(sim_bridge rclcpp std_msgs)
target_include_directories(sim_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(sim_bridge PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS sim_bridge
  DESTINATION lib/${PROJECT_NAME})

ament_package()
