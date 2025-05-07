cmake_minimum_required(VERSION 3.8)
project(sim_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# include_directories("/usr/local/include/XPBDSim")
include_directories("/workspace/include")

list(APPEND CMAKE_PREFIX_PATH "/thirdparty/Easy3D/build") # needed in order to find Easy3D

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED)   # find Eigen
find_package(Easy3D COMPONENTS viewer fileio renderer core util REQUIRED)  # request Easy3D (recommended to request only needed components)
find_package(assimp REQUIRED)   # find assimp (for mesh loading/conversion)

add_executable(sim_bridge src/SimBridge.cpp src/sim_bridge.cpp)
ament_target_dependencies(sim_bridge rclcpp std_msgs geometry_msgs shape_msgs sensor_msgs)
target_link_libraries(sim_bridge 
  ResidualSimulation
  VirtuosoSimulation
  Collision
  SimObject
  Solver
  Graphics
  Haptics
  Geometry
  GeometryUtils
  MeshUtils
  assimp
  gmsh
  Eigen3::Eigen
  yaml-cpp
  easy3d::core
  easy3d::util
  easy3d::renderer
  easy3d::viewer
  HD
  HDU
  mesh2sdf
)


target_include_directories(sim_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(sim_bridge PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS sim_bridge
  DESTINATION lib/${PROJECT_NAME})

ament_package()
