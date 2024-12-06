cmake_minimum_required(VERSION 3.13)
set(CMAKE_CUDA_ARCHITECTURES "native")
set(CMAKE_CUDA_COMPILER "/usr/local/cuda/bin/nvcc")
project(MyProject LANGUAGES CXX CUDA)
set(CMAKE_CXX_STANDARD 17)                       # specify C++ standard
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra")


list(APPEND CMAKE_PREFIX_PATH "/thirdparty/Easy3D/build")

find_package(Easy3D COMPONENTS viewer fileio renderer core util REQUIRED)  # request Easy3D (recommended to request only needed components)
find_package(assimp REQUIRED)   # find assimp (for mesh loading/conversion)

include_directories("./include")
include_directories("/thirdparty/eigen")

add_library(MeshUtils include/utils/MeshUtils.hpp src/utils/MeshUtils.cpp) # create MeshUtils library
set_target_properties(MeshUtils PROPERTIES LINKER_LANGUAGE CXX) # make sure the linker knows it's C++
target_link_libraries(MeshUtils gmsh assimp) # link MeshUtils with gmsh (for volume mesh generation) and assimp (for mesh loading/conversion)

# Collision library
add_library(Collision)
add_subdirectory("src/collision")

# Graphics library
add_library(Graphics)
add_subdirectory("src/graphics")

# Haptics library
add_library(Haptics)
add_subdirectory("src/haptics")


# Solver library
add_library(Solver)
add_subdirectory("src/solver")

# SimObject library
add_library(SimObject)
add_subdirectory("src/simobject")

# link in necessary libraries
target_link_libraries(Graphics easy3d::core easy3d::util easy3d::renderer easy3d::viewer) # link Easy3D for Graphics library
target_link_libraries(Haptics HD HDU)   # link OpenHaptics API libraries for Haptics library
target_link_libraries(SimObject MeshUtils Solver)   # link MeshUtils and Solver libraries
target_link_libraries(Solver SimObject)         # link SimObject library

### Simulation libraries - each has its own executable
add_library(ResidualSimulation)
add_library(BeamSimulation)
add_library(BeamStretchSimulation)
add_library(TissueGraspingSimulation)
add_library(InitialDeformationSimulation)
add_subdirectory("src/simulation")

target_link_libraries(ResidualSimulation Collision Graphics SimObject yaml-cpp)
target_link_libraries(BeamSimulation ResidualSimulation)
target_link_libraries(BeamStretchSimulation ResidualSimulation)
target_link_libraries(TissueGraspingSimulation Haptics ResidualSimulation)
target_link_libraries(InitialDeformationSimulation ResidualSimulation)

### Create executables
add_executable(Test src/exec/main.cpp)
target_link_libraries(Test ResidualSimulation)

add_executable(BeamTest src/exec/beam_test_main.cpp)
target_link_libraries(BeamTest BeamSimulation)

add_executable(TissueGraspingTest src/exec/tissue_grasping_test_main.cpp)
target_link_libraries(TissueGraspingTest TissueGraspingSimulation)

add_executable(BeamStretchTest src/exec/beam_stretch_test_main.cpp)
target_link_libraries(BeamStretchTest BeamStretchSimulation)

# add_executable(InitialDeformationTest src/exec/initial_deformation_test_main.cpp)
# target_link_libraries(InitialDeformationTest XPBD easy3d::viewer gmsh)

# add_executable(StaticLoadingTest src/exec/static_loading_test_main.cpp)
# target_link_libraries(StaticLoadingTest XPBD easy3d::viewer gmsh)

add_executable(HapticTest src/exec/haptic_main.cpp)
target_link_libraries(HapticTest XPBD easy3d::viewer gmsh)

add_executable(CudaTest src/exec/cuda_test.cu)
