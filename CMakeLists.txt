cmake_minimum_required(VERSION 3.13)

if(EXISTS /usr/local/cuda)
    set(HAVE_CUDA true)
else()
    set(HAVE_CUDA false)
endif()

if (HAVE_CUDA AND NOT NO_GPU)
    message("\n-- WE HAVE CUDA! -- \n")
    set(CMAKE_CUDA_ARCHITECTURES "native")
    set(CMAKE_CUDA_COMPILER "/usr/local/cuda/bin/nvcc")
    project(MyProject LANGUAGES CXX CUDA)
else ()
    project(MyProject LANGUAGES CXX)
endif()


set(CMAKE_CXX_STANDARD 17)                       # specify C++ standard
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra")
set(CMAKE_BUILD_TYPE Debug)

list(APPEND CMAKE_PREFIX_PATH "/thirdparty/Easy3D/build")

find_package(Easy3D COMPONENTS viewer fileio renderer core util REQUIRED)  # request Easy3D (recommended to request only needed components)
find_package(assimp REQUIRED)   # find assimp (for mesh loading/conversion)
find_package(Eigen3 3.3 REQUIRED)   # find Eigen

include_directories("./include")

if (HAVE_CUDA AND NOT NO_GPU)
    include_directories("/usr/local/cuda/include")
endif()

add_library(MeshUtils include/utils/MeshUtils.hpp src/utils/MeshUtils.cpp) # create MeshUtils library
set_target_properties(MeshUtils PROPERTIES LINKER_LANGUAGE CXX) # make sure the linker knows it's C++
target_link_libraries(MeshUtils gmsh assimp Eigen3::Eigen) # link MeshUtils with gmsh (for volume mesh generation) and assimp (for mesh loading/conversion)

add_library(GeometryUtils include/utils/GeometryUtils.hpp src/utils/GeometryUtils.cpp)
target_link_libraries(GeometryUtils Eigen3::Eigen)

# Collision library
add_library(Collision)
add_subdirectory("src/collision")

# Graphics library
add_library(Graphics)
add_subdirectory("src/graphics")

# Geometry library
add_library(Geometry)
add_subdirectory("src/geometry")

# GPU library
if (HAVE_CUDA AND NOT NO_GPU)
    add_library(GPU)
    add_subdirectory("src/gpu")
endif()

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
target_link_libraries(Collision Eigen3::Eigen mesh2sdf Geometry GeometryUtils)
target_link_libraries(Graphics Eigen3::Eigen easy3d::core easy3d::util easy3d::renderer easy3d::viewer) # link Easy3D for Graphics library
target_link_libraries(Geometry Eigen3::Eigen mesh2sdf SimObject GeometryUtils)
target_link_libraries(Haptics Eigen3::Eigen HD HDU)   # link OpenHaptics API libraries for Haptics library
target_link_libraries(SimObject Eigen3::Eigen MeshUtils GeometryUtils Geometry Solver)   # link MeshUtils and Solver libraries
target_link_libraries(Solver Eigen3::Eigen SimObject)         # link SimObject library

### Simulation libraries - each has its own executable
add_library(ResidualSimulation)
add_library(BeamSimulation)
# add_library(BeamStretchSimulation)
# add_library(TissueGraspingSimulation)
# add_library(InitialDeformationSimulation)
add_subdirectory("src/simulation")

target_link_libraries(ResidualSimulation Collision Graphics SimObject Solver yaml-cpp)
target_link_libraries(BeamSimulation ResidualSimulation)
# target_link_libraries(BeamStretchSimulation ResidualSimulation)
# target_link_libraries(TissueGraspingSimulation Haptics ResidualSimulation)
# target_link_libraries(InitialDeformationSimulation ResidualSimulation)

### Create executables
add_executable(Test src/exec/main.cpp)
target_link_libraries(Test ResidualSimulation)

add_executable(BeamTest src/exec/beam_test_main.cpp)
target_link_libraries(BeamTest BeamSimulation)

# add_executable(TissueGraspingTest src/exec/tissue_grasping_test_main.cpp)
# target_link_libraries(TissueGraspingTest TissueGraspingSimulation)

# add_executable(BeamStretchTest src/exec/beam_stretch_test_main.cpp)
# target_link_libraries(BeamStretchTest BeamStretchSimulation)

# add_executable(InitialDeformationTest src/exec/initial_deformation_test_main.cpp)
# target_link_libraries(InitialDeformationTest XPBD easy3d::viewer gmsh)

# add_executable(StaticLoadingTest src/exec/static_loading_test_main.cpp)
# target_link_libraries(StaticLoadingTest XPBD easy3d::viewer gmsh)

add_executable(VariadicVectorContainerTest src/exec/test/variadic_vector_container_test.cpp)
target_link_libraries(VariadicVectorContainerTest Eigen3::Eigen)

if (HAVE_CUDA AND NOT NO_GPU)
    target_link_libraries(GPU Geometry SimObject Solver)
    target_link_libraries(SimObject GPU)
    target_link_libraries(Collision GPU)
    target_link_libraries(ResidualSimulation GPU)
    # target_link_libraries(Solver GPU)

    target_compile_definitions(Collision PRIVATE HAVE_CUDA)
    target_compile_definitions(Graphics PRIVATE HAVE_CUDA)
    target_compile_definitions(Geometry PRIVATE HAVE_CUDA)
    target_compile_definitions(GeometryUtils PRIVATE HAVE_CUDA)
    target_compile_definitions(GPU PRIVATE HAVE_CUDA)
    target_compile_definitions(MeshUtils PRIVATE HAVE_CUDA)
    target_compile_definitions(Haptics PRIVATE HAVE_CUDA)
    target_compile_definitions(SimObject PRIVATE HAVE_CUDA)
    target_compile_definitions(Solver PRIVATE HAVE_CUDA)
    target_compile_definitions(ResidualSimulation PRIVATE HAVE_CUDA)
    target_compile_definitions(BeamSimulation PRIVATE HAVE_CUDA)

    # target_compile_options(GPU PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:
    #                    -G
    #                    -g
    #                    >)

    add_executable(CudaTest src/exec/cuda_test.cu)
    target_link_libraries(CudaTest Eigen3::Eigen Geometry SimObject)
    target_compile_definitions(CudaTest PRIVATE HAVE_CUDA)

    add_executable(MeshSDFCudaTest src/exec/mesh_sdf_cuda_test.cu)
    target_link_libraries(MeshSDFCudaTest Eigen3::Eigen Geometry SimObject)
    target_compile_definitions(MeshSDFCudaTest PRIVATE HAVE_CUDA)

    add_executable(XPBDCudaTest src/exec/xpbd_cuda_test.cu)
    target_link_libraries(XPBDCudaTest Eigen3::Eigen Geometry SimObject)
    target_compile_definitions(XPBDCudaTest PRIVATE HAVE_CUDA)
    # target_compile_options(XPBDCudaTest PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--keep>)

    # add_executable(XPBDConstraintCudaTest src/exec/constraints_cuda_test.cu)
    # target_link_libraries(XPBDConstraintCudaTest Eigen3::Eigen Geometry SimObject GPU ResidualSimulation)
    # target_compile_definitions(XPBDConstraintCudaTest PRIVATE HAVE_CUDA)
endif()


