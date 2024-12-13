cmake_minimum_required(VERSION 3.12)
set(CMAKE_CUDA_ARCHITECTURES "native")
set(CMAKE_CUDA_COMPILER "/usr/local/cuda/bin/nvcc")
project(MyProject LANGUAGES CXX CUDA)
set(CMAKE_CXX_STANDARD 17)                       # specify C++ standard
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra")


list(APPEND CMAKE_PREFIX_PATH "/thirdparty/Easy3D/build")

find_package(Easy3D COMPONENTS viewer fileio renderer core util REQUIRED)  # request Easy3D (recommended to request only needed components)
find_package(assimp REQUIRED)

# add_subdirectory("/thirdparty/bvh" build)

include_directories("./include")
include_directories("./include/common")
include_directories("./include/collision")
include_directories("./include/haptics")
include_directories("./include/config")
include_directories("./include/graphics")
include_directories("./include/simobject")
include_directories("./include/simulation")
include_directories("./include/utils")
include_directories("/thirdparty/eigen")

add_library(MeshUtils include/utils/MeshUtils.hpp src/utils/MeshUtils.cpp) # create MeshUtils library
set_target_properties(MeshUtils PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(MeshUtils gmsh assimp) # link MeshUtils with CGAL

set(XPBD_FILES
    "include/common/colors.hpp"
    "include/collision/CollisionScene.hpp"
    "include/haptics/HapticDeviceManager.hpp"
    "include/simulation/Simulation.hpp" 
    "include/simulation/OutputSimulation.hpp"
    "include/simulation/BeamSimulation.hpp"
    "include/simulation/ResidualSimulation.hpp"
    "include/simulation/TissueGraspingSimulation.hpp"
    "include/simulation/BeamStretchSimulation.hpp"
    "include/simulation/InitialDeformationSimulation.hpp"
    "include/simulation/StaticLoadingSimulation.hpp"
    "include/simobject/ElasticMaterial.hpp"
    "include/simobject/MeshObject.hpp" 
    "include/simobject/RigidMeshObject.hpp"
    "include/simobject/ElasticMeshObject.hpp"
    "include/simobject/XPBDMeshObject.hpp"
    "include/simobject/FirstOrderXPBDMeshObject.hpp"
    "include/simobject/FastFEMMeshObject.hpp"
    "include/graphics/TextRenderingViewer.hpp" 
    "include/config/Config.hpp"
    "include/config/MeshObjectConfig.hpp"
    "include/config/ElasticMeshObjectConfig.hpp"
    "include/config/XPBDMeshObjectConfig.hpp"
    "include/config/FirstOrderXPBDMeshObjectConfig.hpp"
    "include/config/FastFEMMeshObjectConfig.hpp"
    "include/config/ElasticMaterialConfig.hpp"
    "include/config/RigidMeshObjectConfig.hpp"
    "include/config/SimulationConfig.hpp"
    "include/config/OutputSimulationConfig.hpp"
    "include/config/BeamStretchSimulationConfig.hpp"
    "include/config/TissueGraspingSimulationConfig.hpp"
    "include/config/InitialDeformationSimulationConfig.hpp"
    "include/config/StaticLoadingSimulationConfig.hpp"
    "src/collision/CollisionScene.cpp"
    "src/haptics/HapticDeviceManager.cpp"
    "src/simulation/Simulation.cpp"
    "src/simulation/OutputSimulation.cpp" 
    "src/simulation/BeamSimulation.cpp"
    "src/simulation/ResidualSimulation.cpp"
    "src/simulation/TissueGraspingSimulation.cpp"
    "src/simulation/BeamStretchSimulation.cpp"
    "src/simulation/InitialDeformationSimulation.cpp"
    "src/simulation/StaticLoadingSimulation.cpp"
    "src/simobject/MeshObject.cpp" 
    "src/simobject/RigidMeshObject.cpp"
    "src/simobject/ElasticMeshObject.cpp"
    "src/simobject/XPBDMeshObject.cpp"
    "src/simobject/FirstOrderXPBDMeshObject.cpp"
    "src/simobject/FastFEMMeshObject.cpp"
    "src/graphics/TextRenderingViewer.cpp" 
)

add_library(XPBD ${XPBD_FILES} ) # create XPBD library
target_link_libraries(XPBD easy3d::core easy3d::util easy3d::renderer easy3d::viewer MeshUtils yaml-cpp HD HDU)
add_executable(Test src/exec/main.cpp)                    # create an executable target
target_link_libraries(Test XPBD easy3d::viewer gmsh)       # link to necessary Easy3D modules (add more if needed, e.g., algo)

add_executable(BeamTest src/exec/beam_test_main.cpp)
target_link_libraries(BeamTest XPBD easy3d::viewer gmsh)

add_executable(TissueGraspingTest src/exec/tissue_grasping_test_main.cpp)
target_link_libraries(TissueGraspingTest XPBD easy3d::viewer gmsh)

add_executable(BeamStretchTest src/exec/beam_stretch_test_main.cpp)
target_link_libraries(BeamStretchTest XPBD easy3d::viewer gmsh)

add_executable(InitialDeformationTest src/exec/initial_deformation_test_main.cpp)
target_link_libraries(InitialDeformationTest XPBD easy3d::viewer gmsh)

add_executable(StaticLoadingTest src/exec/static_loading_test_main.cpp)
target_link_libraries(StaticLoadingTest XPBD easy3d::viewer gmsh)

add_executable(HapticTest src/exec/haptic_main.cpp)
target_link_libraries(HapticTest XPBD easy3d::viewer gmsh)

add_executable(CudaTest src/exec/cuda_test.cu)