cmake_minimum_required(VERSION 3.12)
project(MyProject)
set(CMAKE_CXX_STANDARD 17)                       # specify C++ standard
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra")

list(APPEND CMAKE_PREFIX_PATH "/thirdparty/Easy3D/build")

find_package(Easy3D COMPONENTS viewer fileio renderer core util REQUIRED)  # request Easy3D (recommended to request only needed components)
find_package(assimp REQUIRED)

include_directories("./include")
include_directories("/thirdparty/eigen")

add_library(MeshUtils include/MeshUtils.hpp src/MeshUtils.cpp) # create MeshUtils library
set_target_properties(MeshUtils PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(MeshUtils gmsh assimp) # link MeshUtils with CGAL

set(XPBD_FILES 
    "include/Simulation.hpp" 
    "include/BeamSimulation.hpp"
    "include/ResidualSimulation.hpp"
    "include/TissueGraspingSimulation.hpp"
    "include/ElasticMaterial.hpp"
    "include/MeshObject.hpp" 
    "include/RigidMeshObject.hpp"
    "include/ElasticMeshObject.hpp"
    "include/XPBDMeshObject.hpp"
    "include/TextRenderingViewer.hpp" 
    "include/config/Config.hpp"
    "include/config/MeshObjectConfig.hpp"
    "include/config/ElasticMeshObjectConfig.hpp"
    "include/config/XPBDMeshObjectConfig.hpp"
    "include/config/ElasticMaterialConfig.hpp"
    "include/config/RigidMeshObjectConfig.hpp"
    "include/config/SimulationConfig.hpp"
    "src/Simulation.cpp" 
    "src/BeamSimulation.cpp"
    "src/ResidualSimulation.cpp"
    "src/TissueGraspingSimulation.cpp"
    "src/MeshObject.cpp" 
    "src/RigidMeshObject.cpp"
    "src/ElasticMeshObject.cpp"
    "src/TextRenderingViewer.cpp" 
    "src/XPBDMeshObject.cpp"
)

add_library(XPBD ${XPBD_FILES} ) # create XPBD library
target_link_libraries(XPBD easy3d::core easy3d::util easy3d::renderer easy3d::viewer MeshUtils yaml-cpp)
add_executable(Test src/main.cpp)                    # create an executable target
target_link_libraries(Test XPBD easy3d::viewer gmsh)       # link to necessary Easy3D modules (add more if needed, e.g., algo)

add_executable(BeamTest src/beam_test_main.cpp)
target_link_libraries(BeamTest XPBD easy3d::viewer gmsh)

add_executable(TissueGraspingTest src/tissue_grasping_test_main.cpp)
target_link_libraries(TissueGraspingTest XPBD easy3d::viewer gmsh)