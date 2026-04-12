# ============================================================================
# XPBDPhysicsCore - Standalone physics library (no graphics, no haptics)
# ============================================================================
cmake_minimum_required(VERSION 3.13)
project(XPBDPhysicsCore LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra -march=native")
set(CMAKE_BUILD_TYPE Release)

# ---- Dependencies (math/geometry only) ----
find_package(Eigen3 REQUIRED)
find_package(embree 4 REQUIRED)
find_package(OpenMP)
find_package(assimp REQUIRED)

# gmsh: optional, needed for volume mesh generation from surface meshes
find_library(GMSH_LIB gmsh)
find_path(GMSH_INCLUDE gmsh.h PATH_SUFFIXES include)
if(GMSH_LIB)
    message(STATUS "Found gmsh: ${GMSH_LIB}")
else()
    message(WARNING "gmsh not found - mesh generation from OBJ/STL will not work")
endif()

# mesh2sdf
find_package(mesh2sdf QUIET)
if(NOT mesh2sdf_FOUND)
    find_library(MESH2SDF_LIB mesh2sdf)
    find_path(MESH2SDF_INCLUDE Mesh2SDF/MeshSDF.hpp PATHS /usr/local/include)
    if(MESH2SDF_LIB)
        message(STATUS "Found mesh2sdf: ${MESH2SDF_LIB}")
    else()
        message(FATAL_ERROR "mesh2sdf not found")
    endif()
endif()

# yaml-cpp (for Config)
find_package(yaml-cpp QUIET)
if(NOT yaml-cpp_FOUND)
    find_library(YAMLCPP_LIB yaml-cpp)
endif()

# ---- Source directory (parent of physics_core/) ----
set(XPBD_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")

# CRITICAL: stubs come BEFORE the real include/ so our minimal
# Simulation.hpp is found instead of the full one that pulls in Graphics
include_directories(BEFORE "${CMAKE_CURRENT_SOURCE_DIR}/stubs")
include_directories("${XPBD_ROOT}/include")

# ---- Gather all physics source files ----

# Solver
file(GLOB SOLVER_SRC    "${XPBD_ROOT}/src/solver/constraint/*.cpp"
                        "${XPBD_ROOT}/src/solver/xpbd_projector/*.cpp")

# SimObject
set(SIMOBJECT_SRC
    ${XPBD_ROOT}/src/simobject/RigidObject.cpp
    ${XPBD_ROOT}/src/simobject/RigidPrimitives.cpp
    ${XPBD_ROOT}/src/simobject/XPBDMeshObject.cpp
    ${XPBD_ROOT}/src/simobject/RigidMeshObject.cpp
    ${XPBD_ROOT}/src/simobject/VirtuosoArm.cpp
    ${XPBD_ROOT}/src/simobject/VirtuosoRobot.cpp
)

# Geometry
file(GLOB GEOMETRY_SRC  "${XPBD_ROOT}/src/geometry/*.cpp"
                        "${XPBD_ROOT}/src/geometry/embree/*.cpp")

# Collision
set(COLLISION_SRC ${XPBD_ROOT}/src/collision/CollisionScene.cpp)


# Config
file(GLOB CONFIG_SRC "${XPBD_ROOT}/src/config/simobject/*.cpp")

# Utils
set(UTILS_SRC
    ${XPBD_ROOT}/src/utils/MeshUtils.cpp
    ${XPBD_ROOT}/src/utils/MTLParser.cpp
    ${XPBD_ROOT}/src/utils/GeometryUtils.cpp
)

# ---- Build static library ----
add_library(xpbd_physics STATIC
    ${SOLVER_SRC}
    ${SIMOBJECT_SRC}
    ${GEOMETRY_SRC}
    ${COLLISION_SRC}
    ${CONFIG_SRC}
    ${UTILS_SRC}
    ${XPBD_ROOT}/src/simulation/SimulationStateRecorder.cpp
    ${XPBD_ROOT}/src/simulation/SimulationLogger.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/stubs/simulation/debug_sizeof.cpp
)

target_include_directories(xpbd_physics PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/stubs"
    "${XPBD_ROOT}/include"
)

target_link_libraries(xpbd_physics PUBLIC
    Eigen3::Eigen
    embree
)

if(mesh2sdf_FOUND)
    target_link_libraries(xpbd_physics PUBLIC mesh2sdf::mesh2sdf)
else()
    target_link_libraries(xpbd_physics PUBLIC ${MESH2SDF_LIB})
    target_include_directories(xpbd_physics PUBLIC ${MESH2SDF_INCLUDE})
endif()

if(GMSH_LIB)
    target_link_libraries(xpbd_physics PRIVATE ${GMSH_LIB})
    if(GMSH_INCLUDE)
        target_include_directories(xpbd_physics PRIVATE ${GMSH_INCLUDE})
    endif()
endif()

target_link_libraries(xpbd_physics PRIVATE assimp)

if(YAMLCPP_LIB)
    target_link_libraries(xpbd_physics PRIVATE ${YAMLCPP_LIB})
elseif(yaml-cpp_FOUND)
    target_link_libraries(xpbd_physics PRIVATE yaml-cpp)
endif()

if(OpenMP_CXX_FOUND)
    target_link_libraries(xpbd_physics PUBLIC OpenMP::OpenMP_CXX)
    target_compile_definitions(xpbd_physics PUBLIC ENABLE_JACOBI_OPENMP)
endif()

# ---- Install ----
install(TARGETS xpbd_physics ARCHIVE DESTINATION lib)
install(DIRECTORY "${XPBD_ROOT}/include/" DESTINATION include)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/stubs/" DESTINATION include)
