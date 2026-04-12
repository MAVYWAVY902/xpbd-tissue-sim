# ============================================================================
# XPBD Physics - Host build (no Graphics/Easy3D/VTK)
# Includes real Simulation.cpp (not stubs)
# ============================================================================
cmake_minimum_required(VERSION 3.13)
project(XPBDPhysicsHost LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -Wextra")
set(CMAKE_BUILD_TYPE Release)

# ---- Dependencies ----
find_package(Eigen3 REQUIRED)
find_package(embree 4 REQUIRED)
find_package(OpenMP)
find_package(assimp REQUIRED)
find_package(yaml-cpp REQUIRED)

find_library(GMSH_LIB gmsh)
find_library(MESH2SDF_LIB mesh2sdf)

# ---- Source directory ----
set(XPBD_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")

include_directories("${XPBD_ROOT}/include")

# ---- Source files (everything except Graphics) ----

# Solver
file(GLOB SOLVER_SRC "${XPBD_ROOT}/src/solver/constraint/*.cpp"
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
file(GLOB GEOMETRY_SRC "${XPBD_ROOT}/src/geometry/*.cpp"
                       "${XPBD_ROOT}/src/geometry/embree/*.cpp")

# Collision
set(COLLISION_SRC ${XPBD_ROOT}/src/collision/CollisionScene.cpp)

# Simulation (real, not stub — GraphicsScene is forward-declared in header)
set(SIMULATION_SRC
    ${XPBD_ROOT}/src/simulation/Simulation.cpp
    ${XPBD_ROOT}/src/simulation/SimulationStateRecorder.cpp
    ${XPBD_ROOT}/src/simulation/SimulationLogger.cpp
)

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
    ${SIMULATION_SRC}
    ${CONFIG_SRC}
    ${UTILS_SRC}
)

target_include_directories(xpbd_physics PUBLIC "${XPBD_ROOT}/include")
# NO_GRAPHICS: skip Graphics-related code in Simulation.cpp
target_compile_definitions(xpbd_physics PRIVATE NO_GRAPHICS)

target_link_libraries(xpbd_physics PUBLIC Eigen3::Eigen embree)
target_link_libraries(xpbd_physics PRIVATE assimp yaml-cpp)

if(GMSH_LIB)
    target_link_libraries(xpbd_physics PRIVATE ${GMSH_LIB})
endif()
if(MESH2SDF_LIB)
    target_link_libraries(xpbd_physics PRIVATE ${MESH2SDF_LIB})
endif()
if(OpenMP_CXX_FOUND)
    target_link_libraries(xpbd_physics PUBLIC OpenMP::OpenMP_CXX)
    target_compile_definitions(xpbd_physics PUBLIC ENABLE_JACOBI_OPENMP)
endif()

# ---- Install ----
install(TARGETS xpbd_physics ARCHIVE DESTINATION lib)
install(DIRECTORY "${XPBD_ROOT}/include/" DESTINATION include)
