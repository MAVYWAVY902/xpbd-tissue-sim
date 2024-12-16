cmake_minimum_required(VERSION 3.13)
project(MyProject)

set(GEOMETRY_INCLUDE_DIR "../../include/geometry")
set(GEOMETRY_SRC_DIR "./")

target_sources(Geometry
    PRIVATE
        ${GEOMETRY_INCLUDE_DIR}/AABB.hpp
        ${GEOMETRY_INCLUDE_DIR}/Mesh.hpp
        ${GEOMETRY_INCLUDE_DIR}/TetMesh.hpp
        ${GEOMETRY_SRC_DIR}/Mesh.cpp
        ${GEOMETRY_SRC_DIR}/TetMesh.cpp
)