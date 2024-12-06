cmake_minimum_required(VERSION 3.13)
project(MyProject)

set(SIMOBJECT_INCLUDE_DIR "../../include/simobject")
set(SIMOBJECT_SRC_DIR "./")

target_sources(SimObject
    PRIVATE
        ${SIMOBJECT_INCLUDE_DIR}/MeshObject.hpp
        ${SIMOBJECT_INCLUDE_DIR}/ElasticMeshObject.hpp
        ${SIMOBJECT_INCLUDE_DIR}/XPBDMeshObject.hpp
        ${SIMOBJECT_INCLUDE_DIR}/FirstOrderXPBDMeshObject.hpp
        ${SIMOBJECT_INCLUDE_DIR}/RigidMeshObject.hpp
        ${SIMOBJECT_INCLUDE_DIR}/FastFEMMeshObject.hpp
        ${SIMOBJECT_INCLUDE_DIR}/ElasticMaterial.hpp
        ${SIMOBJECT_SRC_DIR}/MeshObject.cpp
        ${SIMOBJECT_SRC_DIR}/ElasticMeshObject.cpp
        ${SIMOBJECT_SRC_DIR}/XPBDMeshObject.cpp
        ${SIMOBJECT_SRC_DIR}/FirstOrderXPBDMeshObject.cpp
        ${SIMOBJECT_SRC_DIR}/RigidMeshObject.cpp
        ${SIMOBJECT_SRC_DIR}/FastFEMMeshObject.cpp
)