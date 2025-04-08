cmake_minimum_required(VERSION 3.13)
project(MyProject)

set(SIMOBJECT_INCLUDE_DIR "../../include/simobject")
set(SIMOBJECT_SRC_DIR "./")

target_sources(SimObject
    PRIVATE
        ${SIMOBJECT_INCLUDE_DIR}/Object.hpp
        ${SIMOBJECT_INCLUDE_DIR}/RigidObject.hpp
        ${SIMOBJECT_INCLUDE_DIR}/RigidPrimitives.hpp
        ${SIMOBJECT_INCLUDE_DIR}/XPBDMeshObject.hpp
        ${SIMOBJECT_INCLUDE_DIR}/FirstOrderXPBDMeshObject.hpp
        ${SIMOBJECT_INCLUDE_DIR}/RigidMeshObject.hpp
        ${SIMOBJECT_INCLUDE_DIR}/ElasticMaterial.hpp
        ${SIMOBJECT_INCLUDE_DIR}/VirtuosoArm.hpp
        ${SIMOBJECT_INCLUDE_DIR}/VirtuosoRobot.hpp
        ${SIMOBJECT_SRC_DIR}/RigidObject.cpp
        ${SIMOBJECT_SRC_DIR}/RigidPrimitives.cpp
        ${SIMOBJECT_SRC_DIR}/XPBDMeshObject.cpp
        ${SIMOBJECT_SRC_DIR}/FirstOrderXPBDMeshObject.cpp
        ${SIMOBJECT_SRC_DIR}/RigidMeshObject.cpp
        ${SIMOBJECT_SRC_DIR}/VirtuosoArm.cpp
        ${SIMOBJECT_SRC_DIR}/VirtuosoRobot.cpp
)