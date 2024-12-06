cmake_minimum_required(VERSION 3.13)
project(MyProject)

set(COLLISION_INCLUDE_DIR "../../include/collision")
set(COLLISION_SRC_DIR "./")

target_sources(Collision
    PRIVATE
        ${COLLISION_INCLUDE_DIR}/CollisionScene.hpp
        ${COLLISION_SRC_DIR}/CollisionScene.cpp
)