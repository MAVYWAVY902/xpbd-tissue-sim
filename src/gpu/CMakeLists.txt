cmake_minimum_required(VERSION 3.13)
project(MyProject)

set(GPU_INCLUDE_DIR "../../include/gpu")
set(GPU_SRC_DIR "./")

target_sources(GPU
    PRIVATE
        ${GPU_INCLUDE_DIR}/GPUResource.hpp
        ${GPU_INCLUDE_DIR}/MeshGPUResource.hpp
        ${GPU_INCLUDE_DIR}/GPUResourceManager.hpp
        ${GPU_SRC_DIR}/GPUResourceManager.cpp
)