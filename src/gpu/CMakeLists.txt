cmake_minimum_required(VERSION 3.13)
project(MyProject)

set(GPU_INCLUDE_DIR "../../include/gpu")
set(GPU_SRC_DIR "./")

target_sources(GPU
    PRIVATE
        ${GPU_SRC_DIR}/common/helper.cu
        ${GPU_SRC_DIR}/Collision.cu
        ${GPU_SRC_DIR}/XPBDSolver.cu
        ${GPU_SRC_DIR}/resource/XPBDMeshObjectGPUResource.cpp
        # ${GPU_SRC_DIR}/projector/GPUNeohookeanCombinedConstraintProjector.cu
)