#ifndef __XPBD_SOLVER_CUH
#define __XPBD_SOLVER_CUH

#define BLOCK_SIZE 64

#include "gpu/GPUStructs.hpp"

#include "gpu/resource/GPUResource.hpp"
#include "gpu/resource/WritableArrayGPUResource.hpp"
#include "gpu/resource/MeshGPUResource.hpp"

#include "gpu/projector/GPUConstraintProjector.cuh"
#include "gpu/projector/GPUCombinedConstraintProjector.cuh"

__device__ void ElementJacobiSolve(int elem_index, const int* elements, int num_elements, const float* vertices,
        const float* masses, const float* volumes, const float* Qs,
        float lambda, float mu, float dt,
        float* new_vertices);

__global__ void XPBDJacobiSolve(const int* elements, int num_elements, const float* vertices,
        const float* masses, const float* volumes, const float* Qs,
        float lambda, float mu, float dt,
        float* new_vertices);

__global__ void CopyVertices(const float* src_vertices, float* dst_vertices, int num_vertices);

__host__ void LaunchXPBDJacobiSolve(const int* elements, int num_elements, const float* vertices,
        const float* masses, const float* volumes, const float* Qs,
        float lambda, float mu, float dt,
        float* new_vertices);

__host__ void LaunchCopyVertices(const float* src_vertices, float* dst_vertices, int num_vertices);

template<class ConstraintProjector>
__global__ void ProjectConstraints(ConstraintProjector* projectors, int num_projectors, const float* vertices, float* new_vertices);

template<class ConstraintProjector>
__host__ void LaunchProjectConstraints(ConstraintProjector* projectors, int num_projectors, const float* vertices, float* new_vertices);

__host__ void CopyVerticesMemcpy(float* dst_vertices, const float* src_vertices, int num_vertices);

#endif