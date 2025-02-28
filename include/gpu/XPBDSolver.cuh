#ifndef __XPBD_SOLVER_CUH
#define __XPBD_SOLVER_CUH

#include "gpu/GPUStructs.hpp"

#include "gpu/GPUResource.hpp"
#include "gpu/WritableArrayGPUResource.hpp"
#include "gpu/MeshGPUResource.hpp"

__host__ __device__ void Mat3MulTranspose(const float* A, const float* B, float* C);

__host__ __device__ void Mat3Mul(const float* A, const float* B, float* C);

__host__ __device__ void Vec3Cross(const float* v1, const float* v2, float* v3);

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

#endif