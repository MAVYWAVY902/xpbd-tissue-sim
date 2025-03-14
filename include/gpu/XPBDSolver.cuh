#ifndef __XPBD_SOLVER_CUH
#define __XPBD_SOLVER_CUH

#define BLOCK_SIZE 64

#include "gpu/GPUStructs.hpp"

#include "gpu/resource/GPUResource.hpp"
#include "gpu/resource/WritableArrayGPUResource.hpp"
#include "gpu/resource/MeshGPUResource.hpp"

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


template<class ConstraintProjector>
__global__ void ProjectConstraints(ConstraintProjector* projectors, int num_projectors, const float* vertices, float* new_vertices)
{
    int proj_index = blockIdx.x * blockDim.x + threadIdx.x;
    if (proj_index >= num_projectors)   return;

    // load ConstraintProjectors into shared mem using coalesced global memory accesses
    int proj_size_float = sizeof(ConstraintProjector) / sizeof(float);
    double proj_size_float_dbl = (1.0 * sizeof(ConstraintProjector)) / sizeof(float);
    int proj_global_start = proj_size_float * BLOCK_SIZE * blockIdx.x;

    __shared__ ConstraintProjector proj_smem[BLOCK_SIZE];

    float* proj_smem_float = reinterpret_cast<float*>(proj_smem);
    float* proj_gmem_float = reinterpret_cast<float*>(projectors + BLOCK_SIZE * blockIdx.x);
    int skip = min(num_projectors - blockIdx.x * BLOCK_SIZE, BLOCK_SIZE);
    for (int i = 0; i < sizeof(ConstraintProjector)/sizeof(float); i++)
    {
        proj_smem_float[threadIdx.x + skip*i] = proj_gmem_float[threadIdx.x + skip*i];
        // proj_smem_float[i] = proj_gmem_float[i];
    }
    __syncthreads();

    ConstraintProjector* bad_proj = proj_smem + threadIdx.x;
    
    bad_proj->initialize();
    bad_proj->project(vertices, new_vertices);
}

#endif