#ifndef __HELPER_CUH
#define __HELPER_CUH

#include "common/types.hpp"

// computes A * B^T and stores result in C
__host__ __device__ void Mat3MulTranspose(const float* A, const float* B, float* C);

// computes A * B and stores result in C
__host__ __device__ void Mat3Mul(const float* A, const float* B, float* C);

// computes cross product between 2 vectors
__host__ __device__ void Vec3Cross(const float* v1, const float* v2, float* v3);

// computes dot product between 2 3-vectors
__host__ __device__ float Vec3Dot(const float* v1, const float* v2);

// computes the rest state matrix Q and the volume for a tetrahedral element defined by (v0, v1, v2, v3)
__host__ static void computeQandVolume(const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, const Eigen::Vector3f& v3, float* Q, float* rest_volume);

// computes the deformation gradient F given the current nodal positions x and the rest state matrix Q
__device__ static void computeF(const float* x, const float* Q, float* F);

#endif // __HELPER_CUH