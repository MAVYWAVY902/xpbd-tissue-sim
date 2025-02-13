#ifndef __COLLISION_CUH
#define __COLLISION_CUH

#include "gpu/GPUStructs.hpp"

#include "gpu/GPUResource.hpp"
#include "gpu/ArrayGPUResource.hpp"
#include "gpu/MeshGPUResource.hpp"

__host__ void launchCollisionKernel(const Sim::HostReadableGPUResource* sdf_resource, const Sim::MeshGPUResource* mesh_resource, int num_vertices, int num_faces, Sim::ArrayGPUResource<Sim::GPUCollision>* collision_resource);

__device__ float sphere_sdf_distance(const Sim::GPUSphereSDF* sphere_sdf, const float3& x);

__global__ void sphereMeshCollisionDetection(const Sim::GPUSphereSDF* sphere_sdf, const float* vertices, int num_vertices, const int* faces, int num_faces, Sim::GPUCollision* collisions);

__global__ void sphereMeshCollisionDetectionParallel(const Sim::GPUSphereSDF* sphere_sdf, const float* vertices, int num_vertices, const int* faces, int num_faces, Sim::GPUCollision* collisions);

#endif // __COLLISION_CUH