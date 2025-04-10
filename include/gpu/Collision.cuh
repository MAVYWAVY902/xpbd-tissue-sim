#ifndef __COLLISION_CUH
#define __COLLISION_CUH

#include "gpu/GPUStructs.hpp"

#include "gpu/resource/GPUResource.hpp"
#include "gpu/resource/WritableArrayGPUResource.hpp"
#include "gpu/resource/MeshGPUResource.hpp"

__host__ void launchCollisionKernel(const Sim::HostReadableGPUResource* sdf_resource, const Sim::MeshGPUResource* mesh_resource, int num_vertices, int num_faces, Sim::WritableArrayGPUResource<Sim::GPUCollision>* collision_resource);

__device__ void global_to_body(const float3& x, const float3& body_position, const float4& body_orientation);

__device__ float sphere_sdf_distance(const Sim::GPUSphereSDF* sphere_sdf, const float3& x);

__device__ float3 sphere_sdf_gradient(const Sim::GPUSphereSDF* sphere_sdf, const float3& x);

__device__ float box_sdf_distance(const Sim::GPUBoxSDF* box_sdf, const float3& x);

__device__ float cyl_sdf_distance(const Sim::GPUCylinderSDF* cyl_sdf, const float3& x);

__device__ float mesh_sdf_distance(const Sim::GPUMeshSDF* mesh_sdf, const float3& x);

__global__ void sphereMeshCollisionDetection(const Sim::GPUSphereSDF* sphere_sdf, const float* vertices, int num_vertices, const int* faces, int num_faces, Sim::GPUCollision* collisions);

__global__ void sphereMeshCollisionDetectionParallel(const Sim::GPUSphereSDF* sphere_sdf, const float* vertices, int num_vertices, const int* faces, int num_faces, Sim::GPUCollision* collisions);

__global__ void boxMeshCollisionDetection(const Sim::GPUBoxSDF* box_sdf, const float* vertices, int num_vertices, const int* faces, int num_faces, Sim::GPUCollision* collisions);

__global__ void cylinderMeshCollisionDetection(const Sim::GPUCylinderSDF* cyl_sdf, const float* vertices, int num_vertices, const int* faces, int num_faces, Sim::GPUCollision* collisions);

__global__ void meshMeshCollisionDetection(const Sim::GPUMeshSDF* mesh_sdf, const float* vertices, int num_vertices, const int* faces, int num_faces, Sim::GPUCollision* collisions);

#endif // __COLLISION_CUH