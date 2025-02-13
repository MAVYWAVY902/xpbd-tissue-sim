/** Defines a bunch of POD structs for use on the GPU */

#ifndef __GPU_STRUCTS_HPP
#define __GPU_STRUCTS_HPP

#include <cuda_runtime_api.h>

namespace Sim
{

// struct GPUCollision
// {
//     float time;
//     float penetration_dist;
//     float3 normal;
//     float3 bary_coords;
//     float3 surface_point;
// };

struct GPUCollision
{
    float penetration_dist;
    float3 bary_coords;
};

struct GPUSphereSDF
{
    float3 position;
    float radius;
};

struct GPUBoxSDF
{
    float3 position;
    float4 orientation;
    float3 size;
};

struct GPUCylinderSDF
{
    float3 position;
    float4 orientation;
    float radius;
    float height;
};

struct GPUMeshSDF
{
    float3 position;
    float4 orientation;
    float* dist_grid;
    float* grad_grid;
};

} // namespace Sim

#endif // __GPU_STRUCTS_HPP