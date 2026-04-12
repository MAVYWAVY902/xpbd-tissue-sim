/** Defines a bunch of POD structs for use on the GPU */

#ifndef __GPU_STRUCTS_HPP
#define __GPU_STRUCTS_HPP

#include <cuda_runtime_api.h>

namespace Sim
{

struct GPUCollision
{
    float penetration_dist;
    float3 bary_coords;
};


//////////////////////////////////////////////////////////
// SDF structs for the GPU
//////////////////////////////////////////////////////////

struct GPUSphereSDF
{
    // fields that change throughout the sim
    float3 position;

    // fields that are constant throughout the sim
    float radius;
};

struct GPUBoxSDF
{
    // fields that change throughout the sim
    float3 position;
    float4 orientation;

    // fields that are constant throughout the sim
    float3 size;
};

struct GPUCylinderSDF
{
    // fields that change throughout the sim
    float3 position;
    float4 orientation;

    // fields that are constant throughout the sim
    float radius;
    float height;
};

struct GPUMeshSDF
{
    // fields that change throughout the sim
    float3 position;
    float4 orientation;

    // fields that are constant throughout the sim
    float3 grid_cell_size;
    float3 grid_bbox_min;
    int3 grid_dims;
    cudaPitchedPtr dev_dist_grid_ptr; 
};

} // namespace Sim

#endif // __GPU_STRUCTS_HPP