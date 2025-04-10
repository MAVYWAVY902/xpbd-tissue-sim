#ifndef __GPU_RIGID_DEFORMABLE_COLLISION_CONSTRAINT_CUH
#define __GPU_RIGID_DEFORMABLE_COLLISION_CONSTRAINT_CUH

#include "gpu/common/gcc_hostdevice.hpp"
#include "gpu/common/helper.cuh"
#include "gpu/constraint/GPUPositionReference.cuh"

#include "utils/CudaHelperMath.h"

// TODO - actually implement RigidDeformableCollisionConstraint
// right now this is just static-deformable collision constraint
struct GPURigidDeformableCollisionConstraint
{
    GPUPositionReference positions[3];
    float bary_coords[3];
    float p[3];
    float n[3];
    float alpha;
    
    // __host__ GPURigidDeformableCollisionConstraint(Sim::XPBDMeshObject_Base* xpbd_obj, const int v0, const int v1, const int v2,
    //                                                 const float u, const float v, const float w,
    //                                                 const Eigen::Vector3f& static_point, const Eigen::Vector3f& collision_normal)
    // {
    //     positions[0].inv_mass = 1 / xpbd_obj->vertexMass(v0);
    //     positions[0].index = v0;
        
    //     positions[1].inv_mass = 1 / xpbd_obj->vertexMass(v1);
    //     positions[1].index = v1;
        
    //     positions[2].inv_mass = 1 / xpbd_obj->vertexMass(v2);
    //     positions[2].index = v2;
        
    //     bary_coords[0] = u;
    //     bary_coords[1] = v;
    //     bary_coords[2] = w;
        
    //     p[0] = static_point[0];
    //     p[1] = static_point[1];
    //     p[2] = static_point[2];
        
    //     n[0] = collision_normal[0];
    //     n[1] = collision_normal[1];
    //     n[2] = collision_normal[2];

    //     alpha = 0;
    // }

    __host__ GPURigidDeformableCollisionConstraint( int v0_ind, float inv_m0,
                                                    int v1_ind, float inv_m1,
                                                    int v2_ind, float inv_m2,
                                                    float u, float v, float w,
                                                    const Eigen::Vector3f& static_point, const Eigen::Vector3f& collision_normal)
    {
        positions[0].inv_mass = inv_m0;
        positions[0].index = v0_ind;

        positions[1].inv_mass = inv_m1;
        positions[1].index = v1_ind;

        positions[2].inv_mass = inv_m2;
        positions[2].index = v2_ind;

        bary_coords[0] = u;
        bary_coords[1] = v;
        bary_coords[2] = w;

        p[0] = static_point[0];
        p[1] = static_point[1];
        p[2] = static_point[2];

        n[0] = collision_normal[0];
        n[1] = collision_normal[1];
        n[2] = collision_normal[2];
    }
    
    __device__ GPURigidDeformableCollisionConstraint() {}
    
    constexpr __host__ __device__ static int numPositions() { return 3; }

    constexpr __host__ __device__ static bool isInequality() { return true; }
    
    __device__ void _loadVertices(const float* vertices, float* x)
    {
    *reinterpret_cast<float3*>(x)   = *reinterpret_cast<const float3*>(vertices + 3*positions[0].index);
    *reinterpret_cast<float3*>(x+3) = *reinterpret_cast<const float3*>(vertices + 3*positions[1].index);
    *reinterpret_cast<float3*>(x+6) = *reinterpret_cast<const float3*>(vertices + 3*positions[2].index);
    }
    
    __device__ void evaluate(const float* vertices, float* C)
    {
    float x[9];
    _loadVertices(vertices, x);
    
    float diff[3];
    for (int i = 0; i < 3; i++)
    {
    diff[i] = bary_coords[0] * x[i] + bary_coords[1] * x[i+3] + bary_coords[2] * x[i+6] - p[i];
    }
    
    *C = Vec3Dot(n, diff);
    }
    
    __device__ void gradient(const float* /* vertices */, float* delC)
    {
    delC[0] = bary_coords[0] * n[0];    delC[1] = bary_coords[0] * n[1];    delC[2] = bary_coords[0] * n[2];
    delC[3] = bary_coords[1] * n[0];    delC[4] = bary_coords[1] * n[1];    delC[5] = bary_coords[1] * n[2];
    delC[6] = bary_coords[2] * n[0];    delC[7] = bary_coords[2] * n[1];    delC[8] = bary_coords[2] * n[2];
    }
    
    __device__ void evaluateWithGradient(const float* vertices, float* C, float* delC)
    {
    evaluate(vertices, C);
    gradient(vertices, delC);
    }
};


#endif // __GPU_RIGID_DEFORMABLE_COLLISION_CONSTRAINT_CUH