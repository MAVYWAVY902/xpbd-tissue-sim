#ifndef __GPU_ATTACHMENT_CONSTRAINT_CUH
#define __GPU_ATTACHMENT_CONSTRAINT_CUH

#include "gpu/common/gcc_hostdevice.hpp"
#include "gpu/common/helper.cuh"
#include "gpu/constraint/GPUPositionReference.cuh"

#include "utils/CudaHelperMath.h"

struct GPUAttachmentConstraint
{
    float3 attached_pos;
    float3 attachment_offset;
    GPUPositionReference position;
    float alpha;

    __host__ GPUAttachmentConstraint( int v0_ind, float inv_m0,
                                      const Eigen::Vector3d& attach_pt, const Eigen::Vector3d& offset,
                                      float alpha_)
    {
        position.inv_mass = inv_m0;
        position.index = v0_ind;

        attached_pos[0] = attach_pt[0];
        attached_pos[1] = attach_pt[1];
        attached_pos[2] = attach_pt[2];

        attachment_offset[0] = offset[0];
        attachment_offset[1] = offset[1];
        attachment_offset[2] = offset[2];

        alpha = alpha_;
    }

    __device__ GPUAttachmentConstraint() {}

    constexpr __host__ __device__ static int numPositions() { return 1; } 

    constexpr __host__ __device__ static bool isInequality() { return false; }

    // __device__ void _loadVertices(float* x)
    // {
    //     x[0] = positions[0].ptr[0];  x[1] = positions[0].ptr[1];  x[2] = positions[0].ptr[2];
    //     x[3] = positions[1].ptr[0];  x[4] = positions[1].ptr[1];  x[5] = positions[1].ptr[2];
    //     x[6] = positions[2].ptr[0];  x[7] = positions[2].ptr[1];  x[8] = positions[2].ptr[2];
    //     x[9] = positions[3].ptr[0];  x[10] = positions[3].ptr[1]; x[11] = positions[3].ptr[2];
    // }
    __device__ void _loadVertices(const float* vertices, float3* x)
    {
        *reinterpret_cast<float3*>(x)   = *reinterpret_cast<const float3*>(vertices + 3*positions[0].index);
    }

    __device__ void evaluate(const float* vertices, float* C)
    {
        float3 x;
        _loadVertices(vertices, &x);

        float3 diff = x - (attached_pos + attachment_offset);
        *C = norm(diff);
    }

    __device__ void gradient(const float* vertices, float* delC)
    {
        float3 x;
        _loadVertices(vertices, &x);

        float3 attach_pt = attached_pos + attachment_offset;
        const double dist = norm( x - attach_pt );
        grad[0] = (x.x - attach_pt.x) / dist;
        grad[1] = (x.y - attach_pt.y) / dist;
        grad[2] = (x.z - attach_pt.z) / dist;
    }

    __device__ void evaluateWithGradient(double* C, double* grad) const override
    {
        float3 x;
        _loadVertices(vertices, &x);

        float3 attach_pt = attached_pos + attachment_offset;
        const double dist = norm( x - attach_pt );
        *C = dist;
        
        grad[0] = (x.x - attach_pt.x) / dist;
        grad[1] = (x.y - attach_pt.y) / dist;
        grad[2] = (x.z - attach_pt.z) / dist;
    }
};

#endif // __GPU_ATTACHMENT_CONSTRAINT_CUH