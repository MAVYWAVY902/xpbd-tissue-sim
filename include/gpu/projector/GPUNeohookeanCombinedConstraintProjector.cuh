#ifndef __GPU_NEOHOOKEAN_COMBINED_CONSTRAINT_PROJECTOR_CUH
#define __GPU_NEOHOOKEAN_COMBINED_CONSTRAINT_PROJECTOR_CUH

#include "gpu/common/helper.cuh"
#include "gpu/projector/GPUCombinedConstraintProjector.cuh"
#include "gpu/constraint/GPUDeviatoricConstraint.cuh"
#include "gpu/constraint/GPUHydrostaticConstraint.cuh"

template <>
struct GPUCombinedConstraintProjector<GPUDeviatoricConstraint, GPUHydrostaticConstraint>
{
    float dt;
    float lambda[2];
    GPUPositionReference positions[4];
    float Q[9];
    float alpha_h;
    float alpha_d;
    float gamma;

    __host__ GPUCombinedConstraintProjector(const GPUDeviatoricConstraint& constraint1_, const GPUHydrostaticConstraint& constraint2_, float dt_);
    
    __host__ GPUCombinedConstraintProjector(GPUDeviatoricConstraint&& constraint1_, GPUHydrostaticConstraint&& constraint2_, float dt_);

    __device__ GPUCombinedConstraintProjector();

    __device__ void initialize();

    __device__ void project(const float* vertices, float* new_vertices);
};

#endif // __GPU_NEOHOOKEAN_COMBINED_CONSTRAINT_PROJECTOR_CUH