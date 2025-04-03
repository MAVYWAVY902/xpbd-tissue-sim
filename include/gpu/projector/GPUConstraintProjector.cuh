#ifndef __GPU_CONSTRAINT_PROJECTOR_CUH
#define __GPU_CONSTRAINT_PROJECTOR_CUH

#include "utils/CudaHelperMath.h"
#include "common/VariadicContainer.hpp"

template<bool IsFirstOrder, class Constraint>
struct GPUConstraintProjector
{
    float dt;
    float lambda;
    Constraint constraint;

    __host__ GPUConstraintProjector(const Constraint& constraint_, float dt_)
        :  dt(dt_), constraint(constraint_)
    {

    } 
    __host__ GPUConstraintProjector(Constraint&& constraint_, float dt_)
        :  dt(dt_), constraint(std::move(constraint_))
    {
    }

    __device__ GPUConstraintProjector()
    {
        
    }

    __device__ void initialize()
    {
        lambda = 0;
    }

    __device__ void project(const float* vertices, float* new_vertices)
    {
        float C;
        float delC[Constraint::numPositions()*3];

        // evaluate constraint and its gradient
        constraint.evaluateWithGradient(vertices, &C, delC);

        float alpha_tilde;
        if constexpr (IsFirstOrder)
        {
            alpha_tilde = constraint.alpha / dt;
        }
        else
        {
            alpha_tilde = constraint.alpha / (dt*dt);
        }

        // compute LHS of lambda upate - delC^T * M^-1 * delC
        float LHS = alpha_tilde;
        for (int i = 0; i < constraint.numPositions(); i++)
        {
            LHS += constraint.positions[i].inv_mass * (delC[3*i]*delC[3*i] + delC[3*i+1]*delC[3*i+1] + delC[3*i+2]*delC[3*i+2]);
        }

        // compute RHS of lambda update - -C - alpha_tilde * lambda
        float RHS = -C - alpha_tilde * lambda;

        // compute lambda update
        float dlam = RHS / LHS;
        lambda += dlam;

        // update positions
        for (int i = 0; i < constraint.numPositions(); i++)
        {
            float* v_ptr = new_vertices + constraint.positions[i].index;
            atomicAdd(v_ptr,     constraint.positions[i].inv_mass * delC[3*i] * dlam);
            atomicAdd(v_ptr + 1, constraint.positions[i].inv_mass * delC[3*i+1] * dlam);
            atomicAdd(v_ptr + 2, constraint.positions[i].inv_mass * delC[3*i+2] * dlam);
        }
    }
};

#endif // __GPU_CONSTRAINT_PROJECTOR_CUH