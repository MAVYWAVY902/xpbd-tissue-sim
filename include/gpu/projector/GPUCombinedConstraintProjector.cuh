#ifndef __GPU_COMBINED_CONSTRAINT_PROJECTOR_CUH
#define __GPU_COMBINED_CONSTRAINT_PROJECTOR_CUH

#include "utils/CudaHelperMath.h"

#include <iostream>

template<class Constraint1, class Constraint2>
struct GPUCombinedConstraintProjector
{
    float dt;
    float lambda[2];
    Constraint1 constraint1;
    Constraint2 constraint2;

    __host__ GPUCombinedConstraintProjector(const Constraint1& constraint1_, const Constraint2& constraint2_, float dt_)
        : dt(dt_), constraint1(constraint1_), constraint2(constraint2_)
    {
        std::cout << "GPUCombinedConstraintProjector const& constructor!" << std::endl;
    }
    
    __host__ GPUCombinedConstraintProjector(Constraint1&& constraint1_, Constraint2&& constraint2_, float dt_)
        : dt(dt_), constraint1(std::move(constraint1_)), constraint2(std::move(constraint2_))
    {
        std::cout << "GPUCombinedConstraintProjector && constructor!" << std::endl;
    }
    
    __device__ GPUCombinedConstraintProjector()
    {

    }

    __device__ void initialize()
    {
        lambda[0] = 0;
        lambda[1] = 0;
    }

    __device__ void project(const float* vertices, float* new_vertices)
    {
        float C[2];
        float delC[2*(Constraint1::numPositions()*3)];  // FOR NOW: we assume that both constraints share the same exact positions

        // evaluate constraint and its gradient
        constraint1.evaluateWithGradient(vertices, C, delC);
        constraint2.evaluateWithGradient(vertices, C + 1, delC + constraint1.numPositions()*3);

        float alpha_tilde[2] = {constraint1.alpha / (dt*dt), constraint2.alpha / (dt*dt)};

        // compute LHS of lambda upate - delC^T * M^-1 * delC
        float LHS[4];

        for (int ci = 0; ci < 2; ci++)
        {
            for (int cj = 0; cj < 2; cj++)
            {
                if (ci == cj) LHS[cj*2 + ci] = alpha_tilde[ci];
                else LHS[cj*2 + ci] = 0;
            }
        }

        for (int ci = 0; ci < 2; ci++)
        {
            float* delC_i = delC + ci*constraint1.numPositions()*3;
            for (int cj = ci; cj < 2; cj++)
            {
                float* delC_j = delC + cj*constraint1.numPositions()*3;

                for (int i = 0; i < constraint1.numPositions(); i++)
                {
                    LHS[cj*2 + ci] += constraint1.positions[i].inv_mass * (delC_i[3*i]*delC_j[3*i] + delC_i[3*i+1]*delC_j[3*i+1] + delC_i[3*i+2]*delC_j[3*i+2]);
                }

                LHS[ci*2 + cj] = LHS[cj*2 + ci];
            }
            
        }

        // compute RHS of lambda update - -C - alpha_tilde * lambda
        float RHS[2];
        for (int ci = 0; ci < 2; ci++)
        {
            RHS[ci] = -C[ci] - alpha_tilde[ci] * lambda[ci];
        }

        // compute lambda update - solve 2x2 system
        float dlam[2];
        const float det = LHS[0]*LHS[3] - LHS[1]*LHS[2];

        dlam[0] = (RHS[0]*LHS[3] - RHS[1]*LHS[2]) / det;
        dlam[1] = (RHS[1]*LHS[0] - RHS[0]*LHS[1]) / det;

        // printf("RHS[0]: %f, RHS[1]: %f\n", RHS[0], RHS[1]);
        // printf("LHS(0,0): %f, LHS(1,0): %f, LHS(0,1): %f, LHS(1,1): %f\n", LHS[0], LHS[1], LHS[2], LHS[3] );
        // printf("dlam[0]: %f, dlam[1]: %f\n", dlam[0], dlam[1]);

        // update lambda
        lambda[0] += dlam[0];
        lambda[1] += dlam[1];

        // update positions
        float* delC_c2 = delC + constraint1.numPositions()*3;
        for (int i = 0; i < 4; i++)
        {
            float* v_ptr = new_vertices + 3*constraint1.positions[i].index;
            // const float pos_update1 = constraint1.positions[i].inv_mass * (delC[3*i] * dlam[0] + delC_c2[3*i] * dlam[1]);
            // const float pos_update2 = constraint1.positions[i].inv_mass * (delC[3*i+1] * dlam[0] + delC_c2[3*i+1] * dlam[1]);
            // const float pos_update3 = constraint1.positions[i].inv_mass * (delC[3*i+2] * dlam[0] + delC_c2[3*i+2] * dlam[1]);
            atomicAdd(v_ptr,     constraint1.positions[i].inv_mass * (delC[3*i] * dlam[0] + delC_c2[3*i] * dlam[1]));
            atomicAdd(v_ptr + 1, constraint1.positions[i].inv_mass * (delC[3*i+1] * dlam[0] + delC_c2[3*i+1] * dlam[1]));
            atomicAdd(v_ptr + 2, constraint1.positions[i].inv_mass * (delC[3*i+2] * dlam[0] + delC_c2[3*i+2] * dlam[1]));

            // printf("update: (%f, %f, %f)\n", pos_update1, pos_update2, pos_update3);
        }

        // printf("new_vertices0: %f, %f, %f\n", new_vertices[0], new_vertices[1], new_vertices[2]);
        // printf("new_vertices1: %f, %f, %f\n", new_vertices[3], new_vertices[4], new_vertices[5]);
        // printf("new_vertices2: %f, %f, %f\n", new_vertices[6], new_vertices[7], new_vertices[8]);
        // printf("new_vertices3: %f, %f, %f\n", new_vertices[9], new_vertices[10], new_vertices[11]);
    }
};

#endif // __GPU_COMBINED_CONSTRAINT_PROJECTOR_CUH