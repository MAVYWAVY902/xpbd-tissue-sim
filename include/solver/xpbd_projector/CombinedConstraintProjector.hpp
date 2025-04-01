#ifndef __COMBINED_CONSTRAINT_PROJECTOR_HPP
#define __COMBINED_CONSTRAINT_PROJECTOR_HPP

#include "solver/constraint/Constraint.hpp"
#include "solver/xpbd_solver/XPBDSolverUpdates.hpp"

#include "common/TypeList.hpp"

#ifdef HAVE_CUDA
#include "gpu/projector/GPUCombinedConstraintProjector.cuh"
#endif

namespace Solver
{

template<bool IsFirstOrder, class Constraint1, class Constraint2>
class CombinedConstraintProjector
{
    public:
    constexpr static int NUM_CONSTRAINTS = 2;
    constexpr static int MAX_NUM_COORDINATES = Constraint1::NUM_COORDINATES + Constraint2::NUM_COORDINATES;
    
    using constraint_type_list = TypeList<Constraint1, Constraint2>;
    constexpr static bool is_first_order = IsFirstOrder;

    public:
    explicit CombinedConstraintProjector(Real dt, Constraint1* constraint1, Constraint2* constraint2)
        : _dt(dt), _constraint1(constraint1), _constraint2(constraint2)
    {
    }

    void setValidity(bool valid) { _valid = valid; }
    bool isValid() const { return _valid; }

    int numCoordinates() { return Constraint1::NUM_COORDINATES; } // TODO: for now we're assuming that constraint1 and constraint2 share the same coords exactly

    void initialize()
    {
        _lambda[0] = 0;
        _lambda[1] = 0;
    }

    void project(CoordinateUpdate* coordinate_updates_ptr)
    {
        Real C[2];
        Real delC[Constraint1::NUM_COORDINATES + Constraint2::NUM_COORDINATES];
        _constraint1->evaluateWithGradient(C, delC);
        _constraint2->evaluateWithGradient(C+1, delC+Constraint1::NUM_COORDINATES);

        // std::cout << "Cd_grad: " << delC[0]<<", "<<delC[1]<<", "<<delC[2]<<", "<<delC[3]<<", "<<delC[4]<<", "<<delC[5]<<", "<<delC[6]<<", "<<delC[7]<<", "<<delC[8]<<", "<<delC[9]<<", "<<delC[10]<<", "<<delC[11]<<std::endl;
        // std::cout << "Ch_grad: " << delC[12]<<", "<<delC[13]<<", "<<delC[14]<<", "<<delC[15]<<", "<<delC[16]<<", "<<delC[17]<<", "<<delC[18]<<", "<<delC[19]<<", "<<delC[20]<<", "<<delC[21]<<", "<<delC[22]<<", "<<delC[23]<<std::endl;
        
        // compute alpha_tilde
        // if using 1st order XPBD method - alpha_tilde = alpha / dt
        // if using 2nd order XPBD method - alpha_tilde = alpha / dt^2
        Real alpha_tilde[2];
        if constexpr(IsFirstOrder)
        {
            alpha_tilde[0] = _constraint1->alpha() / _dt;
            alpha_tilde[1] = _constraint2->alpha() / _dt;
        }
        else
        {
            alpha_tilde[0] = _constraint1->alpha() / (_dt * _dt);
            alpha_tilde[1] = _constraint2->alpha() / (_dt * _dt);
        }

        // calculate LHS of lambda update: delC^T * M^-1 * delC
        Real LHS[4];
        for (int ci = 0; ci < 2; ci++)
        {
            for (int cj = 0; cj < 2; cj++)
            {
                if (ci == cj) LHS[cj*2 + ci] = alpha_tilde[ci];
                else LHS[cj*2 + ci] = 0;
            }
        }

        // TODO: FOR NOW assuming that both constraints share exactly the same coordinates, in exactly the same order. FIND A MORE GENERAL WAY
        for (int ci = 0; ci < 2; ci++)
        {
            float* delC_i = delC + ci*Constraint1::NUM_COORDINATES;
            for (int cj = ci; cj < 2; cj++)
            {
                float* delC_j = delC + cj*Constraint1::NUM_COORDINATES;

                for (int i = 0; i < Constraint1::NUM_POSITIONS; i++)
                {
                    LHS[cj*2 + ci] += _constraint1->positions()[i].inv_mass * (delC_i[3*i]*delC_j[3*i] + delC_i[3*i+1]*delC_j[3*i+1] + delC_i[3*i+2]*delC_j[3*i+2]);
                }

                LHS[ci*2 + cj] = LHS[cj*2 + ci];
            }
            
        }
        // compute RHS of lambda update: -C - alpha_tilde * lambda
        float RHS[2];
        for (int ci = 0; ci < 2; ci++)
        {
            RHS[ci] = -C[ci] - alpha_tilde[ci] * _lambda[ci];
        }

        // compute lambda update - solve 2x2 system
        float dlam[2];
        const float det = LHS[0]*LHS[3] - LHS[1]*LHS[2];

        dlam[0] = (RHS[0]*LHS[3] - RHS[1]*LHS[2]) / det;
        dlam[1] = (RHS[1]*LHS[0] - RHS[0]*LHS[1]) / det;

        // std::cout << "dlam[0]: " << dlam[0] << ", dlam[1]: " << dlam[1] << std::endl;

        // update lambdas
        _lambda[0] += dlam[0];
        _lambda[1] += dlam[1];

        // compute position updates
        // TODO: FOR NOW ASSUMING THAT BOTH CONSTRAINTS SHARE EXACTLY THE THE SAME COORDINATES, IN THE SAME ORDER
        Real* delC_c2 = delC + Constraint2::NUM_COORDINATES;
        for (int i = 0; i < Constraint1::NUM_POSITIONS; i++)
        {
            Real update_x = _constraint1->positions()[i].inv_mass * (delC[3*i] * dlam[0] + delC_c2[3*i] * dlam[1]);
            Real update_y = _constraint1->positions()[i].inv_mass * (delC[3*i+1] * dlam[0] + delC_c2[3*i+1] * dlam[1]);
            Real update_z = _constraint1->positions()[i].inv_mass * (delC[3*i+2] * dlam[0] + delC_c2[3*i+2] * dlam[1]);
            
            coordinate_updates_ptr[3*i].ptr = _constraint1->positions()[i].position_ptr;
            coordinate_updates_ptr[3*i].update = update_x;
            coordinate_updates_ptr[3*i+1].ptr = _constraint1->positions()[i].position_ptr+1;
            coordinate_updates_ptr[3*i+1].update = update_y;
            coordinate_updates_ptr[3*i+2].ptr = _constraint1->positions()[i].position_ptr+2;
            coordinate_updates_ptr[3*i+2].update = update_z;
        }
    }

    #ifdef HAVE_CUDA
    typedef GPUCombinedConstraintProjector<typename Constraint1::GPUConstraintType, typename Constraint2::GPUConstraintType> GPUConstraintProjectorType;
    GPUConstraintProjectorType createGPUConstraintProjector() const
    {
        typename Constraint1::GPUConstraintType gpu_constraint1 = _constraint1->createGPUConstraint();
        typename Constraint2::GPUConstraintType gpu_constraint2 = _constraint2->createGPUConstraint();
        return GPUConstraintProjectorType(std::move(gpu_constraint1), std::move(gpu_constraint2), _dt);
    }
    #endif

    private:
    Real _dt;
    Real _lambda[2];
    Constraint1* _constraint1;
    Constraint2* _constraint2;
    bool _valid;
};


} // namespace Solver

#endif // __COMBINED_CONSTRAINT_PROJECTOR_HPP