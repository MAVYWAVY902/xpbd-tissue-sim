#ifndef __RIGID_BODY_CONSTRAINT_PROJECTOR_HPP
#define __RIGID_BODY_CONSTRAINT_PROJECTOR_HPP

#include "solver/xpbd_solver/XPBDSolverUpdates.hpp"
#include "solver/constraint/RigidBodyConstraint.hpp"

#include <type_traits>

#ifdef HAVE_CUDA
#include "gpu/projector/GPUConstraintProjector.cuh"
#endif

namespace Solver
{

template<class RBConstraint>
class RigidBodyConstraintProjector
{
    public:
    constexpr static int NUM_CONSTRAINTS = 1;
    constexpr static int NUM_COORDINATES = RBConstraint::NUM_COORDINATES;

    public:
    explicit RigidBodyConstraintProjector(Real dt, RBConstraint* constraint_ptr)
        : _dt(dt), _constraint(constraint_ptr), _valid(true)
    {
    }

    void setValidity(bool valid) { _valid = valid; }
    bool isValid() { return _valid; }

    void initialize()
    {
        _lambda = 0;
    }

    void project(CoordinateUpdate* coordinate_updates_ptr, RigidBodyUpdate* rigid_body_updates_ptr)
    {
        Real C;
        Real delC[RBConstraint::NUM_COORDINATES];
        _constraint->evaluateWithGradient(&C, delC);

        // if inequality constraint, make sure that the constraint should actually be enforce
        if (C > 0 && _constraint->isInequality())
        {
            for (int i = 0; i < RBConstraint::NUM_COORDINATES; i++) 
            { 
                coordinate_updates_ptr[i].ptr = nullptr;
            }
            for (int i = 0; i < RBConstraint::NUM_RIGID_BODIES; i++)
            {
                rigid_body_updates_ptr[i].obj_ptr = nullptr;
            }
            return;
        }

        // calculate LHS of lambda update: delC^T * M^-1 * delC
        Real alpha_tilde = _constraint->alpha() / (_dt * _dt);
        Real LHS = alpha_tilde;
        const std::vector<PositionReference>& positions = _constraint->positions();
        
        for (int i = 0; i < RBConstraint::NUM_POSITIONS; i++)
        {
            LHS += _constraint->positions[i].inv_mass * (delC[3*i]*delC[3*i] + delC[3*i+1]*delC[3*i+1] + delC[3*i+2]*delC[3*i+2]);
        }

        // compute RHS of lambda update: -C - alpha_tilde*lambda
        Real RHS = -C - alpha_tilde * _lambda;

        // compute lambda update
        Real dlam = RHS / LHS;
        _lambda += dlam;

        // compute position updates
        for (int i = 0; i < RBConstraint::NUM_POSITIONS; i++)
        {
            Real update_x = positions[i].inv_mass * delC[3*i] * dlam;
            Real update_y = positions[i].inv_mass * delC[3*i+1] * dlam;
            Real update_z = positions[i].inv_mass * delC[3*i+2] * dlam;
            
            coordinate_updates_ptr[3*i].ptr = positions[i].position_ptr;
            coordinate_updates_ptr[3*i].update = update_x;
            coordinate_updates_ptr[3*i+1].ptr = positions[i].position_ptr+1;
            coordinate_updates_ptr[3*i+1].update = update_y;
            coordinate_updates_ptr[3*i+2].ptr = positions[i].position_ptr+2;
            coordinate_updates_ptr[3*i+2].update = update_z;
        }

        // compute the update for each rigid body using the corresponding RigidBodyXPBDHelper object
        for (int ri = 0; ri < _constraint->numRigidBodies(); ri++)
        {
            rigid_body_updates_ptr[ri].obj_ptr = _constraint->rigidBodies()[ri];
            _constraint->rigidBodyHelpers()[ri]->update(dlam, rigid_body_updates_ptr[ri].position_update, rigid_body_updates_ptr[ri].orientation_update);
        }
    }

    // TODO: implement specific GPUConstraintProjector type for rigid body constraints - need RigidObjectGPUResource and stuff like that
    #ifdef HAVE_CUDA
    typedef GPUConstraintProjector<typename RBConstraint::GPUConstraintType> GPUConstraintProjectorType;
    GPUConstraintProjectorType createGPUConstraintProjector() const
    {
        typename RBConstraint::GPUConstraintType gpu_constraint = _constraint->createGPUConstraint();
        return GPUConstraintProjectorType(std::move(gpu_constraint), _dt);
    }
    #endif

    private:
    Real _dt;
    Real _lambda;
    RBConstraint* _constraint;
    bool _valid;
};

} // namespace Solver

#endif // __RIGID_BODY_CONSTRAINT_PROJECTOR_HPP