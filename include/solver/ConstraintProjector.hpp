#ifndef __CONSTRAINT_PROJECTOR_HPP
#define __CONSTRAINT_PROJECTOR_HPP

#include "solver/Constraint.hpp"
#include "solver/XPBDSolverUpdates.hpp"

#include "common/TypeList.hpp"

#ifdef HAVE_CUDA
#include "gpu/projector/GPUConstraintProjector.cuh"
#endif

namespace Solver
{

struct ConstraintProjectorOptions
{
    bool with_residual;
    bool with_damping;
    bool first_order;
    Real damping_gamma;

    ConstraintProjectorOptions()
        : with_residual(false), with_damping(false), first_order(false), damping_gamma(0)
    {}
};

template<class Constraint>
class ConstraintProjector
{
    public:
    // TODO: implement num_coordinates and MAX_NUM_COORDINATES - for single constraint projector they will be the same
    constexpr static int NUM_CONSTRAINTS = 1;
    constexpr static int MAX_NUM_COORDINATES = Constraint::NUM_COORDINATES;

    typedef TypeList<Constraint> ConstraintTypes;

    public:
    explicit ConstraintProjector(Real dt, Constraint* constraint_ptr)
        : _dt(dt), _constraint(constraint_ptr), _valid(true)
    {
    }

    void setValidity(bool valid) { _valid = valid; }
    bool isValid() const { return _valid; }

    int numCoordinates() { return MAX_NUM_COORDINATES; }

    void initialize()
    {
        _lambda = 0;
    }

    void project(CoordinateUpdate* coordinate_updates_ptr)
    {
        Real C;
        Real delC[Constraint::NUM_COORDINATES];
        _constraint->evaluateWithGradient(&C, delC);

        // if inequality constraint, make sure that the constraint should actually be enforce
        if (C > 0 && _constraint->isInequality())
        {
            for (int i = 0; i < Constraint::NUM_COORDINATES; i++) 
            { 
                coordinate_updates_ptr[i].ptr = nullptr;
                coordinate_updates_ptr[i].update = 0;
            }
            return;
        }

        // calculate LHS of lambda update: delC^T * M^-1 * delC
        Real alpha_tilde = _constraint->alpha() / (_dt * _dt);
        Real LHS = alpha_tilde;
        const std::vector<PositionReference>& positions = _constraint->positions();
        
        for (int i = 0; i < Constraint::NUM_POSITIONS; i++)
        {
            LHS += positions[i].inv_mass * (delC[3*i]*delC[3*i] + delC[3*i+1]*delC[3*i+1] + delC[3*i+2]*delC[3*i+2]);
        }

        // compute RHS of lambda update: -C - alpha_tilde*lambda
        Real RHS = -C - alpha_tilde * _lambda;

        // compute lambda update
        Real dlam = RHS / LHS;
        _lambda += dlam;

        // compute position updates
        for (int i = 0; i < Constraint::NUM_POSITIONS; i++)
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
    }

    #ifdef HAVE_CUDA
    typedef GPUConstraintProjector<typename Constraint::GPUConstraintType> GPUConstraintProjectorType;
    GPUConstraintProjectorType createGPUConstraintProjector() const
    {
        typename Constraint::GPUConstraintType gpu_constraint = _constraint->createGPUConstraint();
        return GPUConstraintProjectorType(std::move(gpu_constraint), _dt);
    }
    #endif

    private:
    Real _dt;
    Real _lambda;
    Constraint* _constraint;
    bool _valid;
    

};


} // Solver

#endif // __CONSTRAINT_PROJECTOR_HPP