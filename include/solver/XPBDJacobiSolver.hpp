#ifndef __XPBD_JACOBI_SOLVER_HPP
#define __XPBD_JACOBI_SOLVER_HPP

#include "solver/XPBDSolver.hpp"

namespace Solver
{

/** Enforces constraints using the XPBD algorithm with Jacobi iterations, where the state is updated after all constraint projections.
 */
template <typename ...ConstraintProjectors>
class XPBDJacobiSolver : public XPBDSolver<ConstraintProjectors...>
{
    public:
    /** Same constructor as XPBDSolver */
    explicit XPBDJacobiSolver(Sim::XPBDMeshObject_Base* obj, int num_iter, XPBDResidualPolicy residual_policy)
        : XPBDSolver<ConstraintProjectors...>(obj, num_iter, residual_policy)
    {}

    void setup()
    {
        
    }

    // TODO: figure out how to "override" XPBDSolver functionality for addConstraintProjector
    template<class... Constraints>
    void addConstraintProjector(Real dt, const ConstraintProjectorOptions& options, Constraints* ... constraints)
    {
        auto projector = this->_createConstraintProjector(dt, options, constraints...);
    
        // make sure that the data buffer of the coordinate updates vector is large enough to accomodate the new projector
        this->_coordinate_updates.resize(this->_coordinate_updates.size() + projector.numCoordinates());
    
        // increase the total number of constraints (needed for the constraint residual size)
        this->_num_constraints += decltype(projector)::NUM_CONSTRAINTS;
    
        // check if primary residual is needed for constraint projection
        if (options.with_residual)
        {
            // TODO: FIX THIS for the NEW APPROACH!!!

            // _constraints_using_primary_residual = true;
            // if (WithDistributedPrimaryResidual* wpr = dynamic_cast<WithDistributedPrimaryResidual*>(projector.get()))
            // {
            //     wpr->setPrimaryResidual(_primary_residual.data());
            // }
        }
            
        this->_constraint_projectors.template push_back<decltype(projector)>(std::move(projector));
    }


    protected:
    /** Implements a Jacobi update strategy for constraint projection.
     */
    /** Implements a Gauss-Seidel update strategy for constraint projection.
     * @param data - the pre-allocated data block to use for evaluating the constraints and their gradients. Assumes that it is large enough to accomodate the ConstraintProjector with the largest memory requirement.
     */
    void _iterateConstraints()
    {
        int total_coord_updates = 0;
        this->_constraint_projectors.for_each_element([&](auto& proj)
        {
            if (!proj.isValid())
                return;

            // TODO: check if proj is a RigidBodyConstraintProjector
            // if constexpr ()

            proj.project(this->_coordinate_updates.data() + total_coord_updates);
            total_coord_updates += proj.numCoordinates();
        });

        // apply the position updates
        for (int i = 0; i < total_coord_updates; i++)
        {
            if (this->_coordinate_updates[i].ptr)
                *(this->_coordinate_updates[i].ptr) += this->_coordinate_updates[i].update;
        }
    }
};

} // namespace Solver


#endif // __XPBD_JACOBI_SOLVER_HPP