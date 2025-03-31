#ifndef __XPBD_GAUSS_SEIDEL_SOLVER_HPP
#define __XPBD_GAUSS_SEIDEL_SOLVER_HPP

#include "solver/XPBDSolver.hpp"

namespace Solver
{

/** Enforces constraints using the XPBD algorithm with a Gauss-Seidel update strategy, where the state is updated after every constraint projection.
 * This is the algorithm/update strategy proposed by the original XPBD paper.
 */
template <typename ...ConstraintProjectors>
class XPBDGaussSeidelSolver : public XPBDSolver<ConstraintProjectors...>
{
    public:
    /** Same constructor as XPBDSolver */
    explicit XPBDGaussSeidelSolver(Sim::XPBDMeshObject_Base* obj, int num_iter, XPBDSolverResidualPolicyEnum residual_policy)
        : XPBDSolver<ConstraintProjectors...>(obj, num_iter, residual_policy)
    {
    }

    protected:
    /** Implements a Gauss-Seidel update strategy for constraint projection.
     * @param data - the pre-allocated data block to use for evaluating the constraints and their gradients. Assumes that it is large enough to accomodate the ConstraintProjector with the largest memory requirement.
     */
    virtual void _iterateConstraints() override
    {

        this->_constraint_projectors.for_each_element([&](auto& proj)
        {
            if (!proj.isValid())
                return;

            // TODO: check if proj is a RigidBodyConstraintProjector
            // if constexpr ()
            proj.project(this->_coordinate_updates.data());

            // apply the position updates
            for (int i = 0; i < proj.numCoordinates(); i++)
            {
                if (this->_coordinate_updates[i].ptr)
                    *(this->_coordinate_updates[i].ptr) += this->_coordinate_updates[i].update;
            }
        });
    }
};

} // namespace Solver


#endif // __XPBD_GAUSS_SEIDEL_SOLVER_HPP