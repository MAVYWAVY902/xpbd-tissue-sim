#ifndef __XPBD_JACOBI_SOLVER_HPP
#define __XPBD_JACOBI_SOLVER_HPP

#include "solver/XPBDSolver.hpp"

namespace Solver
{

/** Enforces constraints using the XPBD algorithm with Jacobi iterations, where the state is updated after all constraint projections.
 */
class XPBDJacobiSolver : public XPBDSolver
{
    public:
    /** Same constructor as XPBDSolver */
    explicit XPBDJacobiSolver(Sim::XPBDMeshObject* obj, int num_iter, XPBDSolverResidualPolicyEnum residual_policy);

    protected:
    /** Implements a Gauss-Seidel update strategy for constraint projection.
     * @param data - the pre-allocated data block to use for evaluating the constraints and their gradients. Assumes that it is large enough to accomodate the ConstraintProjector with the largest memory requirement.
     */
    virtual void _solveConstraints(Real* data) override;

    protected:
    Geometry::Mesh::VerticesMat _position_updates;
};

} // namespace Solver


#endif // __XPBD_JACOBI_SOLVER_HPP