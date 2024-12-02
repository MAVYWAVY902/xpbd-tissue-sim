#ifndef __XPBD_GAUSS_SEIDEL_SOLVER_HPP
#define __XPBD_GAUSS_SEIDEL_SOLVER_HPP

#include "solver/XPBDSolver.hpp"

namespace Solver
{

/** Enforces constraints using the XPBD algorithm with a Gauss-Seidel update strategy, where the state is updated after every constraint projection.
 * This is the algorithm/update strategy proposed by the original XPBD paper.
 */
class XPBDGaussSeidelSolver : public XPBDSolver
{
    public:
    /** Same constructor as XPBDSolver */
    explicit XPBDGaussSeidelSolver(XPBDMeshObject const* obj, unsigned num_iter, XPBDResidualPolicy residual_policy);

    protected:
    /** Implements a Gauss-Seidel update strategy for constraint projection.
     * @param data - the pre-allocated data block to use for evaluating the constraints and their gradients. Assumes that it is large enough to accomodate the ConstraintProjector with the largest memory requirement.
     */
    virtual void _solveConstraints(double* data) override;
};

} // namespace Solver


#endif // __XPBD_GAUSS_SEIDEL_SOLVER_HPP