#ifndef __XPBD_GAUSS_SEIDEL_SOLVER_HPP
#define __XPBD_GAUSS_SEIDEL_SOLVER_HPP

#include "solver/XPBDSolver.hpp"

namespace Solver
{

class XPBDGaussSeidelSolver : public XPBDSolver
{
    public:
    explicit XPBDGaussSeidelSolver(XPBDMeshObject const* obj, unsigned num_iter, XPBDResidualPolicy residual_policy);

    protected:
    virtual void _solveConstraints() override;
};

} // namespace Solver


#endif // __XPBD_GAUSS_SEIDEL_SOLVER_HPP