#include "solver/XPBDGaussSeidelSolver.hpp"
#include "solver/Constraint.hpp"
#include "simobject/XPBDMeshObject.hpp"

namespace Solver
{

XPBDGaussSeidelSolver::XPBDGaussSeidelSolver(XPBDMeshObject const* obj, unsigned num_iter, XPBDResidualPolicy residual_policy)
    : XPBDSolver(obj, num_iter, residual_policy)
{}

void XPBDGaussSeidelSolver::_solveConstraints()
{
    const std::vector<std::unique_ptr<Constraint>>& constraints = _obj->constraints();
    // loop through constraints
    for (const auto& constraint : constraints)
    {
        // get the position updates for this constraint
        std::vector<Constraint::PositionUpdate> position_updates = constraint->project();
        for (const auto& position_update : position_updates)
        {
            // apply each position update
            const Eigen::Vector3d& dx = position_update.second;
            const PositionReference& p_ref = position_update.first;
            p_ref.obj->setVertex(p_ref.index, p_ref.position() + dx);
        }
    }
}

} // namespace Solver