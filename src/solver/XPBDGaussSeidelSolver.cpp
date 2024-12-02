#include "solver/XPBDGaussSeidelSolver.hpp"
#include "solver/Constraint.hpp"
#include "solver/ConstraintProjector.hpp"
#include "simobject/XPBDMeshObject.hpp"

namespace Solver
{

XPBDGaussSeidelSolver::XPBDGaussSeidelSolver(XPBDMeshObject const* obj, unsigned num_iter, XPBDResidualPolicy residual_policy)
    : XPBDSolver(obj, num_iter, residual_policy)
{}

void XPBDGaussSeidelSolver::_solveConstraints(double* data)
{
    // const std::vector<std::unique_ptr<ConstraintProjector>>& projectors = _obj->constraintProjectors();
    // loop through constraints
    //for (const auto& proj : projectors)
    for (const auto& proj : _constraint_projectors)
    {
        // get the position updates for this constraint
        proj->project(data, _coordinate_updates.data());
        const std::vector<PositionReference>& positions = proj->positions();
        for (unsigned i = 0; i < proj->numPositions(); i++)
        {
            const PositionReference& p_ref = positions[i];
            // p_ref.obj->displaceVertex(p_ref.index, _coordinate_updates[3*i], _coordinate_updates[3*i+1], _coordinate_updates[3*i+2]);
            p_ref.position_ptr[0] += _coordinate_updates[3*i];
            p_ref.position_ptr[1] += _coordinate_updates[3*i+1];
            p_ref.position_ptr[2] += _coordinate_updates[3*i+2];
        }
    }
}

} // namespace Solver