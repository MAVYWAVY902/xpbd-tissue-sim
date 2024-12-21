#include "solver/XPBDGaussSeidelSolver.hpp"
#include "solver/Constraint.hpp"
#include "solver/ConstraintProjector.hpp"
#include "simobject/XPBDMeshObject.hpp"

#include <chrono>

namespace Solver
{

XPBDGaussSeidelSolver::XPBDGaussSeidelSolver(const Sim::XPBDMeshObject* obj, int num_iter, XPBDResidualPolicy residual_policy)
    : XPBDSolver(obj, num_iter, residual_policy)
{}

void XPBDGaussSeidelSolver::_solveConstraints(double* data)
{
    for (const auto& proj : _constraint_projectors)
    {
        if (!proj)
            continue;
            
        // get the position updates for this constraint - they are put in the _coordinate_updates data block
        proj->project(data, _coordinate_updates.data());
        const std::vector<PositionReference>& positions = proj->positions();
        for (int i = 0; i < proj->numPositions(); i++)
        {
            const PositionReference& p_ref = positions[i];

            p_ref.position_ptr[0] += _coordinate_updates[3*i];
            p_ref.position_ptr[1] += _coordinate_updates[3*i+1];
            p_ref.position_ptr[2] += _coordinate_updates[3*i+2];
        }
    }
}

} // namespace Solver