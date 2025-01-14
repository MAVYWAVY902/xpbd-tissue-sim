#include "solver/XPBDGaussSeidelSolver.hpp"
#include "solver/Constraint.hpp"
#include "solver/ConstraintProjector.hpp"
#include "solver/RigidDeformableCollisionConstraintProjector.hpp"
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
        
        if (RigidDeformableCollisionConstraintProjector* rdcc_proj = dynamic_cast<RigidDeformableCollisionConstraintProjector*>(proj.get()))
        {
            Eigen::Vector<double, 7> rigid_body_update;
            rdcc_proj->project(data, _coordinate_updates.data(), rigid_body_update);
            Sim::RigidObject* rigid_obj = rdcc_proj->rigidObject();
            rigid_obj->setPosition(rigid_obj->position() + rigid_body_update(Eigen::seq(0,2)));
            rigid_obj->setOrientation(rigid_obj->orientation() + rigid_body_update(Eigen::seq(3,6)));
        }
        else
        {
            // get the position updates for this constraint - they are put in the _coordinate_updates data block
            proj->project(data, _coordinate_updates.data());
        }

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