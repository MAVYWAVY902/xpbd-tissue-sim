#include "solver/XPBDGaussSeidelSolver.hpp"
#include "solver/Constraint.hpp"
#include "solver/ConstraintProjector.hpp"
#include "solver/RigidBodyConstraintProjector.hpp"
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
        
        if (RigidBodyConstraintProjector* rb_proj = dynamic_cast<RigidBodyConstraintProjector*>(proj.get()))
        {
            rb_proj->project(data, _coordinate_updates.data(), _rigid_body_updates.data());
            const std::vector<Sim::RigidObject*>& rigid_bodies = rb_proj->rigidBodies();
            for (int i = 0; i < rb_proj->numRigidBodies(); i++)
            {
                double* rb_update = _rigid_body_updates.data() + 7*i;
                rigid_bodies[i]->setPosition(rigid_bodies[i]->position() + Eigen::Map<Eigen::Vector3d>(rb_update));
                rigid_bodies[i]->setOrientation(rigid_bodies[i]->orientation() + Eigen::Map<Eigen::Vector4d>(rb_update+3));
            }
            // Sim::RigidObject* rigid_obj = rdcc_proj->rigidObject();
            // rigid_obj->setPosition(rigid_obj->position() + rigid_body_update(Eigen::seq(0,2)));
            // rigid_obj->setOrientation(rigid_obj->orientation() + rigid_body_update(Eigen::seq(3,6)));
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