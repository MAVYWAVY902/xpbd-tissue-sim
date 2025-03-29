#include "solver/XPBDJacobiSolver.hpp"
#include "solver/Constraint.hpp"
#include "solver/ConstraintProjector.hpp"
#include "solver/RigidBodyConstraintProjector.hpp"
#include "simobject/XPBDMeshObject.hpp"

namespace Solver
{
XPBDJacobiSolver::XPBDJacobiSolver(Sim::XPBDMeshObject* obj, int num_iter, XPBDResidualPolicy residual_policy)
    : XPBDSolver(obj, num_iter, residual_policy)
{
    _position_updates.conservativeResize(3, obj->mesh()->numVertices());
}

void XPBDJacobiSolver::_solveConstraints(Real* data)
{
    _position_updates = Geometry::Mesh::VerticesMat::Zero(3, _obj->mesh()->numVertices());
    for (const auto& proj : _constraint_projectors)
    {
        if (!proj)
            continue;
        
        // if the constraint projector is projecting a constraint that involves a rigid body, we need to handle the rigid body updates specially
        if (RigidBodyConstraintProjector* rb_proj = dynamic_cast<RigidBodyConstraintProjector*>(proj.get()))
        {
            // get the deformable position updates for this constraint - they are put in the _coordinate_updates data block
            // this will also get the rigid body position + orientation updates for this constraint - they are put in the _rigid_body_updates data block
            rb_proj->project(data, _coordinate_updates.data(), _rigid_body_updates.data());
            const std::vector<Sim::RigidObject*>& rigid_bodies = rb_proj->rigidBodies();
            // apply the rigid body updates
            for (int i = 0; i < rb_proj->numRigidBodies(); i++)
            {
                Real* rb_update = _rigid_body_updates.data() + 7*i;
                rigid_bodies[i]->setPosition(rigid_bodies[i]->position() + Eigen::Map<Vec3r>(rb_update));
                rigid_bodies[i]->setOrientation(rigid_bodies[i]->orientation() + Eigen::Map<Vec4r>(rb_update+3));
            }
        }
        else
        {
            // get the position updates for this constraint - they are put in the _coordinate_updates data block
            proj->project(data, _coordinate_updates.data());
        }

        // store the deformable position updates
        const std::vector<PositionReference>& positions = proj->positions();
        for (int i = 0; i < proj->numPositions(); i++)
        {
            const PositionReference& p_ref = positions[i];

            _position_updates.col(p_ref.index) += Eigen::Map<Vec3r>(_coordinate_updates.data() + 3*i);

            // p_ref.position_ptr[0] += _coordinate_updates[3*i];
            // p_ref.position_ptr[1] += _coordinate_updates[3*i+1];
            // p_ref.position_ptr[2] += _coordinate_updates[3*i+2];
        }
    }

    // apply position updates after projecting all constraints
    for (int i = 0; i < _obj->mesh()->numVertices(); i++)
    {
        const Vec3r& pos_update = _position_updates.col(i);
        _obj->mesh()->displaceVertex(i, pos_update);
    }
}


} // namespace Solver