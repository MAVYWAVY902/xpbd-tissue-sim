#include "simobject/FirstOrderXPBDMeshObject.hpp"
#include "solver/ConstraintProjectorDecorator.hpp"
#include "solver/StaticDeformableCollisionConstraint.hpp"
#include "solver/RigidDeformableCollisionConstraint.hpp"
#include "solver/RigidBodyConstraintProjector.hpp"
#include "solver/XPBDGaussSeidelSolver.hpp"
#include "simulation/Simulation.hpp"

namespace Sim
{

FirstOrderXPBDMeshObject::FirstOrderXPBDMeshObject(const Simulation* sim, const FirstOrderXPBDMeshObjectConfig* config)
    : XPBDMeshObject(sim, config)
{
    std::cout << "FirstOrderXPBDMeshObject constructor! " << std::endl;
    _damping_multiplier = config->dampingMultiplier().value();

    std::cout << "Damping Multiplier: " << _damping_multiplier << std::endl;
}

void FirstOrderXPBDMeshObject::_calculatePerVertexQuantities()
{
    XPBDMeshObject::_calculatePerVertexQuantities();

    _inv_B.resize(_mesh->numVertices());
    for (int i = 0; i < _mesh->numVertices(); i++)
    {
        _inv_B[i] = 1.0 / (_vertex_volumes[i] * _damping_multiplier);
    }
}

void FirstOrderXPBDMeshObject::setup()
{
    // TODO: lots of duplicated code here...

    _loadAndConfigureMesh();

    // initialize the previous vertices matrix once we've loaded the mesh
    _previous_vertices = _mesh->vertices();
    _vertex_velocities = Geometry::Mesh::VerticesMat::Zero(3, _mesh->numVertices());

    _calculatePerVertexQuantities();
    _createSolver(_solver_type, _num_solver_iters, _residual_policy);       // create the Solver object first and then add ConstraintProjectors to it
    _createConstraints(_constraint_type, _constraints_with_residual, _constraints_with_damping, true);     // create constraints and add ConstraintProjectors to the solver object
}

std::string FirstOrderXPBDMeshObject::toString(const int indent) const
{
    // TODO: better toString
    return XPBDMeshObject::toString(indent);
}

void FirstOrderXPBDMeshObject::_movePositionsInertially()
{
    for (int i = 0; i < _mesh->numVertices(); i++)
    {
        _mesh->displaceVertex(i, 0, 0, -_sim->gAccel() * _vertex_masses[i] * _sim->dt() * _inv_B[i]);
    }   
}

void FirstOrderXPBDMeshObject::addStaticCollisionConstraint(const Geometry::SDF* sdf, const Eigen::Vector3d& p, const Eigen::Vector3d& n,
                                    const XPBDMeshObject* obj, const int v1, const int v2, const int v3, const double u, const double v, const double w)
{
    std::unique_ptr<Solver::StaticDeformableCollisionConstraint> collision_constraint = std::make_unique<Solver::StaticDeformableCollisionConstraint>(sdf, p, n, obj, v1, v2, v3, u, v, w);
    std::vector<Solver::Constraint*> collision_vec; collision_vec.push_back(collision_constraint.get());
    std::unique_ptr<Solver::ConstraintProjector> collision_projector = std::make_unique<Solver::ConstraintProjector>(collision_vec, _sim->dt());

    // std::cout << "FirstOrder addStaticCollisionConstraint" << std::endl;
    int index = _solver->addConstraintProjector(_decorateConstraintProjector(std::move(collision_projector), false, false, true));

    XPBDCollisionConstraint xpbd_collision_constraint;
    xpbd_collision_constraint.constraint = std::move(collision_constraint);
    xpbd_collision_constraint.projector_index = index;
    xpbd_collision_constraint.num_steps_unused = 0;

    _collision_constraints.push_back(std::move(xpbd_collision_constraint));
}

void FirstOrderXPBDMeshObject::addRigidDeformableCollisionConstraint(const Geometry::SDF* sdf, Sim::RigidObject* rigid_obj, const Eigen::Vector3d& rigid_body_point, const Eigen::Vector3d& collision_normal,
                                       const Sim::XPBDMeshObject* deformable_obj, const int v1, const int v2, const int v3, const double u, const double v, const double w)
{
    std::unique_ptr<Solver::RigidDeformableCollisionConstraint> collision_constraint = std::make_unique<Solver::RigidDeformableCollisionConstraint>(sdf, rigid_obj, rigid_body_point, collision_normal, deformable_obj, v1, v2, v3, u, v, w);
    std::unique_ptr<Solver::ConstraintProjector> collision_projector = std::make_unique<Solver::RigidBodyConstraintProjector>(collision_constraint.get(), _sim->dt());

    // int index = _solver->addConstraintProjector(_decorateConstraintProjector(std::move(collision_projector), false, false, true));
    int index = _solver->addConstraintProjector(std::move(collision_projector));

    XPBDCollisionConstraint xpbd_collision_constraint;
    xpbd_collision_constraint.constraint = std::move(collision_constraint);
    xpbd_collision_constraint.projector_index = index;
    xpbd_collision_constraint.num_steps_unused = 0;

    _collision_constraints.push_back(std::move(xpbd_collision_constraint));
}

} //namespace Sim