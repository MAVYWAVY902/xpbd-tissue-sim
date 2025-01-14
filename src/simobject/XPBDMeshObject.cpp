#include "simobject/XPBDMeshObject.hpp"
#include "simobject/RigidObject.hpp"
#include "simulation/Simulation.hpp"
#include "solver/XPBDGaussSeidelSolver.hpp"
#include "solver/StaticDeformableCollisionConstraint.hpp"
#include "solver/RigidDeformableCollisionConstraint.hpp"
#include "solver/RigidDeformableCollisionConstraintProjector.hpp"
#include "solver/HydrostaticConstraint.hpp"
#include "solver/DeviatoricConstraint.hpp"
#include "solver/ConstraintProjectorDecorator.hpp"
#include "solver/CombinedNeohookeanConstraintProjector.hpp"
#include "utils/MeshUtils.hpp"

namespace Sim
{

XPBDMeshObject::XPBDMeshObject(const Simulation* sim, const XPBDMeshObjectConfig* config)
    : Object(sim, config), TetMeshObject(config, config), _material(config->materialConfig())
{
    /* extract values from the Config object */
    
    // set initial velocity if specified in config
    _initial_velocity = config->initialVelocity();

    // solver options
    _solver_type = config->solverType().value();
    _num_solver_iters = config->numSolverIters().value();
    _residual_policy = config->residualPolicy().value();
    
    // constraint specifications
    _constraints_with_residual = config->withResidual().value();
    _constraints_with_damping = config->withDamping().value();
    _constraint_type = config->constraintType().value();

    _damping_gamma = config->dampingGamma().value();
}

XPBDMeshObject::~XPBDMeshObject()
{

}

Geometry::AABB XPBDMeshObject::boundingBox() const
{
    return _mesh->boundingBox();
}

void XPBDMeshObject::setup()
{
    _loadAndConfigureMesh();

    // initialize the previous vertices matrix once we've loaded the mesh
    _previous_vertices = _mesh->vertices();
    _vertex_velocities = Geometry::Mesh::VerticesMat::Zero(3, _mesh->numVertices());
    _vertex_velocities.colwise() = _initial_velocity;

    _calculatePerVertexQuantities();
    _createSolver(_solver_type, _num_solver_iters, _residual_policy);       // create the Solver object first and then add ConstraintProjectors to it
    _createConstraints(_constraint_type, _constraints_with_residual, _constraints_with_damping, false);     // create constraints and add ConstraintProjectors to the solver object
}

int XPBDMeshObject::numConstraintsForPosition(const int index) const
{
    if (_constraint_type == XPBDConstraintType::STABLE_NEOHOOKEAN)
    {
        return 2*_vertex_attached_elements[index];   // if sequential constraints are used, there are 2 constraints per element ==> # of constraint updates = 2 * # of elements attached to that vertex
    }
    else if (_constraint_type == XPBDConstraintType::STABLE_NEOHOOKEAN_COMBINED)
    {
        return _vertex_attached_elements[index];     // if combined constraints are used, there are 2 constraints per element but they are solved together ==> # of constraint updates = # of elements attached to that vertex
    }
    else
    {
        assert(0); // something weird happened, shouldn't get to here
        return 0;
    }
}

void XPBDMeshObject::addStaticCollisionConstraint(const Geometry::SDF* sdf, const Eigen::Vector3d& p, const Eigen::Vector3d& n,
                                    const XPBDMeshObject* obj, const int v1, const int v2, const int v3, const double u, const double v, const double w)
{
    std::unique_ptr<Solver::Constraint> collision_constraint = std::make_unique<Solver::StaticDeformableCollisionConstraint>(sdf, p, n, obj, v1, v2, v3, u, v, w);
    std::vector<Solver::Constraint*> collision_vec; collision_vec.push_back(collision_constraint.get());
    std::unique_ptr<Solver::ConstraintProjector> collision_projector = std::make_unique<Solver::ConstraintProjector>(collision_vec, _sim->dt());

    int index = _solver->addConstraintProjector(std::move(collision_projector));

    XPBDCollisionConstraint xpbd_collision_constraint;
    xpbd_collision_constraint.constraint = std::move(collision_constraint);
    xpbd_collision_constraint.projector_index = index;
    xpbd_collision_constraint.num_steps_unused = 0;

    _collision_constraints.push_back(std::move(xpbd_collision_constraint));
}

void XPBDMeshObject::addRigidDeformableCollisionConstraint(const Geometry::SDF* sdf, Sim::RigidObject* rigid_obj, const Eigen::Vector3d& rigid_body_point, const Eigen::Vector3d& collision_normal, double penetration_dist,
                                       const Sim::XPBDMeshObject* deformable_obj, const int v1, const int v2, const int v3, const double u, const double v, const double w)
{
    std::unique_ptr<Solver::RigidDeformableCollisionConstraint> collision_constraint = std::make_unique<Solver::RigidDeformableCollisionConstraint>(sdf, rigid_obj, rigid_body_point, collision_normal, penetration_dist, deformable_obj, v1, v2, v3, u, v, w);
    std::unique_ptr<Solver::ConstraintProjector> collision_projector = std::make_unique<Solver::RigidDeformableCollisionConstraintProjector>(collision_constraint.get(), _sim->dt());

    int index = _solver->addConstraintProjector(std::move(collision_projector));

    XPBDCollisionConstraint xpbd_collision_constraint;
    xpbd_collision_constraint.constraint = std::move(collision_constraint);
    xpbd_collision_constraint.projector_index = index;
    xpbd_collision_constraint.num_steps_unused = 0;

    _collision_constraints.push_back(std::move(xpbd_collision_constraint));
}

void XPBDMeshObject::clearCollisionConstraints()
{
    for (const auto& c : _collision_constraints)
    {
        _solver->removeConstraintProjector(c.projector_index);
    }
    _collision_constraints.clear();
}

void XPBDMeshObject::removeOldCollisionConstraints(const int threshold)
{
    for (int i = _collision_constraints.size() - 1; i >= 0; i--)
    {
        if (_collision_constraints[i].num_steps_unused >= threshold)
        {
            // std::cout << "OLD COLLISION CONSTRAINT REMOVED!" << std::endl;
            _solver->removeConstraintProjector(_collision_constraints[i].projector_index);
            _collision_constraints.erase(_collision_constraints.begin() + i);
        }
    }
}

void XPBDMeshObject::_calculatePerVertexQuantities()
{
    // calculate masses for each vertex
    _vertex_masses.resize(_mesh->numVertices());
    _vertex_inv_masses.resize(_mesh->numVertices());
    _vertex_volumes.resize(_mesh->numVertices());
    _vertex_attached_elements.resize(_mesh->numVertices());
    for (int i = 0; i < tetMesh()->numElements(); i++)
    {
        const Eigen::Vector4i& element = tetMesh()->element(i);
        // compute volume from X
        const double volume = tetMesh()->elementVolume(i);
        // _vols(i) = vol;

        // compute mass of element
        const double element_mass = volume * _material.density();
        // add mass contribution of element to each of its vertices
        _vertex_masses[element[0]] += element_mass/4.0;
        _vertex_masses[element[1]] += element_mass/4.0;
        _vertex_masses[element[2]] += element_mass/4.0;
        _vertex_masses[element[3]] += element_mass/4.0;

        // add volume contribution of element to each of its vertices
        _vertex_volumes[element[0]] += volume/4.0;
        _vertex_volumes[element[1]] += volume/4.0;
        _vertex_volumes[element[2]] += volume/4.0;
        _vertex_volumes[element[3]] += volume/4.0;

        // increment number of elements attached to each vertex in this element
        _vertex_attached_elements[element[0]]++;
        _vertex_attached_elements[element[1]]++;
        _vertex_attached_elements[element[2]]++;
        _vertex_attached_elements[element[3]]++;
    }

    // calculate inverse masses
    for (int i = 0; i < _mesh->numVertices(); i++)
    {
        _vertex_inv_masses[i] = 1.0 / _vertex_masses[i];
    } 
}

void XPBDMeshObject::_createConstraints(XPBDConstraintType constraint_type, bool with_residual, bool with_damping, bool first_order)
{
    // create constraint(s) for each element
    for (int i = 0; i < tetMesh()->numElements(); i++)
    {
        const Eigen::Vector4i element = tetMesh()->element(i);
        const int v0 = element[0];
        const int v1 = element[1];
        const int v2 = element[2];
        const int v3 = element[3];
        if (constraint_type == XPBDConstraintType::STABLE_NEOHOOKEAN)
        {
            std::unique_ptr<Solver::Constraint> hyd_constraint, dev_constraint;
            hyd_constraint = std::make_unique<Solver::HydrostaticConstraint>(this, v0, v1, v2, v3);
            dev_constraint = std::make_unique<Solver::DeviatoricConstraint>(this, v0, v1, v2, v3);

            std::vector<Solver::Constraint*> hyd_vec; hyd_vec.push_back(hyd_constraint.get());
            std::unique_ptr<Solver::ConstraintProjector> hyd_projector = std::make_unique<Solver::ConstraintProjector>(hyd_vec, _sim->dt());
            std::vector<Solver::Constraint*> dev_vec; dev_vec.push_back(dev_constraint.get());
            std::unique_ptr<Solver::ConstraintProjector> dev_projector = std::make_unique<Solver::ConstraintProjector>(dev_vec, _sim->dt());

            _elastic_constraints.push_back(std::move(dev_constraint));
            _elastic_constraints.push_back(std::move(hyd_constraint));

            _solver->addConstraintProjector(_decorateConstraintProjector(std::move(dev_projector), with_residual, with_damping, first_order));
            _solver->addConstraintProjector(_decorateConstraintProjector(std::move(hyd_projector), with_residual, with_damping, first_order));
            
        }
        else if (constraint_type == XPBDConstraintType::STABLE_NEOHOOKEAN_COMBINED)
        {
            std::unique_ptr<Solver::Constraint> hyd_constraint, dev_constraint;
            hyd_constraint = std::make_unique<Solver::HydrostaticConstraint>(this, v0, v1, v2, v3);
            dev_constraint = std::make_unique<Solver::DeviatoricConstraint>(this, v0, v1, v2, v3);
            
            std::vector<Solver::Constraint*> vec; vec.push_back(dev_constraint.get()); vec.push_back(hyd_constraint.get());
            std::unique_ptr<Solver::CombinedNeohookeanConstraintProjector> projector = std::make_unique<Solver::CombinedNeohookeanConstraintProjector>(vec, _sim->dt());

            _elastic_constraints.push_back(std::move(dev_constraint));
            _elastic_constraints.push_back(std::move(hyd_constraint));
            
            _solver->addConstraintProjector( _decorateConstraintProjector(std::move(projector), with_residual, with_damping, first_order));
        }
    }
}

std::unique_ptr<Solver::ConstraintProjector> XPBDMeshObject::_decorateConstraintProjector(std::unique_ptr<Solver::ConstraintProjector> projector, bool with_residual, bool with_damping, bool first_order)
{
    if (with_damping)
    {
        projector = std::make_unique<Solver::WithDamping>(std::move(projector), _damping_gamma);        // wrap the ConstraintProjector with the WithDamping decorator
    }

    if (first_order)
    {
        projector = std::make_unique<Solver::FirstOrder>(std::move(projector));                         // wrap the ConstraintProjector with the FirstOrder decorator
    }

    // IMPORTANT: the WithDistributedPrimaryResidual should be applied last so that XPBDSolver can downcast and use the setPrimaryResidual() method.
    // if, for example, the WithDisributedPrimaryResidual is wrapped with a FirstOrder decorator, a downcast to WithDistributedPrimaryResidual will fail,
    //  and there won't be any way to point the WithDistributedPrimaryResidual decorator to the primary residual vector of the XPBDSolver, resulting in a segfault.
    // this is probably indicative of bad design and needs some rethinking, but it's okay for now
    if (with_residual)
    {
        projector = std::make_unique<Solver::WithDistributedPrimaryResidual>(std::move(projector));     // wrap the ConstraintProjector with the WithDistributedPRimaryResidual decorator
    }

    return projector;
}

void XPBDMeshObject::_createSolver(XPBDSolverType solver_type, int num_solver_iters, XPBDResidualPolicy residual_policy)
{
    if (solver_type == XPBDSolverType::GAUSS_SEIDEL)
    {
        _solver = std::make_unique<Solver::XPBDGaussSeidelSolver>(this, num_solver_iters, residual_policy);
    }
    else if (solver_type == XPBDSolverType::JACOBI)
    {
        std::cout << "\nJacobi solver not implemented yet!" << std::endl;
        assert(0);
    }
}

std::string XPBDMeshObject::toString(const int indent) const
{
    // TODO: complete toString
    return Object::toString(indent+1);
}

void XPBDMeshObject::update()
{
    // set _x_prev to be ready for the next substep
    _previous_vertices = _mesh->vertices();

    _movePositionsInertially();
    _projectConstraints();
}

void XPBDMeshObject::_movePositionsInertially()
{
    const double dt = _sim->dt();
    // move vertices according to their velocity
    _mesh->moveSeparate(dt*_vertex_velocities);
    // external forces (right now just gravity, which acts in -z direction)
    for (int i = 0; i < _mesh->numVertices(); i++)
    {
        const double dz = -_sim->gAccel() * dt * dt;
        _mesh->displaceVertex(i, 0, 0, dz);
    }
}

void XPBDMeshObject::_projectConstraints()
{
    _solver->solve();

    // update collision constraints unused
    for (auto& c : _collision_constraints)
    {
        if (_solver->constraintProjectors()[c.projector_index]->lambda()[0] != 0)
        {
            c.num_steps_unused = 0;
        }
        else
        {
            c.num_steps_unused++;
        }
    }

    // TODO: replace with real collision detection
    for (int i = 0; i < _mesh->numVertices(); i++)
    {
        // if (vertexFixed(i))
        // {
        //     _vertices.row(i) = _x_prev.row(i);
        // }
        
        // const Eigen::Vector3d& v = _mesh->vertex(i);
        // if (v[2] <= 0)
        // {
        //     _mesh->displaceVertex(i, 0, 0, -v[2]);
        // }
    }
}

void XPBDMeshObject::velocityUpdate()
{
    const Geometry::Mesh::VerticesMat& cur_vertices = _mesh->vertices();
    // velocities are simply (cur_pos - last_pos) / deltaT
    _vertex_velocities = (cur_vertices - _previous_vertices) / _sim->dt();
}

} // namespace Sim