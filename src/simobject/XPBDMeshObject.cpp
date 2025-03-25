#include "simobject/XPBDMeshObject.hpp"
#include "simobject/RigidObject.hpp"
#include "simulation/Simulation.hpp"

#include "solver/XPBDGaussSeidelSolver.hpp"
#include "solver/XPBDJacobiSolver.hpp"
#include "solver/XPBDParallelJacobiSolver.hpp"
#include "solver/StaticDeformableCollisionConstraint.hpp"
#include "solver/RigidDeformableCollisionConstraint.hpp"
#include "solver/RigidBodyConstraintProjector.hpp"
#include "solver/HydrostaticConstraint.hpp"
#include "solver/DeviatoricConstraint.hpp"
// #include "solver/ConstraintProjectorDecorator.hpp"
// #include "solver/CombinedNeohookeanConstraintProjector.hpp"
#include "solver/CombinedConstraintProjector.hpp"
#include "solver/ConstraintProjector.hpp"
#include "utils/MeshUtils.hpp"

#ifdef HAVE_CUDA
#include "gpu/resource/XPBDMeshObjectGPUResource.hpp"
#endif

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

void XPBDMeshObject::addStaticCollisionConstraint(const Geometry::SDF* sdf, const Vec3r& p, const Vec3r& n,
                                    const XPBDMeshObject* obj, const int v1, const int v2, const int v3, const Real u, const Real v, const Real w)
{
    Solver::StaticDeformableCollisionConstraint& collision_constraint = _constraints.template emplace_back<Solver::StaticDeformableCollisionConstraint>(sdf, p, n, obj, v1, v2, v3, u, v, w);

    Solver::ConstraintProjectorOptions projector_options;
    _solver->addConstraintProjector(_sim->dt(), projector_options, &collision_constraint); // TODO: accomodate for first-order method

    // XPBDCollisionConstraint xpbd_collision_constraint;
    // xpbd_collision_constraint.constraint = std::move(collision_constraint);
    // xpbd_collision_constraint.projector_index = index;
    // xpbd_collision_constraint.num_steps_unused = 0;

    // _collision_constraints.push_back(std::move(xpbd_collision_constraint));
}

void XPBDMeshObject::addRigidDeformableCollisionConstraint(const Geometry::SDF* sdf, Sim::RigidObject* rigid_obj, const Vec3r& rigid_body_point, const Vec3r& collision_normal,
                                       const Sim::XPBDMeshObject* deformable_obj, const int v1, const int v2, const int v3, const Real u, const Real v, const Real w)
{
    // std::unique_ptr<Solver::RigidDeformableCollisionConstraint> collision_constraint = std::make_unique<Solver::RigidDeformableCollisionConstraint>(sdf, rigid_obj, rigid_body_point, collision_normal, deformable_obj, v1, v2, v3, u, v, w);
    // std::unique_ptr<Solver::ConstraintProjector> collision_projector = std::make_unique<Solver::RigidBodyConstraintProjector>(collision_constraint.get(), _sim->dt());

    Solver::RigidDeformableCollisionConstraint& collision_constraint = _constraints.template emplace_back<Solver::RigidDeformableCollisionConstraint>(sdf, rigid_obj, rigid_body_point, collision_normal, deformable_obj, v1, v2, v3, u, v, w);

    Solver::ConstraintProjectorOptions projector_options;
    _solver->addConstraintProjector(_sim->dt(), projector_options, &collision_constraint); // TODO: accomodate for first-order method

    // XPBDCollisionConstraint xpbd_collision_constraint;
    // xpbd_collision_constraint.constraint = std::move(collision_constraint);
    // xpbd_collision_constraint.projector_index = index;
    // xpbd_collision_constraint.num_steps_unused = 0;

    // _collision_constraints.push_back(std::move(xpbd_collision_constraint));
}

void XPBDMeshObject::clearCollisionConstraints()
{
    // TODO: implement
    
}

void XPBDMeshObject::removeOldCollisionConstraints(const int threshold)
{
    // TODO: implement (actually I don't think this is used, but it might be in the future)
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
        const Real volume = tetMesh()->elementVolume(i);
        // _vols(i) = vol;

        // compute mass of element
        const Real element_mass = volume * _material.density();
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
    // TODO: think about this... we need to resize each vector initially so that pointers to constraints are still valid...
    // alternative: use vector unique_ptr<Constraint> 
    _constraints.template reserve<Solver::HydrostaticConstraint>(tetMesh()->numElements());
    _constraints.template reserve<Solver::DeviatoricConstraint>(tetMesh()->numElements());
    _constraints.template reserve<Solver::StaticDeformableCollisionConstraint>(_mesh->numFaces());
    _constraints.template reserve<Solver::RigidDeformableCollisionConstraint>(_mesh->numFaces());

    Solver::ConstraintProjectorOptions projector_options;
    projector_options.with_residual = with_residual;
    projector_options.with_damping = with_damping;
    projector_options.first_order = first_order;
    projector_options.damping_gamma = _damping_gamma;

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
            Solver::HydrostaticConstraint& hyd_constraint = _constraints.template emplace_back<Solver::HydrostaticConstraint>(this, v0, v1, v2, v3);
            Solver::DeviatoricConstraint& dev_constraint = _constraints.template emplace_back<Solver::DeviatoricConstraint>(this, v0, v1, v2, v3);
            
            // TODO: support separate constraints - maybe though SeparateConstraintProjector class? For now, just have them be together.
            // _solver->addConstraintProjector(_sim->dt(), projector_options, &hyd_constraint);
            // _solver->addConstraintProjector(_sim->dt(), projector_options, &dev_constraint);
            _solver->addConstraintProjector(_sim->dt(), projector_options, &dev_constraint, &hyd_constraint);
            
        }
        else if (constraint_type == XPBDConstraintType::STABLE_NEOHOOKEAN_COMBINED)
        {
            Solver::HydrostaticConstraint& hyd_constraint = _constraints.template emplace_back<Solver::HydrostaticConstraint>(this, v0, v1, v2, v3);
            Solver::DeviatoricConstraint& dev_constraint = _constraints.template emplace_back<Solver::DeviatoricConstraint>(this, v0, v1, v2, v3);

            _solver->addConstraintProjector(_sim->dt(), projector_options, &dev_constraint, &hyd_constraint);
        }
    }
}

void XPBDMeshObject::_createSolver(XPBDSolverType solver_type, int num_solver_iters, XPBDResidualPolicy residual_policy)
{
    if (solver_type == XPBDSolverType::GAUSS_SEIDEL)
    {
        typedef Solver::XPBDGaussSeidelSolver<
        Solver::CombinedConstraintProjector<Solver::DeviatoricConstraint, Solver::HydrostaticConstraint>,
            Solver::ConstraintProjector<Solver::StaticDeformableCollisionConstraint>,
            Solver::ConstraintProjector<Solver::RigidDeformableCollisionConstraint>
        > SolverType;

        _solver = std::make_unique<SolverType>(this, num_solver_iters, residual_policy);
    }
    else if (solver_type == XPBDSolverType::JACOBI)
    {
        typedef Solver::XPBDJacobiSolver<
            Solver::CombinedConstraintProjector<Solver::DeviatoricConstraint, Solver::HydrostaticConstraint>,
            Solver::ConstraintProjector<Solver::StaticDeformableCollisionConstraint>,
            Solver::ConstraintProjector<Solver::RigidDeformableCollisionConstraint>
        > SolverType;

        _solver = std::make_unique<SolverType>(this, num_solver_iters, residual_policy);
    }
    else if (solver_type == XPBDSolverType::PARALLEL_JACOBI)
    {
        typedef Solver::XPBDParallelJacobiSolver<
            Solver::CombinedConstraintProjector<Solver::DeviatoricConstraint, Solver::HydrostaticConstraint>,
            Solver::ConstraintProjector<Solver::StaticDeformableCollisionConstraint>,
            Solver::ConstraintProjector<Solver::RigidDeformableCollisionConstraint>
        > SolverType;
        _solver = std::make_unique<SolverType>(this, num_solver_iters, residual_policy);

        // _solver = std::make_unique<Solver::XPBDParallelJacobiSolver>(this, num_solver_iters, residual_policy);
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
    const Real dt = _sim->dt();
    // move vertices according to their velocity
    _mesh->moveSeparate(dt*_vertex_velocities);
    // external forces (right now just gravity, which acts in -z direction)
    for (int i = 0; i < _mesh->numVertices(); i++)
    {
        const Real dz = -_sim->gAccel() * dt * dt;
        _mesh->displaceVertex(i, Vec3r(0, 0, dz));
    }
}

void XPBDMeshObject::_projectConstraints()
{
    _solver->solve();

    // TODO: update collision constraints unused
    // for (auto& c : _collision_constraints)
    // {
    //     if (_solver->constraintProjectors()[c.projector_index]->lambda()[0] != 0)
    //     {
    //         c.num_steps_unused = 0;
    //     }
    //     else
    //     {
    //         c.num_steps_unused++;
    //     }
    // }

    // TODO: remove
    for (int i = 0; i < _mesh->numVertices(); i++)
    {
        const Vec3r& v = _mesh->vertex(i);
        if (v[2] < 0)
        {
            _mesh->setVertex(i, Vec3r(v[0], v[1], 0));
        }
    }

}

void XPBDMeshObject::velocityUpdate()
{
    // TODO: apply frictional forces
    // we do this in the velocity update (i.e. after update() is finished) to ensure that all objects have had their constraints projected already
    // for (const auto& c : _collision_constraints)
    // {
    //     const Real lam = _solver->constraintProjectors()[c.projector_index]->lambda()[0];
    //     // only apply friction forces for this constraint if it was active (i.e. lambda > 0)
    //     // if it was "inactive", there was no penetration and thus no contact and thus no friction
    //     if (lam > 0)
    //     {
    //         c.constraint->applyFriction(lam, _material.muS(), _material.muK());
    //     }
    // }

    const Geometry::Mesh::VerticesMat& cur_vertices = _mesh->vertices();
    // velocities are simply (cur_pos - last_pos) / deltaT
    _vertex_velocities = (cur_vertices - _previous_vertices) / _sim->dt();
}

#ifdef HAVE_CUDA
void XPBDMeshObject::createGPUResource()
{
    _gpu_resource = std::make_unique<Sim::XPBDMeshObjectGPUResource>(this);
    _gpu_resource->allocate();
}

XPBDMeshObjectGPUResource* XPBDMeshObject::gpuResource()
{
    assert(_gpu_resource);
    // TODO: see if we can remove this dynamic_cast somehow
    return dynamic_cast<XPBDMeshObjectGPUResource*>(_gpu_resource.get());
}

const XPBDMeshObjectGPUResource* XPBDMeshObject::gpuResource() const
{
    assert(_gpu_resource);
    // TODO: see if we can remove this dynamic_cast somehow
    return dynamic_cast<const XPBDMeshObjectGPUResource*>(_gpu_resource.get());
}
#endif

} // namespace Sim