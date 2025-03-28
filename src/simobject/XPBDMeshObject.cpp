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

XPBDMeshObject_Base::XPBDMeshObject_Base(const Simulation* sim, const XPBDMeshObjectConfig* config)
    : Object(sim, config), TetMeshObject(config, config)
{}

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

template<typename SolverType, typename ...ConstraintTypes>
XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::XPBDMeshObject(TypeList<ConstraintTypes...>, const Simulation* sim, const XPBDMeshObjectConfig* config)
    : XPBDMeshObject_Base(sim, config), _material(config->materialConfig()),
        _solver(this, config->numSolverIters().value(), config->residualPolicy().value())
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
    _constraint_type = config->constraintType();

    _damping_gamma = config->dampingGamma().value();
}

template<typename SolverType, typename... ConstraintTypes>
XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::~XPBDMeshObject()
{

}

template<typename SolverType, typename... ConstraintTypes>
Geometry::AABB XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::boundingBox() const
{
    return _mesh->boundingBox();
}

template<typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::setup()
{
    _loadAndConfigureMesh();

    _solver.setup();

    // initialize the previous vertices matrix once we've loaded the mesh
    _previous_vertices = _mesh->vertices();
    _vertex_velocities = Geometry::Mesh::VerticesMat::Zero(3, _mesh->numVertices());
    _vertex_velocities.colwise() = _initial_velocity;

    _calculatePerVertexQuantities();
    _createConstraints(_constraint_type, _constraints_with_residual, _constraints_with_damping, false);     // create constraints and add ConstraintProjectors to the solver object
}

template<typename SolverType, typename... ConstraintTypes>
int XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::numConstraintsForPosition(const int index) const
{
    if (std::holds_alternative<XPBDMeshObjectConstraintTypes::StableNeohookean>(_constraint_type))
    {
        return 2*_vertex_attached_elements[index];   // if sequential constraints are used, there are 2 constraints per element ==> # of constraint updates = 2 * # of elements attached to that vertex
    }
    else if (std::holds_alternative<XPBDMeshObjectConstraintTypes::StableNeohookeanCombined>(_constraint_type))
    {
        return _vertex_attached_elements[index];     // if combined constraints are used, there are 2 constraints per element but they are solved together ==> # of constraint updates = # of elements attached to that vertex
    }
    else
    {
        assert(0); // something weird happened, shouldn't get to here
        return 0;
    }
}

template<typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::addStaticCollisionConstraint(const Geometry::SDF* sdf, const Vec3r& p, const Vec3r& n,
                                    const XPBDMeshObject_Base* obj, const int v1, const int v2, const int v3, const Real u, const Real v, const Real w)
{
    Real* v1_ptr = obj->mesh()->vertexPointer(v1);
    Real* v2_ptr = obj->mesh()->vertexPointer(v2);
    Real* v3_ptr = obj->mesh()->vertexPointer(v3);

    Real m1 = obj->vertexMass(v1);
    Real m2 = obj->vertexMass(v2);
    Real m3 = obj->vertexMass(v3);
    Solver::StaticDeformableCollisionConstraint& collision_constraint = 
        _constraints.template emplace_back<Solver::StaticDeformableCollisionConstraint>(sdf, p, n, v1, v1_ptr, m1, v2, v2_ptr, m2, v3, v3_ptr, m3, u, v, w);

    Solver::ConstraintProjectorOptions projector_options;
    _solver.addConstraintProjector(_sim->dt(), projector_options, &collision_constraint); // TODO: accomodate for first-order method

    // XPBDCollisionConstraint xpbd_collision_constraint;
    // xpbd_collision_constraint.constraint = std::move(collision_constraint);
    // xpbd_collision_constraint.projector_index = index;
    // xpbd_collision_constraint.num_steps_unused = 0;

    // _collision_constraints.push_back(std::move(xpbd_collision_constraint));
}

template<typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::addRigidDeformableCollisionConstraint(const Geometry::SDF* sdf, Sim::RigidObject* rigid_obj, const Vec3r& rigid_body_point, const Vec3r& collision_normal,
                                       const Sim::XPBDMeshObject_Base* deformable_obj, const int v1, const int v2, const int v3, const Real u, const Real v, const Real w)
{
    // std::unique_ptr<Solver::RigidDeformableCollisionConstraint> collision_constraint = std::make_unique<Solver::RigidDeformableCollisionConstraint>(sdf, rigid_obj, rigid_body_point, collision_normal, deformable_obj, v1, v2, v3, u, v, w);
    // std::unique_ptr<Solver::ConstraintProjector> collision_projector = std::make_unique<Solver::RigidBodyConstraintProjector>(collision_constraint.get(), _sim->dt());
    Real* v1_ptr = deformable_obj->mesh()->vertexPointer(v1);
    Real* v2_ptr = deformable_obj->mesh()->vertexPointer(v2);
    Real* v3_ptr = deformable_obj->mesh()->vertexPointer(v3);

    Real m1 = deformable_obj->vertexMass(v1);
    Real m2 = deformable_obj->vertexMass(v2);
    Real m3 = deformable_obj->vertexMass(v3);

    Solver::RigidDeformableCollisionConstraint& collision_constraint = 
        _constraints.template emplace_back<Solver::RigidDeformableCollisionConstraint>(sdf, rigid_obj, rigid_body_point, collision_normal, v1, v1_ptr, m1, v2, v2_ptr, m2, v3, v3_ptr, m3, u, v, w);

    Solver::ConstraintProjectorOptions projector_options;
    _solver.addConstraintProjector(_sim->dt(), projector_options, &collision_constraint); // TODO: accomodate for first-order method

    // XPBDCollisionConstraint xpbd_collision_constraint;
    // xpbd_collision_constraint.constraint = std::move(collision_constraint);
    // xpbd_collision_constraint.projector_index = index;
    // xpbd_collision_constraint.num_steps_unused = 0;

    // _collision_constraints.push_back(std::move(xpbd_collision_constraint));
}

template<typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::clearCollisionConstraints()
{
    // TODO: implement
    
}

template<typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::removeOldCollisionConstraints(const int /*threshold*/)
{
    // TODO: implement (actually I don't think this is used, but it might be in the future)
}

template<typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::_calculatePerVertexQuantities()
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

template<typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::_createConstraints(XPBDMeshObjectConstraintTypes::variant_type /*constraint_type*/, bool with_residual, bool with_damping, bool first_order)
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

        Real* v0_ptr = _mesh->vertexPointer(v0);
        Real* v1_ptr = _mesh->vertexPointer(v1);
        Real* v2_ptr = _mesh->vertexPointer(v2);
        Real* v3_ptr = _mesh->vertexPointer(v3);

        Real m0 = vertexMass(v0);
        Real m1 = vertexMass(v1);
        Real m2 = vertexMass(v2);
        Real m3 = vertexMass(v3);
        // if (std::holds_alternative<XPBDMeshObjectConstraintTypes::StableNeohookean>(_constraint_type))
        if constexpr (std::is_same_v<typename SolverType::projector_type_list, XPBDMeshObjectConstraintTypes::StableNeohookean::projector_type_list>)
        {
            Solver::HydrostaticConstraint& hyd_constraint = 
                _constraints.template emplace_back<Solver::HydrostaticConstraint>(v0, v0_ptr, m0, v1, v1_ptr, m1, v2, v2_ptr, m2, v3, v3_ptr, m3, _material);
            Solver::DeviatoricConstraint& dev_constraint = 
                _constraints.template emplace_back<Solver::DeviatoricConstraint>(v0, v0_ptr, m0, v1, v1_ptr, m1, v2, v2_ptr, m2, v3, v3_ptr, m3, _material);
            
            // TODO: support separate constraints - maybe though SeparateConstraintProjector class?.
            _solver.addConstraintProjector(_sim->dt(), projector_options, &hyd_constraint);
            _solver.addConstraintProjector(_sim->dt(), projector_options, &dev_constraint);
            // _solver.addConstraintProjector(_sim->dt(), projector_options, &dev_constraint, &hyd_constraint);
            
        }
        // else if (std::holds_alternative<XPBDMeshObjectConstraintTypes::StableNeohookeanCombined>(_constraint_type))
        if constexpr (std::is_same_v<typename SolverType::projector_type_list, XPBDMeshObjectConstraintTypes::StableNeohookeanCombined::projector_type_list>)
        {
            Solver::HydrostaticConstraint& hyd_constraint = 
                _constraints.template emplace_back<Solver::HydrostaticConstraint>(v0, v0_ptr, m0, v1, v1_ptr, m1, v2, v2_ptr, m2, v3, v3_ptr, m3, _material);
            Solver::DeviatoricConstraint& dev_constraint = 
                _constraints.template emplace_back<Solver::DeviatoricConstraint>(v0, v0_ptr, m0, v1, v1_ptr, m1, v2, v2_ptr, m2, v3, v3_ptr, m3, _material);

            _solver.addConstraintProjector(_sim->dt(), projector_options, &dev_constraint, &hyd_constraint);
        }
    }
}

template<typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::_createSolver(XPBDSolverType /*solver_type*/, int /*num_solver_iters*/, XPBDResidualPolicy /*residual_policy*/)
{
    // if (solver_type == XPBDSolverType::GAUSS_SEIDEL)
    // {
    //     typedef Solver::XPBDGaussSeidelSolver<
    //     Solver::CombinedConstraintProjector<Solver::DeviatoricConstraint, Solver::HydrostaticConstraint>,
    //         Solver::ConstraintProjector<Solver::StaticDeformableCollisionConstraint>,
    //         Solver::ConstraintProjector<Solver::RigidDeformableCollisionConstraint>
    //     > SolverType;

    //     _solver = std::make_unique<SolverType>(this, num_solver_iters, residual_policy);
    // }
    // else if (solver_type == XPBDSolverType::JACOBI)
    // {
    //     typedef Solver::XPBDJacobiSolver<
    //         Solver::CombinedConstraintProjector<Solver::DeviatoricConstraint, Solver::HydrostaticConstraint>,
    //         Solver::ConstraintProjector<Solver::StaticDeformableCollisionConstraint>,
    //         Solver::ConstraintProjector<Solver::RigidDeformableCollisionConstraint>
    //     > SolverType;

    //     _solver = std::make_unique<SolverType>(this, num_solver_iters, residual_policy);
    // }
    // else if (solver_type == XPBDSolverType::PARALLEL_JACOBI)
    // {
    //     typedef Solver::XPBDParallelJacobiSolver<
    //         Solver::CombinedConstraintProjector<Solver::DeviatoricConstraint, Solver::HydrostaticConstraint>,
    //         Solver::ConstraintProjector<Solver::StaticDeformableCollisionConstraint>,
    //         Solver::ConstraintProjector<Solver::RigidDeformableCollisionConstraint>
    //     > SolverType;
    //     _solver = std::make_unique<SolverType>(this, num_solver_iters, residual_policy);

    //     // _solver = std::make_unique<Solver::XPBDParallelJacobiSolver>(this, num_solver_iters, residual_policy);
    // }
    // _solver = SolverType(this, num_solver_iters, residual_policy);
}

template<typename SolverType, typename... ConstraintTypes>
std::string XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::toString(const int indent) const
{
    // TODO: complete toString
    return Object::toString(indent+1);
}

template<typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::update()
{
    // set _x_prev to be ready for the next substep
    _previous_vertices = _mesh->vertices();

    _movePositionsInertially();
    _projectConstraints();
}

template<typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::_movePositionsInertially()
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

template<typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::_projectConstraints()
{
    _solver.solve();

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

template<typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::velocityUpdate()
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

template<typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::createGPUResource()
{
    _gpu_resource = std::make_unique<Sim::XPBDMeshObjectGPUResource>(this);
    _gpu_resource->allocate();
}

template<typename SolverType, typename... ConstraintTypes>
XPBDMeshObjectGPUResource* XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::gpuResource()
{
    assert(_gpu_resource);
    // TODO: see if we can remove this dynamic_cast somehow
    return dynamic_cast<XPBDMeshObjectGPUResource*>(_gpu_resource.get());
}

template<typename SolverType, typename... ConstraintTypes>
const XPBDMeshObjectGPUResource* XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::gpuResource() const
{
    assert(_gpu_resource);
    // TODO: see if we can remove this dynamic_cast somehow
    return dynamic_cast<const XPBDMeshObjectGPUResource*>(_gpu_resource.get());
}
#endif

#include "common/XPBDTypedefs.hpp"
// instantiate templates
// TODO: find a way to automate this!
typedef XPBDMeshObjectSolverTypes<XPBDMeshObjectConstraintTypes::StableNeohookean::projector_type_list>::GaussSeidel StableNeohookeanGaussSeidel;
template class XPBDMeshObject<StableNeohookeanGaussSeidel, XPBDMeshObjectConstraintTypes::StableNeohookean::constraint_type_list>;

typedef XPBDMeshObjectSolverTypes<XPBDMeshObjectConstraintTypes::StableNeohookean::projector_type_list>::Jacobi StableNeohookeanJacobi;
template class XPBDMeshObject<StableNeohookeanJacobi, XPBDMeshObjectConstraintTypes::StableNeohookean::constraint_type_list>;

typedef XPBDMeshObjectSolverTypes<XPBDMeshObjectConstraintTypes::StableNeohookean::projector_type_list>::ParallelJacobi StableNeohookeanParallelJacobi;
template class XPBDMeshObject<StableNeohookeanParallelJacobi, XPBDMeshObjectConstraintTypes::StableNeohookean::constraint_type_list>;

typedef XPBDMeshObjectSolverTypes<XPBDMeshObjectConstraintTypes::StableNeohookeanCombined::projector_type_list>::GaussSeidel StableNeohookeanCombinedGaussSeidel;
template class XPBDMeshObject<StableNeohookeanCombinedGaussSeidel, XPBDMeshObjectConstraintTypes::StableNeohookeanCombined::constraint_type_list>;

typedef XPBDMeshObjectSolverTypes<XPBDMeshObjectConstraintTypes::StableNeohookeanCombined::projector_type_list>::Jacobi StableNeohookeanCombinedJacobi;
template class XPBDMeshObject<StableNeohookeanCombinedJacobi, XPBDMeshObjectConstraintTypes::StableNeohookeanCombined::constraint_type_list>;

typedef XPBDMeshObjectSolverTypes<XPBDMeshObjectConstraintTypes::StableNeohookeanCombined::projector_type_list>::ParallelJacobi StableNeohookeanCobminedParallelJacobi;
template class XPBDMeshObject<StableNeohookeanCobminedParallelJacobi, XPBDMeshObjectConstraintTypes::StableNeohookeanCombined::constraint_type_list>;


// CTAD
template<typename SolverType, typename ...ConstraintTypes> XPBDMeshObject(TypeList<ConstraintTypes...>, const Simulation*, const XPBDMeshObjectConfig* config)
    -> XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>;

} // namespace Sim