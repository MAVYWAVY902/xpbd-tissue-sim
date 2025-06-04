#include "simobject/XPBDMeshObject.hpp"
#include "simobject/RigidObject.hpp"
#include "simulation/Simulation.hpp"

#include "solver/xpbd_solver/XPBDGaussSeidelSolver.hpp"
#include "solver/xpbd_solver/XPBDJacobiSolver.hpp"
#include "solver/xpbd_solver/XPBDParallelJacobiSolver.hpp"
#include "solver/constraint/StaticDeformableCollisionConstraint.hpp"
#include "solver/constraint/RigidDeformableCollisionConstraint.hpp"
#include "solver/constraint/HydrostaticConstraint.hpp"
#include "solver/constraint/DeviatoricConstraint.hpp"
#include "solver/xpbd_projector/CombinedConstraintProjector.hpp"
#include "solver/xpbd_projector/ConstraintProjector.hpp"
#include "solver/xpbd_projector/RigidBodyConstraintProjector.hpp"
#include "utils/MeshUtils.hpp"

#include "geometry/DeformableMeshSDF.hpp"

#ifdef HAVE_CUDA
#include "gpu/resource/XPBDMeshObjectGPUResource.hpp"
#endif

namespace Sim
{

XPBDMeshObject_Base::XPBDMeshObject_Base(const Simulation* sim, const ConfigType* config)
    : Object(sim, config), TetMeshObject(config, config)
{}

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

template<typename SolverType, typename ...ConstraintTypes>
XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::XPBDMeshObject(const Simulation* sim, const ConfigType* config)
    : XPBDMeshObject_Base(sim, config), _material(config->materialConfig()),
        _solver(this, config->numSolverIters().value(), config->residualPolicy().value())
{
    /* extract values from the Config object */
    
    // set initial velocity if specified in config
    _initial_velocity = config->initialVelocity();
    
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
void XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::createSDF()
{
    _sdf = SDFType(this, _sim->embreeScene());
}

template<typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::setup()
{
    loadAndConfigureMesh();

    _solver.setup();

    // set size of collision constraint projector vectors
    using StaticCollisionConstraintType = Solver::ConstraintProjector<SolverType::is_first_order, Solver::StaticDeformableCollisionConstraint>;
    using RigidCollisionConstraintType = Solver::ConstraintProjector<SolverType::is_first_order, Solver::RigidDeformableCollisionConstraint>;
    // _solver.template setNumProjectorsOfType<StaticCollisionConstraintType>(_mesh->numFaces() + _mesh->numVertices());
    // _solver.template setNumProjectorsOfType<RigidCollisionConstraintType>(_mesh->numFaces());

    // initialize the previous vertices matrix once we've loaded the mesh
    _previous_vertices = _mesh->vertices();
    _vertex_velocities = Geometry::Mesh::VerticesMat::Zero(3, _mesh->numVertices());
    _vertex_velocities.colwise() = _initial_velocity;

    _calculatePerVertexQuantities();
    _createElasticConstraints();     // create constraints and add ConstraintProjectors to the solver object
}

template<typename SolverType, typename... ConstraintTypes>
int XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::numConstraintsForPosition(const int index) const
{
    if constexpr (std::is_same_v<typename SolverType::projector_type_list, XPBDMeshObjectConstraintConfigurations::StableNeohookean::projector_type_list>)
    {
        return 2*_vertex_attached_elements[index];   // if sequential constraints are used, there are 2 constraints per element ==> # of constraint updates = 2 * # of elements attached to that vertex
    }
    else if constexpr (std::is_same_v<typename SolverType::projector_type_list, XPBDMeshObjectConstraintConfigurations::StableNeohookeanCombined::projector_type_list>)
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
                                    int face_ind, const Real u, const Real v, const Real w)
{
    const Eigen::Vector3i face = _mesh->face(face_ind);
    int v1 = face[0];
    int v2 = face[1];
    int v3 = face[2];

    Real* v1_ptr = _mesh->vertexPointer(v1);
    Real* v2_ptr = _mesh->vertexPointer(v2);
    Real* v3_ptr = _mesh->vertexPointer(v3);

    Real m1 = vertexMass(v1);
    Real m2 = vertexMass(v2);
    Real m3 = vertexMass(v3);

    // IN ORDER FOR THIS TO WORK, COLLISION CONSTRAINTS MUST BE RECENTLY CLEARED
    // OTHERWISE, VECTOR MIGHT EXCEED ITS CAPACITY AND POINTERS TO CONSTRAINTS IN CONSTRAINT PROJECTORS WILL BECOME INVALID
    // TODO: is there a better way?
    Solver::StaticDeformableCollisionConstraint& collision_constraint = 
        _constraints.template emplace_back<Solver::StaticDeformableCollisionConstraint>(sdf, p, n, v1, v1_ptr, m1, v2, v2_ptr, m2, v3, v3_ptr, m3, u, v, w);

    _solver.setConstraintProjector(face_ind, _sim->dt(), &collision_constraint);

    // std::cout << "Adding static collision constraint!" << std::endl;
    // _solver.addConstraintProjector(_sim->dt(), &collision_constraint);

    // XPBDCollisionConstraint xpbd_collision_constraint;
    // xpbd_collision_constraint.constraint = std::move(collision_constraint);
    // xpbd_collision_constraint.projector_index = index;
    // xpbd_collision_constraint.num_steps_unused = 0;

    // _collision_constraints.push_back(std::move(xpbd_collision_constraint));
}

template<typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::addVertexStaticCollisionConstraint(const Geometry::SDF* sdf, const Vec3r& p, const Vec3r& n, int vert_ind)
{
    
    Real* v_ptr = _mesh->vertexPointer(vert_ind);

    Real m = vertexMass(vert_ind);

    Solver::StaticDeformableCollisionConstraint& collision_constraint = 
        _constraints.template emplace_back<Solver::StaticDeformableCollisionConstraint>(sdf, p, n, vert_ind, v_ptr, m, vert_ind, v_ptr, m, vert_ind, v_ptr, m, 1, 0, 0);

    _solver.addConstraintProjector(_sim->dt(), &collision_constraint);
    // _solver.setConstraintProjector(_mesh->numFaces() + vert_ind, _sim->dt(), &collision_constraint);
}

template<typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::addRigidDeformableCollisionConstraint(const Geometry::SDF* sdf, Sim::RigidObject* rigid_obj, const Vec3r& rigid_body_point, const Vec3r& collision_normal,
                                       int face_ind, const Real u, const Real v, const Real w)
{
    const Eigen::Vector3i face = _mesh->face(face_ind);
    int v1 = face[0];
    int v2 = face[1];
    int v3 = face[2];
    
    Real* v1_ptr = _mesh->vertexPointer(v1);
    Real* v2_ptr = _mesh->vertexPointer(v2);
    Real* v3_ptr = _mesh->vertexPointer(v3);

    Real m1 = vertexMass(v1);
    Real m2 = vertexMass(v2);
    Real m3 = vertexMass(v3);

    Solver::RigidDeformableCollisionConstraint& collision_constraint = 
        _constraints.template emplace_back<Solver::RigidDeformableCollisionConstraint>(sdf, rigid_obj, rigid_body_point, collision_normal, v1, v1_ptr, m1, v2, v2_ptr, m2, v3, v3_ptr, m3, u, v, w);

    _solver.addConstraintProjector(_sim->dt(), &collision_constraint);
    // _solver.setConstraintProjector(face_ind, _sim->dt(), &collision_constraint);

    // XPBDCollisionConstraint xpbd_collision_constraint;
    // xpbd_collision_constraint.constraint = std::move(collision_constraint);
    // xpbd_collision_constraint.projector_index = index;
    // xpbd_collision_constraint.num_steps_unused = 0;

    // _collision_constraints.push_back(std::move(xpbd_collision_constraint));<
}

template<typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::clearCollisionConstraints()
{
    // set any collision constraint projectors in the solver invalid
    // NOTE: because the collision constraint
    using StaticCollisionConstraintType = Solver::ConstraintProjector<SolverType::is_first_order, Solver::StaticDeformableCollisionConstraint>;
    using RigidCollisionConstraintType = Solver::ConstraintProjector<SolverType::is_first_order, Solver::RigidDeformableCollisionConstraint>;
    // _solver.template setAllProjectorsOfTypeInvalid<StaticCollisionConstraintType>();
    // _solver.template setAllProjectorsOfTypeInvalid<RigidCollisionConstraintType>();
    _solver.template clearProjectorsOfType<StaticCollisionConstraintType>();
    _solver.template clearProjectorsOfType<RigidCollisionConstraintType>();

    // // clear the collision constraints lists
    _constraints.template clear<Solver::StaticDeformableCollisionConstraint>();
    _constraints.template clear<Solver::RigidDeformableCollisionConstraint>();


}

template<typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::removeOldCollisionConstraints(const int /*threshold*/)
{
    // TODO: implement (actually I don't think this is used, but it might be in the future)
}

template<typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::addAttachmentConstraint(int v_ind, const Vec3r* attach_pos_ptr, const Vec3r& attachment_offset)
{
    Real* v_ptr = _mesh->vertexPointer(v_ind);
    Real mass = vertexMass(v_ind);

    Solver::AttachmentConstraint& attachment_constraint = 
        _constraints.template emplace_back<Solver::AttachmentConstraint>(v_ind, v_ptr, mass, attach_pos_ptr, attachment_offset);
    
    _solver.addConstraintProjector(_sim->dt(), &attachment_constraint);
}

template<typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::clearAttachmentConstraints()
{
    using AttachmentConstraintProjType = Solver::ConstraintProjector<SolverType::is_first_order, Solver::AttachmentConstraint>;
    _solver.template clearProjectorsOfType<AttachmentConstraintProjType>();

    _constraints.template clear<Solver::AttachmentConstraint>();
}

template<typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::_calculatePerVertexQuantities()
{
    // calculate masses for each vertex
    _vertex_masses.resize(_mesh->numVertices());
    _vertex_inv_masses.resize(_mesh->numVertices());
    _vertex_volumes.resize(_mesh->numVertices());
    _vertex_attached_elements.resize(_mesh->numVertices());
    _is_fixed_vertex.resize(_mesh->numVertices());
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
        _is_fixed_vertex[i] = false;
    } 
}

template<typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::_createElasticConstraints()
{
    // TODO: think about this... we need to resize each vector initially so that pointers to constraints are still valid...
    // alternative: use vector unique_ptr<Constraint> 
    // std::cout << "reserving " << tetMesh()->numElements() << " for elastic constraints..." << std::endl;
    _constraints.template reserve<Solver::HydrostaticConstraint>(tetMesh()->numElements());
    _constraints.template reserve<Solver::DeviatoricConstraint>(tetMesh()->numElements());
    // std::cout << "reserving " << _mesh->numFaces() << " for collision constraints..." << std::endl;
    _constraints.template reserve<Solver::StaticDeformableCollisionConstraint>(_mesh->numFaces());
    _constraints.template reserve<Solver::RigidDeformableCollisionConstraint>(_mesh->numFaces());

    _constraints.template reserve<Solver::AttachmentConstraint>(_mesh->numVertices());

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
        // if (std::holds_alternative<XPBDMeshObjectConstraintConfigurations::StableNeohookean>(_constraint_type))
        if constexpr (std::is_same_v<typename SolverType::projector_type_list, XPBDMeshObjectConstraintConfigurations::StableNeohookean::projector_type_list>)
        {
            Solver::HydrostaticConstraint& hyd_constraint = 
                _constraints.template emplace_back<Solver::HydrostaticConstraint>(v0, v0_ptr, m0, v1, v1_ptr, m1, v2, v2_ptr, m2, v3, v3_ptr, m3, _material);
            Solver::DeviatoricConstraint& dev_constraint = 
                _constraints.template emplace_back<Solver::DeviatoricConstraint>(v0, v0_ptr, m0, v1, v1_ptr, m1, v2, v2_ptr, m2, v3, v3_ptr, m3, _material);
            
            // TODO: support separate constraints - maybe though SeparateConstraintProjector class?.
            _solver.addConstraintProjector(_sim->dt(), &hyd_constraint);
            _solver.addConstraintProjector(_sim->dt(), &dev_constraint);
            // _solver.addConstraintProjector(_sim->dt(), projector_options, &dev_constraint, &hyd_constraint);
            
        }
        // else if (std::holds_alternative<XPBDMeshObjectConstraintConfigurations::StableNeohookeanCombined>(_constraint_type))
        if constexpr (std::is_same_v<typename SolverType::projector_type_list, XPBDMeshObjectConstraintConfigurations::StableNeohookeanCombined::projector_type_list>)
        {
            Solver::HydrostaticConstraint& hyd_constraint = 
                _constraints.template emplace_back<Solver::HydrostaticConstraint>(v0, v0_ptr, m0, v1, v1_ptr, m1, v2, v2_ptr, m2, v3, v3_ptr, m3, _material);
            Solver::DeviatoricConstraint& dev_constraint = 
                _constraints.template emplace_back<Solver::DeviatoricConstraint>(v0, v0_ptr, m0, v1, v1_ptr, m1, v2, v2_ptr, m2, v3, v3_ptr, m3, _material);

            _solver.addConstraintProjector(_sim->dt(), &dev_constraint, &hyd_constraint);
        }
    }
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
    // std::cout << "Number of collision constraints: " << _constraints.template get<Solver::StaticDeformableCollisionConstraint>().size() << std::endl;
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
    // for (int i = 0; i < _mesh->numVertices(); i++)
    // {
    //     const Vec3r& v = _mesh->vertex(i);
    //     if (v[2] < 0)
    //     {
    //         _mesh->setVertex(i, Vec3r(v[0], v[1], 0));
    //     }
    // }

    // TODO: replace with constraints?
    for (int i = 0; i < _mesh->numVertices(); i++)
    {
        if (vertexFixed(i))
        {
            _mesh->setVertex(i, _previous_vertices.col(i));
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

template<typename SolverType, typename... ConstraintTypes>
Vec3r XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::elasticForceAtVertex(int index)
{
    // get elements attached to the vertex in the mesh
    const std::vector<int>& _attached_elements = tetMesh()->attachedElementsToVertex(index);

    // std::cout << "Looping through attached elements..." << std::endl;
    Vec3r total_force = Vec3r::Zero();
    for (const auto& elem_index : _attached_elements)
    {
        // std::cout << "Elastic force for element " << elem_index << std::endl;
        const Vec3r& dev_force = _constraints.template get<Solver::DeviatoricConstraint>()[elem_index].elasticForce(index);
        const Vec3r& hyd_force = _constraints.template get<Solver::HydrostaticConstraint>()[elem_index].elasticForce(index);

        // TODO: THIS IS A HACK THAT WILL PROBABLY BITE ME IN THE ASS LATER
        // for some reason, very small elements produce incorrect forces (they are very large, probably due to machine precision limits) - which messes up force feedback in the Haptic demos
        // need to find a better fix than this
        if (_constraints.template get<Solver::DeviatoricConstraint>()[elem_index].restVolume() > 1e-10) 
            total_force += dev_force + hyd_force;
        // else
        //     std::cout << "LARGE FORCE ELEMENT VOLUME: " << _constraints.template get<Solver::DeviatoricConstraint>()[elem_index].restVolume() << std::endl;
        // std::cout << "Forces at element " << elem_index << ": (" << dev_force[0] << ", " << dev_force[1] << ", " << dev_force[2] << ") Hyd: ("<< hyd_force[0] << ", " << hyd_force[1] << ", " << hyd_force[2] << ")" << std::endl;
        
    }

    // std::cout << ""

    // std::cout << "Total elastic force at vertex: " << total_force[0] << ", " << total_force[1] << ", " << total_force[2] << std::endl;

    return total_force;
}

#ifdef HAVE_CUDA

template<typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::createGPUResource()
{
    if (!_gpu_resource)
    {
        _gpu_resource = std::make_unique<Sim::XPBDMeshObjectGPUResource>(this);
        _gpu_resource->allocate();
    }
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

// First attempt at automating template instantation - didn't work, bunch of linker errors.

// // Helper to instantiate XPBDMeshObject
// template<typename SolverType, typename ConstraintsTypeList>
// struct XPBDMeshObjectInstantiator
// {
//     // static constexpr void instantiate()
//     // {
//     //     // INSTANTIATE_XPBDMESHOBJECT(SolverType, ConstraintsTypeList);
//     //     return (void)sizeof
//     // }
//     // static constexpr int dummy = sizeof(XPBDMeshObject<SolverType, ConstraintsTypeList>);
//     inline static constexpr int __attribute__((used)) dummy = sizeof(XPBDMeshObject<SolverType, ConstraintsTypeList>);
// };

// template<typename SolverTypeList>
// struct InstantiateXPBDMeshObjectsFromSolverType;

// template<typename ...SolverTypes>
// struct InstantiateXPBDMeshObjectsFromSolverType<TypeList<SolverTypes...>>
// {
//     // static constexpr void instantiate()
//     // {
//     //     (XPBDMeshObjectInstantiator<SolverTypes, typename SolverTypes::constraint_type_list>::instantiate(), ...);
//     // }
//     // using expand = int[];
//     // inline static constexpr expand __attribute__((used)) dummy = {
//     //     (XPBDMeshObjectInstantiator<SolverTypes, typename SolverTypes::constraint_type_list>::dummy, 0)...
//     // };
//     std::tuple<XPBDMeshObject<SolverTypes, typename SolverTypes::constraint_type_list>...> unused;
// };

// template<typename ConstraintConfigTypeList>
// struct InstantiateAllXPBDMeshObjects;

// template<typename ...ConstraintConfigs>
// struct InstantiateAllXPBDMeshObjects<TypeList<ConstraintConfigs...>>
// {
//     // static constexpr void instantiate()
//     // {
//     //     (InstantiateXPBDMeshObjectsFromSolverType<typename XPBDMeshObjectSolverTypes<typename ConstraintConfigs::projector_type_list>::type_list>::instantiate(), ...);
//     // }
//     // using expand = int[];
//     // inline static constexpr expand __attribute__((used)) dummy = {
//     //     (InstantiateXPBDMeshObjectsFromSolverType<typename XPBDMeshObjectSolverTypes<typename ConstraintConfigs::projector_type_list>::type_list>::dummy[0], 0)...
//     // };
//     std::tuple<InstantiateXPBDMeshObjectsFromSolverType<typename XPBDMeshObjectSolverTypes<typename ConstraintConfigs::projector_type_list>::type_list>...> unused;
// };

// // inline constexpr int __attribute__((used)) enusre_instantiation = InstantiateAllXPBDMeshObjects<XPBDMeshObjectConstraintConfigurations::type_list>::dummy[0];
// // InstantiateAllXPBDMeshObjects<typename XPBDMeshObjectConstraintConfigurations::type_list>::instantiate();
// template struct InstantiateAllXPBDMeshObjects<typename XPBDMeshObjectConstraintConfigurations::type_list>;


// TODO: find a way to automate this!
using SolverTypesStableNeohookean = XPBDObjectSolverTypes<XPBDMeshObjectConstraintConfigurations::StableNeohookean::projector_type_list>;
using SolverTypesStableNeohookeanCombined = XPBDObjectSolverTypes<XPBDMeshObjectConstraintConfigurations::StableNeohookeanCombined::projector_type_list>;
using StableNeohookeanConstraints = XPBDMeshObjectConstraintConfigurations::StableNeohookean::constraint_type_list;
using StableNeohookeanCombinedConstraints = XPBDMeshObjectConstraintConfigurations::StableNeohookeanCombined::constraint_type_list;

// Stable Neohookean constraint config
template class XPBDMeshObject<SolverTypesStableNeohookean::GaussSeidel, StableNeohookeanConstraints>;
template class XPBDMeshObject<SolverTypesStableNeohookean::Jacobi, StableNeohookeanConstraints>;
template class XPBDMeshObject<SolverTypesStableNeohookean::ParallelJacobi, StableNeohookeanConstraints>;

// Stable Neohookean Combined constraint config
template class XPBDMeshObject<SolverTypesStableNeohookeanCombined::GaussSeidel, StableNeohookeanCombinedConstraints>;
template class XPBDMeshObject<SolverTypesStableNeohookeanCombined::Jacobi, StableNeohookeanCombinedConstraints>;
template class XPBDMeshObject<SolverTypesStableNeohookeanCombined::ParallelJacobi, StableNeohookeanCombinedConstraints>;

using FirstOrderSolverTypesStableNeohookean = FirstOrderXPBDObjectSolverTypes<FirstOrderXPBDMeshObjectConstraintConfigurations::StableNeohookean::projector_type_list>;
using FirstOrderSolverTypesStableNeohookeanCombined = FirstOrderXPBDObjectSolverTypes<FirstOrderXPBDMeshObjectConstraintConfigurations::StableNeohookeanCombined::projector_type_list>;
using FirstOrderStableNeohookeanConstraints = FirstOrderXPBDMeshObjectConstraintConfigurations::StableNeohookean::constraint_type_list;
using FirstOrderStableNeohookeanCombinedConstraints = FirstOrderXPBDMeshObjectConstraintConfigurations::StableNeohookeanCombined::constraint_type_list;
template class XPBDMeshObject<FirstOrderSolverTypesStableNeohookean::GaussSeidel, FirstOrderStableNeohookeanConstraints>;
template class XPBDMeshObject<FirstOrderSolverTypesStableNeohookean::Jacobi, FirstOrderStableNeohookeanConstraints>;
template class XPBDMeshObject<FirstOrderSolverTypesStableNeohookean::ParallelJacobi, FirstOrderStableNeohookeanConstraints>;

template class XPBDMeshObject<FirstOrderSolverTypesStableNeohookeanCombined::GaussSeidel, FirstOrderStableNeohookeanCombinedConstraints>;
template class XPBDMeshObject<FirstOrderSolverTypesStableNeohookeanCombined::Jacobi, FirstOrderStableNeohookeanCombinedConstraints>;
template class XPBDMeshObject<FirstOrderSolverTypesStableNeohookeanCombined::ParallelJacobi, FirstOrderStableNeohookeanCombinedConstraints>;
// CTAD
// template<typename SolverType, typename ...ConstraintTypes> XPBDMeshObject(TypeList<ConstraintTypes...>, const Simulation*, const XPBDMeshObjectConfig* config)
//     -> XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>;

} // namespace Sim