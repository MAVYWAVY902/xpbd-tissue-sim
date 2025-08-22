#include "simobject/XPBDMeshObject.hpp"

#include "config/simobject/XPBDMeshObjectConfig.hpp"
#include "config/simobject/FirstOrderXPBDMeshObjectConfig.hpp"

#include "simobject/RigidObject.hpp"
#include "simulation/Simulation.hpp"

#include "solver/xpbd_solver/XPBDGaussSeidelSolver.hpp"
#include "solver/xpbd_solver/XPBDJacobiSolver.hpp"
#include "solver/xpbd_solver/XPBDParallelJacobiSolver.hpp"
#include "solver/constraint/StaticDeformableCollisionConstraint.hpp"
#include "solver/constraint/RigidDeformableCollisionConstraint.hpp"
#include "solver/constraint/DeformableDeformableCollisionConstraint.hpp"
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

template<bool IsFirstOrder>
XPBDMeshObject_Base_<IsFirstOrder>::XPBDMeshObject_Base_(const Simulation* sim, const ConfigType* config)
    : Object(sim, config), TetMeshObject(config, config),
    _material(config->materialConfig())
{}

template<bool IsFirstOrder>
void XPBDMeshObject_Base_<IsFirstOrder>::createSDF()
{
    if (!_sdf.has_value())
        _sdf.emplace(this, _sim->embreeScene());
}

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::XPBDMeshObject_(const Simulation* sim, const ConfigType* config)
    : XPBDMeshObject_Base_<IsFirstOrder>(sim, config),
        _solver(this, config->numSolverIters(), config->residualPolicy())
{
    // make sure that if this object is using the 1st-Order formulation, that the XPBDSolver is too
    static_assert(SolverType::is_first_order == IsFirstOrder, "XPBD solver order much match object order!");

    /* extract values from the Config object */
    
    // set initial velocity if specified in config
    _initial_velocity = config->initialVelocity();
    
    // constraint specifications
    _constraint_type = config->constraintType();

    // local collision iterations
    _num_local_collision_iters = config->numLocalCollisionIters();

    // get the damping multiplier for 1st-order objects
    if constexpr (IsFirstOrder)
    {
        _damping_multiplier = config->dampingMultiplier();
    }
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::~XPBDMeshObject_()
{

}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
Geometry::AABB XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::boundingBox() const
{
    return _mesh->boundingBox();
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::setup()
{
    loadAndConfigureMesh();

    _solver.setup();

    // initialize the previous vertices matrix once we've loaded the mesh
    _previous_vertices = _mesh->vertices();
    _vertex_velocities = Geometry::Mesh::VerticesMat::Zero(3, _mesh->numVertices());
    _vertex_velocities.colwise() = _initial_velocity;

    _calculatePerVertexQuantities();
    _createElasticConstraints();     // create constraints and add ConstraintProjectors to the solver object
}

// template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
// int XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::numConstraintsForPosition(const int index) const
// {
//     if constexpr (std::is_same_v<typename SolverType::projector_type_list, XPBDMeshObjectConstraintConfigurations::StableNeohookean::projector_type_list>)
//     {
//         return 2*_vertex_attached_elements[index];   // if sequential constraints are used, there are 2 constraints per element ==> # of constraint updates = 2 * # of elements attached to that vertex
//     }
//     else if constexpr (std::is_same_v<typename SolverType::projector_type_list, XPBDMeshObjectConstraintConfigurations::StableNeohookeanCombined::projector_type_list>)
//     {
//         return _vertex_attached_elements[index];     // if combined constraints are used, there are 2 constraints per element but they are solved together ==> # of constraint updates = # of elements attached to that vertex
//     }
//     else
//     {
//         assert(0); // something weird happened, shouldn't get to here
//         return 0;
//     }
// }

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
Solver::ConstraintProjectorReference<Solver::ConstraintProjector<IsFirstOrder, Solver::StaticDeformableCollisionConstraint>>
XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::addStaticCollisionConstraint(const Geometry::SDF* sdf, const Vec3r& p, const Vec3r& n,
                                    int face_ind, const Real u, const Real v, const Real w)
{
    const Eigen::Vector3i face = _mesh->face(face_ind);
    int v1 = face[0];
    int v2 = face[1];
    int v3 = face[2];

    Real* v1_ptr = _mesh->vertexPointer(v1);
    Real* v2_ptr = _mesh->vertexPointer(v2);
    Real* v3_ptr = _mesh->vertexPointer(v3);

    Real m1 = vertexConstraintInertia(v1);
    Real m2 = vertexConstraintInertia(v2);
    Real m3 = vertexConstraintInertia(v3);

    // IN ORDER FOR THIS TO WORK, COLLISION CONSTRAINTS MUST BE RECENTLY CLEARED
    // OTHERWISE, VECTOR MIGHT EXCEED ITS CAPACITY AND POINTERS TO CONSTRAINTS IN CONSTRAINT PROJECTORS WILL BECOME INVALID
    // TODO: is there a better way?
    std::vector<Solver::StaticDeformableCollisionConstraint>& constraint_vec = _constraints.template get<Solver::StaticDeformableCollisionConstraint>();
    constraint_vec.emplace_back(sdf, p, n, v1, v1_ptr, m1, v2, v2_ptr, m2, v3, v3_ptr, m3, u, v, w);

    using ConstraintRefType = Solver::ConstraintReference<Solver::StaticDeformableCollisionConstraint>;
    return _solver.addConstraintProjector(_sim->dt(), ConstraintRefType(constraint_vec, constraint_vec.size()-1));
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
Solver::ConstraintProjectorReference<Solver::RigidBodyConstraintProjector<IsFirstOrder, Solver::RigidDeformableCollisionConstraint>>
XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::addRigidDeformableCollisionConstraint(const Geometry::SDF* sdf, Sim::RigidObject* rigid_obj, const Vec3r& rigid_body_point, const Vec3r& collision_normal,
                                       int face_ind, const Real u, const Real v, const Real w)
{
    const Eigen::Vector3i face = _mesh->face(face_ind);
    int v1 = face[0];
    int v2 = face[1];
    int v3 = face[2];
    
    Real* v1_ptr = _mesh->vertexPointer(v1);
    Real* v2_ptr = _mesh->vertexPointer(v2);
    Real* v3_ptr = _mesh->vertexPointer(v3);

    Real m1 = vertexConstraintInertia(v1);
    Real m2 = vertexConstraintInertia(v2);
    Real m3 = vertexConstraintInertia(v3);

    std::vector<Solver::RigidDeformableCollisionConstraint>& constraint_vec = _constraints.template get<Solver::RigidDeformableCollisionConstraint>();
    constraint_vec.emplace_back(sdf, rigid_obj, rigid_body_point, collision_normal, v1, v1_ptr, m1, v2, v2_ptr, m2, v3, v3_ptr, m3, u, v, w);

    using ConstraintRefType = Solver::ConstraintReference<Solver::RigidDeformableCollisionConstraint>;
    return _solver.addConstraintProjector(_sim->dt(), ConstraintRefType(constraint_vec, constraint_vec.size()-1));
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::clearCollisionConstraints()
{
    // set any collision constraint projectors in the solver invalid
    // NOTE: because the collision constraint
    using StaticCollisionConstraintType = Solver::ConstraintProjector<IsFirstOrder, Solver::StaticDeformableCollisionConstraint>;
    using DeformableCollisionConstraintType = Solver::ConstraintProjector<IsFirstOrder, Solver::DeformableDeformableCollisionConstraint>;
    using RigidCollisionConstraintType = Solver::RigidBodyConstraintProjector<IsFirstOrder, Solver::RigidDeformableCollisionConstraint>;
    _solver.template clearProjectorsOfType<StaticCollisionConstraintType>();
    _solver.template clearProjectorsOfType<DeformableCollisionConstraintType>();
    _solver.template clearProjectorsOfType<RigidCollisionConstraintType>();

    // clear the collision constraints lists
    _constraints.template clear<Solver::StaticDeformableCollisionConstraint>();
    _constraints.template clear<Solver::DeformableDeformableCollisionConstraint>();
    _constraints.template clear<Solver::RigidDeformableCollisionConstraint>();


}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
Solver::ConstraintProjectorReference<Solver::ConstraintProjector<IsFirstOrder, Solver::AttachmentConstraint>> 
XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::addAttachmentConstraint(int v_ind, const Vec3r* attach_pos_ptr, const Vec3r& attachment_offset)
{
    Real* v_ptr = _mesh->vertexPointer(v_ind);
    Real mass = vertexConstraintInertia(v_ind);

    std::vector<Solver::AttachmentConstraint>& constraint_vec = _constraints.template get<Solver::AttachmentConstraint>();
    constraint_vec.emplace_back(v_ind, v_ptr, mass, attach_pos_ptr, attachment_offset);
    
    using ConstraintRefType = Solver::ConstraintReference<Solver::AttachmentConstraint>;
    return _solver.addConstraintProjector(_sim->dt(), ConstraintRefType(constraint_vec, constraint_vec.size()-1));
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::clearAttachmentConstraints()
{
    using AttachmentConstraintProjType = Solver::ConstraintProjector<IsFirstOrder, Solver::AttachmentConstraint>;
    // clear projectors
    _solver.template clearProjectorsOfType<AttachmentConstraintProjType>();
    // clear constraints
    _constraints.template clear<Solver::AttachmentConstraint>();
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::_calculatePerVertexQuantities()
{
    // calculate masses for each vertex
    _vertex_masses.resize(_mesh->numVertices());
    _vertex_volumes.resize(_mesh->numVertices());
    _is_fixed_vertex.resize(_mesh->numVertices(), false);
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
    }

    // for 1st-order objects, calculate per-vertex damping
    if constexpr (IsFirstOrder)
    {
        _vertex_B.resize(_mesh->numVertices());
        for (int i = 0; i < _mesh->numVertices(); i++)
        {
            _vertex_B[i] = _vertex_volumes[i] * _damping_multiplier;
        }
    }
    
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::_createElasticConstraints()
{
    // reserve space for the elastic constraints we're creating
    _constraints.template reserve<Solver::HydrostaticConstraint>(tetMesh()->numElements());
    _constraints.template reserve<Solver::DeviatoricConstraint>(tetMesh()->numElements());

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

        Real m0 = vertexConstraintInertia(v0);
        Real m1 = vertexConstraintInertia(v1);
        Real m2 = vertexConstraintInertia(v2);
        Real m3 = vertexConstraintInertia(v3);

        // if the constraint configuration is StableNeohookean, add separate constraint projectors for the hydrostatic and deviatoric constraints
        if constexpr (std::is_same_v<typename SolverType::projector_type_list, typename XPBDMeshObjectConstraintConfigurations<IsFirstOrder>::StableNeohookean::projector_type_list>)
        {
            std::vector<Solver::HydrostaticConstraint>& hyd_constraint_vec = _constraints.template get<Solver::HydrostaticConstraint>();
            std::vector<Solver::DeviatoricConstraint>& dev_constraint_vec = _constraints.template get<Solver::DeviatoricConstraint>();
            hyd_constraint_vec.emplace_back(v0, v0_ptr, m0, v1, v1_ptr, m1, v2, v2_ptr, m2, v3, v3_ptr, m3, _material);
            dev_constraint_vec.emplace_back(v0, v0_ptr, m0, v1, v1_ptr, m1, v2, v2_ptr, m2, v3, v3_ptr, m3, _material);
            
            using HydConstraintRefType = Solver::ConstraintReference<Solver::HydrostaticConstraint>;
            using DevConstraintRefType = Solver::ConstraintReference<Solver::DeviatoricConstraint>;
            // TODO: support separate constraints - maybe though SeparateConstraintProjector class?.
            _solver.addConstraintProjector(_sim->dt(), HydConstraintRefType(hyd_constraint_vec, hyd_constraint_vec.size()-1));
            _solver.addConstraintProjector(_sim->dt(), DevConstraintRefType(dev_constraint_vec, dev_constraint_vec.size()-1));
            
        }
        // if the constraint configuration is StableNeohookeanCombined, add a combined constraint projector for the hydrostatic and deviatoric constraints
        if constexpr (std::is_same_v<typename SolverType::projector_type_list, typename XPBDMeshObjectConstraintConfigurations<IsFirstOrder>::StableNeohookeanCombined::projector_type_list>)
        {
            std::vector<Solver::HydrostaticConstraint>& hyd_constraint_vec = _constraints.template get<Solver::HydrostaticConstraint>();
            std::vector<Solver::DeviatoricConstraint>& dev_constraint_vec = _constraints.template get<Solver::DeviatoricConstraint>();
            hyd_constraint_vec.emplace_back(v0, v0_ptr, m0, v1, v1_ptr, m1, v2, v2_ptr, m2, v3, v3_ptr, m3, _material);
            dev_constraint_vec.emplace_back(v0, v0_ptr, m0, v1, v1_ptr, m1, v2, v2_ptr, m2, v3, v3_ptr, m3, _material);

            using HydConstraintRefType = Solver::ConstraintReference<Solver::HydrostaticConstraint>;
            using DevConstraintRefType = Solver::ConstraintReference<Solver::DeviatoricConstraint>;
            
            _solver.addConstraintProjector(_sim->dt(),
                DevConstraintRefType(dev_constraint_vec, dev_constraint_vec.size()-1), 
                HydConstraintRefType(hyd_constraint_vec, hyd_constraint_vec.size()-1)
            );
        }
    }
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
std::string XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::toString(const int indent) const
{
    // TODO: complete toString
    return Object::toString(indent+1);
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::update()
{
    // set _x_prev to be ready for the next substep
    _previous_vertices = _mesh->vertices();

    _movePositionsInertially();
    _projectConstraints();

    // for (int i = 0; i < tetMesh()->numElements(); i++)
    // {
    //     const Mat3r F = tetMesh()->elementDeformationGradient(i);
    //     if (F.determinant() <= 0)
    //     {
    //         std::cout << "element " << i << " det(F) <= 0!" << std::endl;
    //     }
        
    // }
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::_movePositionsInertially()
{
    const Real dt = _sim->dt();
    if constexpr (IsFirstOrder)
    {
        for (int i = 0; i < _mesh->numVertices(); i++)
        {
            const Real dz = -_sim->gAccel() * _vertex_masses[i] * dt / _vertex_B[i];
            _mesh->displaceVertex(i, Vec3r(0,0,dz));
        }
        
    }
    else
    {
        // move vertices according to their velocity
        _mesh->moveSeparate(dt*_vertex_velocities);
        // external forces (right now just gravity, which acts in -z direction)
        for (int i = 0; i < _mesh->numVertices(); i++)
        {
            const Real dz = -_sim->gAccel() * dt * dt;
            _mesh->displaceVertex(i, Vec3r(0, 0, dz));
        }
    }
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::_projectConstraints()
{
    // global iteration - initial solve of all the constraints
    _solver.solve();


    // local iterations - helpful for better convergence of applied collision constraints

    typename SolverType::projector_reference_container_type proj_to_reproject = _gatherProjectorsForLocalCollisionIterations();
    // reproject the added constraints with solver iterations and no re-initialization
    _solver.solve(proj_to_reproject, _num_local_collision_iters, false);

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
    // enforce fixed vertices (move them back to previous position)
    for (int i = 0; i < _mesh->numVertices(); i++)
    {
        if (vertexFixed(i))
        {
            _mesh->setVertex(i, _previous_vertices.col(i));
        }
    }

}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::velocityUpdate()
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

    // for (int i = 0; i < tetMesh()->numElements(); i++)
    // {
    //     Mat3r F = tetMesh()->elementDeformationGradient(i);
    //     if (F.determinant() < 0)
    //     {
    //         std::cout << "det(F) < 0 for element " << i << std::endl;
    //     }
    // }

    const Geometry::Mesh::VerticesMat& cur_vertices = _mesh->vertices();
    // velocities are simply (cur_pos - last_pos) / deltaT
    _vertex_velocities = (cur_vertices - _previous_vertices) / _sim->dt();
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
Real XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::totalStrainEnergy() const
{
    // iterate over all hydrostatic and deviatoric constraints
    Real total_energy = 0;
    _constraints.template for_each_element<Solver::DeviatoricConstraint, Solver::HydrostaticConstraint>([&total_energy](const auto& constraint){
        Real eval;
        constraint.evaluate(&eval);
        total_energy += eval * eval / constraint.alpha();
    });

    return total_energy;
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
Vec3r XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::elasticForceAtVertex(int index) const
{
    // get elements attached to the vertex in the mesh
    const std::vector<int>& _attached_elements = tetMesh()->vertexAttachedElements(index);

    /** TODO: figure out which approach is correct. */
    Vec3r total_force = Vec3r::Zero();
    Vec3r total_force_proj = Vec3r::Zero();
    for (const auto& elem_index : _attached_elements)
    {
        // std::cout << "Elastic force for element " << elem_index << std::endl;
        const Vec3r& dev_force = _constraints.template get<Solver::DeviatoricConstraint>()[elem_index].elasticForce(index);
        const Vec3r& hyd_force = _constraints.template get<Solver::HydrostaticConstraint>()[elem_index].elasticForce(index);

        Vec3r proj_force = Vec3r::Zero();

        if constexpr (std::is_same_v<typename SolverType::projector_type_list, typename XPBDMeshObjectConstraintConfigurations<IsFirstOrder>::StableNeohookeanCombined::projector_type_list>)
        {
            const auto& proj = 
                _solver.template getConstraintProjector<Solver::CombinedConstraintProjector<IsFirstOrder, Solver::DeviatoricConstraint, Solver::HydrostaticConstraint>>(elem_index);
            std::vector<Vec3r> proj_forces = proj.constraintForces();
            const std::vector<Solver::PositionReference>& positions = proj.positions();
            
            for (unsigned i = 0; i < positions.size(); i++)
            {
                if (positions[i].index == index)
                {
                    proj_force = proj_forces[i];
                    break;
                }
            }
        }

        // TODO: THIS IS A HACK THAT WILL PROBABLY BITE ME IN THE ASS LATER
        // for some reason, very small elements produce incorrect forces (they are very large, probably due to machine precision limits) - which messes up force feedback in the Haptic demos
        // need to find a better fix than this
        if (_constraints.template get<Solver::DeviatoricConstraint>()[elem_index].restVolume() > 1e-10)
        {
            total_force += dev_force + hyd_force;
            total_force_proj += proj_force;
        } 
            
        // else
        //     std::cout << "LARGE FORCE ELEMENT VOLUME: " << _constraints.template get<Solver::DeviatoricConstraint>()[elem_index].restVolume() << std::endl;
        // std::cout << "Forces at element " << elem_index << ": (" << dev_force[0] << ", " << dev_force[1] << ", " << dev_force[2] << ") Hyd: ("<< hyd_force[0] << ", " << hyd_force[1] << ", " << hyd_force[2] << ")" << std::endl;
        
    }

    // std::cout << ""

    std::cout << "\nTotal elastic force at vertex (from constraints): " << total_force[0] << ", " << total_force[1] << ", " << total_force[2] << std::endl;
    std::cout << "Total elastic force at vertex (from projectors): " << total_force_proj[0] << ", " << total_force_proj[1] << ", " << total_force_proj[2] << std::endl;

    return total_force;
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
MatXr XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::stiffnessMatrix() const
{
    // FOR NOW, WE ASSUME THAT IF WE ARE CALCULATING THE STIFFNESS MATRIX, WE DON'T HAVE ANY HARD CONSTRAINTS ON THE OBJECT
    // these would have alpha=0 and thus alpha^-1 would be infinity, corresponding to infinite stiffness.

    // assemble global delC matrix
    size_t num_constraints = _constraints.size();
    VecXr C_vec(num_constraints);
    MatXr orig_delC(num_constraints, 3*_mesh->numVertices());
    VecXr alpha_inv(num_constraints);


    // iterate through each constraint and put its gradient into the global delC matrix
    int constraint_index = 0;
    _constraints.for_each_element([&orig_delC, &alpha_inv, &C_vec, &constraint_index](const auto& constraint)
    {
        // get the gradient from the constraint
        using ConstraintType = std::remove_cv_t<std::remove_reference_t<decltype(constraint)>>;
        Real grad[ConstraintType::NUM_COORDINATES];
        Real C;
        constraint.evaluateWithGradient(&C, grad);

        // get the positions that the constraint affects
        const std::vector<Solver::PositionReference>& constraint_positions = constraint.positions();

        for (unsigned i = 0; i < ConstraintType::NUM_POSITIONS; i++)
        {
            int position_index = constraint_positions[i].index;
            const Vec3r grad_i = Eigen::Map<Vec3r>(grad + 3*i);

            orig_delC.block<1,3>(constraint_index, 3*position_index) = grad_i;
        }

        // add constraint stiffness to alpha
        alpha_inv[constraint_index] = 1.0/constraint.alpha();

        C_vec[constraint_index] = C;

        constraint_index++;
    });

    // compute the Hessian term (through numerical differentiation)
    Real* data_ptr = _mesh->vertices().data();
    Real delta = 0.0001;
    MatXr delC(num_constraints, 3*_mesh->numVertices());
    MatXr grad_delC_i(num_constraints, 3*_mesh->numVertices());
    MatXr hessian_term(3*_mesh->numVertices(), 3*_mesh->numVertices());
    for (int dof = 0; dof < _mesh->vertices().size(); dof++)
    {
        delC = MatXr::Zero(num_constraints, 3*_mesh->numVertices());

        // vary each DOF
        data_ptr[dof] += delta;

        // loop through constraints to calculate the change in delC
        int constraint_index = 0;
        _constraints.for_each_element([&delC, &constraint_index](const auto& constraint)
        {
            // get the gradient from the constraint
            using ConstraintType = std::remove_cv_t<std::remove_reference_t<decltype(constraint)>>;
            Real grad[ConstraintType::NUM_COORDINATES];
            constraint.gradient(grad);

            // get the positions that the constraint affects
            const std::vector<Solver::PositionReference>& constraint_positions = constraint.positions();

            for (unsigned i = 0; i < ConstraintType::NUM_POSITIONS; i++)
            {
                int position_index = constraint_positions[i].index;
                const Vec3r grad_i = Eigen::Map<Vec3r>(grad + 3*i);

                delC.block<1,3>(constraint_index, 3*position_index) = grad_i;
            }

            constraint_index++;
        });

        // compute gradient
        grad_delC_i = (delC - orig_delC) / delta;

        // compute associated column in the Hessian term
        hessian_term.col(dof) = grad_delC_i.transpose() * alpha_inv.asDiagonal() * C_vec;

        data_ptr[dof] -= delta;
    }

    MatXr stiffness_matrix = hessian_term + orig_delC.transpose() * alpha_inv.asDiagonal() * orig_delC;

    // "fix" all the fixed vertices in the mesh by adding a large amount to the diagonal corresponding to their 3 DOF
    for (int i = 0; i < _mesh->numVertices(); i++)
    {
        if (vertexFixed(i))
        {
            for (int j = 0; j < 3; j++)
                stiffness_matrix(3*i+j,3*i+j) += 1e9;
        }
    }
    return stiffness_matrix;
    
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::selfCollisionCheck()
{
    const Geometry::EmbreeScene* embree_scene = _sim->embreeScene();
    for (int i = 0; i < _mesh->numVertices(); i++)
    {
        if (!_mesh->vertexOnSurface(i))
            continue;

        std::set<Geometry::EmbreeHit> hits = embree_scene->tetMeshSelfCollisionQuery(i, this);
        if (hits.size() > 0)
        {
            int face_index = _sdf->closestSurfaceFaceToPointInTet(_mesh->vertex(i), hits.begin()->prim_index);

            if (face_index < 0)
                continue;

            const Eigen::Vector3i& face = _mesh->face(face_index);

            Real* q_ptr = _mesh->vertexPointer(i);
            Real* p1_ptr = _mesh->vertexPointer(face[0]);
            Real* p2_ptr = _mesh->vertexPointer(face[1]);
            Real* p3_ptr = _mesh->vertexPointer(face[2]);

            Real qm = vertexConstraintInertia(i);
            Real p1m = vertexConstraintInertia(face[0]);
            Real p2m = vertexConstraintInertia(face[1]);
            Real p3m = vertexConstraintInertia(face[2]);

            
            // std::cout << "  SELF COLLISION WITH VERTEX " << i << " WITH FACE " << face_index << "!" << std::endl;
            // std::cout << "  Tet indices: " << tetMesh()->element(hits.begin()->prim_index).transpose() << std::endl;
            // std::cout << "  Face indices: " << face.transpose() << std::endl;
            // std::cout << "  Vertex: " << _mesh->vertex(i).transpose() << 
            //     "  Face:\n\t" << _mesh->vertex(face[0]).transpose() << ",\n\t" << _mesh->vertex(face[1]).transpose()  << ",\n\t" << _mesh->vertex(face[2]).transpose() << std::endl;
            std::vector<Solver::DeformableDeformableCollisionConstraint>& constraint_vec = _constraints.template get<Solver::DeformableDeformableCollisionConstraint>();
            constraint_vec.emplace_back(i, q_ptr, qm, face[0], p1_ptr, p1m, face[1], p2_ptr, p2m, face[2], p3_ptr, p3m);

            using ConstraintRefType = Solver::ConstraintReference<Solver::DeformableDeformableCollisionConstraint>;
            _solver.addConstraintProjector(_sim->dt(), ConstraintRefType(constraint_vec, constraint_vec.size()-1));
        }
    }
    
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
typename SolverType::projector_reference_container_type
XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::_gatherProjectorsForLocalCollisionIterations()
{
    // create a container to store all the constraint projectors that we should re-project
    typename SolverType::projector_reference_container_type proj_to_reproject;

    // go through each collision constraint and find the ones that were actually projected (lambda != 0)
    using StaticCollisionProjectorType = Solver::ConstraintProjector<IsFirstOrder, Solver::StaticDeformableCollisionConstraint>;
    using StaticCollisionProjectorTypeRef = Solver::ConstraintProjectorReference<StaticCollisionProjectorType>;
    std::vector<StaticCollisionProjectorType>& collision_projectors = _solver.template getConstraintProjectorsOfType<StaticCollisionProjectorType>();

    for (unsigned i = 0; i < collision_projectors.size(); i++)
    {
        // add all collision constraints to be re-projected - this is necessary to maintain a consistent contact set
        proj_to_reproject.template emplace_back<StaticCollisionProjectorTypeRef>(collision_projectors, i);

        // if the collision constraint was violated last frame and projected, then we want to perform local iterations in its local area
        if (collision_projectors[i].lambda() != 0)
        {
            // get the vertices affected by this collision constraint
            const std::vector<Solver::PositionReference>& positions = collision_projectors[i].positions();

            if constexpr (std::is_same_v<typename SolverType::projector_type_list, typename XPBDMeshObjectConstraintConfigurations<IsFirstOrder>::StableNeohookeanCombined::projector_type_list>)
            {
                using DevHydProjectorType = Solver::CombinedConstraintProjector<IsFirstOrder, Solver::DeviatoricConstraint, Solver::HydrostaticConstraint>;
                using DevHydProjectorTypeRef = Solver::ConstraintProjectorReference<DevHydProjectorType>;
                std::vector<DevHydProjectorType>& elastic_projectors = _solver.template getConstraintProjectorsOfType<DevHydProjectorType>();

                // for each of the positions in the collision face, get the elements that they are attached to and add the elastic per-element constraints
                // to be reprojected
                for (const auto& position : positions)
                {
                    for (const auto& element_index : tetMesh()->vertexAttachedElements(position.index))
                    {
                        proj_to_reproject.template emplace_back<DevHydProjectorTypeRef>(elastic_projectors, element_index);
                    }
                }
            }
            else if (std::is_same_v<typename SolverType::projector_type_list, typename XPBDMeshObjectConstraintConfigurations<IsFirstOrder>::StableNeohookean::projector_type_list>)
            {
                using DevProjectorType = Solver::ConstraintProjector<IsFirstOrder, Solver::DeviatoricConstraint>;
                using DevProjectorTypeRef = Solver::ConstraintProjectorReference<DevProjectorType>;
                using HydProjectorType = Solver::ConstraintProjector<IsFirstOrder, Solver::HydrostaticConstraint>;
                using HydProjectorTypeRef = Solver::ConstraintProjectorReference<HydProjectorType>;

                std::vector<DevProjectorType>& dev_projectors = _solver.template getConstraintProjectorsOfType<DevProjectorType>();
                std::vector<HydProjectorType>& hyd_projectors = _solver.template getConstraintProjectorsOfType<HydProjectorType>();
                for (const auto& position : positions)
                {
                    for (const auto& element_index : tetMesh()->vertexAttachedElements(position.index))
                    {
                        proj_to_reproject.template emplace_back<DevProjectorTypeRef>(dev_projectors, element_index);
                        proj_to_reproject.template emplace_back<HydProjectorTypeRef>(hyd_projectors, element_index);
                    }
                }
            }
        }
    }

    return proj_to_reproject;
}

#ifdef HAVE_CUDA

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::createGPUResource()
{
    if (!_gpu_resource)
    {
        _gpu_resource = std::make_unique<Sim::XPBDMeshObjectGPUResource>(this);
        _gpu_resource->allocate();
    }
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
XPBDMeshObjectGPUResource* XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::gpuResource()
{
    assert(_gpu_resource);
    // TODO: see if we can remove this dynamic_cast somehow
    return dynamic_cast<XPBDMeshObjectGPUResource*>(_gpu_resource.get());
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
const XPBDMeshObjectGPUResource* XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::gpuResource() const
{
    assert(_gpu_resource);
    // TODO: see if we can remove this dynamic_cast somehow
    return dynamic_cast<const XPBDMeshObjectGPUResource*>(_gpu_resource.get());
}
#endif

} // namespace Sim




/////////////////////////////////////////////////////////////////
// Explicit template instantiations
////////////////////////////////////////////////////////////////

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

namespace Sim {

// TODO: find a way to automate this!
using SolverTypesStableNeohookean = XPBDObjectSolverTypes<false, typename XPBDMeshObjectConstraintConfigurations<false>::StableNeohookean::projector_type_list>;
using SolverTypesStableNeohookeanCombined = XPBDObjectSolverTypes<false, typename XPBDMeshObjectConstraintConfigurations<false>::StableNeohookeanCombined::projector_type_list>;
using StableNeohookeanConstraints = typename XPBDMeshObjectConstraintConfigurations<false>::StableNeohookean::constraint_type_list;
using StableNeohookeanCombinedConstraints = typename XPBDMeshObjectConstraintConfigurations<false>::StableNeohookeanCombined::constraint_type_list;

// Stable Neohookean constraint config
template class XPBDMeshObject_<false, SolverTypesStableNeohookean::GaussSeidel, StableNeohookeanConstraints>;
template class XPBDMeshObject_<false, SolverTypesStableNeohookean::Jacobi, StableNeohookeanConstraints>;
template class XPBDMeshObject_<false, SolverTypesStableNeohookean::ParallelJacobi, StableNeohookeanConstraints>;

// Stable Neohookean Combined constraint config
template class XPBDMeshObject_<false, SolverTypesStableNeohookeanCombined::GaussSeidel, StableNeohookeanCombinedConstraints>;
template class XPBDMeshObject_<false, SolverTypesStableNeohookeanCombined::Jacobi, StableNeohookeanCombinedConstraints>;
template class XPBDMeshObject_<false, SolverTypesStableNeohookeanCombined::ParallelJacobi, StableNeohookeanCombinedConstraints>;

using FirstOrderSolverTypesStableNeohookean = XPBDObjectSolverTypes<true, typename XPBDMeshObjectConstraintConfigurations<true>::StableNeohookean::projector_type_list>;
using FirstOrderSolverTypesStableNeohookeanCombined = XPBDObjectSolverTypes<true, typename XPBDMeshObjectConstraintConfigurations<true>::StableNeohookeanCombined::projector_type_list>;
using FirstOrderStableNeohookeanConstraints = typename XPBDMeshObjectConstraintConfigurations<true>::StableNeohookean::constraint_type_list;
using FirstOrderStableNeohookeanCombinedConstraints = typename XPBDMeshObjectConstraintConfigurations<true>::StableNeohookeanCombined::constraint_type_list;
template class XPBDMeshObject_<true, FirstOrderSolverTypesStableNeohookean::GaussSeidel, FirstOrderStableNeohookeanConstraints>;
template class XPBDMeshObject_<true, FirstOrderSolverTypesStableNeohookean::Jacobi, FirstOrderStableNeohookeanConstraints>;
template class XPBDMeshObject_<true, FirstOrderSolverTypesStableNeohookean::ParallelJacobi, FirstOrderStableNeohookeanConstraints>;

template class XPBDMeshObject_<true, FirstOrderSolverTypesStableNeohookeanCombined::GaussSeidel, FirstOrderStableNeohookeanCombinedConstraints>;
template class XPBDMeshObject_<true, FirstOrderSolverTypesStableNeohookeanCombined::Jacobi, FirstOrderStableNeohookeanCombinedConstraints>;
template class XPBDMeshObject_<true, FirstOrderSolverTypesStableNeohookeanCombined::ParallelJacobi, FirstOrderStableNeohookeanCombinedConstraints>;
// CTAD
// template<typename SolverType, typename ...ConstraintTypes> XPBDMeshObject(TypeList<ConstraintTypes...>, const Simulation*, const XPBDMeshObjectConfig* config)
//     -> XPBDMeshObject<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>;

} // namespace Sim