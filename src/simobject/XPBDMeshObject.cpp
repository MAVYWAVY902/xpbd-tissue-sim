#include "simobject/XPBDMeshObject.hpp"

#include "common/colors.hpp"

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
#include "solver/constraint/InterObjectDeformableCollisionConstraint.hpp"
#include "solver/constraint/HydrostaticConstraint.hpp"
#include "solver/constraint/DeviatoricConstraint.hpp"
#include "solver/constraint/NerveStretchConstraint.hpp" 
#include "solver/constraint/NerveTumorAdhesionConstraint.hpp"
#include "solver/constraint/InterDeformDeformAdhesionConstraint.hpp"

#include <chrono> 
#include "solver/xpbd_projector/CombinedConstraintProjector.hpp"
#include "solver/xpbd_projector/ConstraintProjector.hpp"
#include "solver/xpbd_projector/RigidBodyConstraintProjector.hpp"
#include "utils/MeshUtils.hpp"
#include "utils/FileUtils.hpp"

#include "geometry/DeformableMeshSDF.hpp"

#ifdef HAVE_CUDA
#include "gpu/resource/XPBDMeshObjectGPUResource.hpp"
#endif

namespace Sim
{

template<bool IsFirstOrder>
XPBDMeshObject_Base_<IsFirstOrder>::XPBDMeshObject_Base_(const Simulation* sim, const ConfigType* config)
    : Object(sim, config), TetMeshObject(config, config)
{
    for (const auto& mat_name : config->materials())
    {
        _materials.push_back(sim->getMaterial(mat_name));
    }

    if (_materials.size() == 0)
    {
        std::cerr << KRED << BOLD << "FATAL: " << RST << KRED << "No materials were specified!" << RST << std::endl;
        assert(0);
    }
}

template<bool IsFirstOrder>
void XPBDMeshObject_Base_<IsFirstOrder>::createSDF()
{
    if (!_sdf.has_value())
    {
        _sdf.emplace(this, _sim->embreeScene());
    }
    else
    {
    }
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

    // inter-object collision flag
    _inter_object_collisions = config->interObjectCollisions();

    // local collision iterations
    _num_local_collision_iters = config->numLocalCollisionIters();

    // filename that has info on element classes (optional)
    _element_classes_filename = config->elementClassesFilename();

    // get the damping multiplier for 1st-order objects
    if constexpr (IsFirstOrder)
    {
        _damping_multiplier = config->dampingMultiplier();
        _adjust_b_to_material = config->adjustDampingToMaterial();
    }

    // capture any fixed-vertices specified in the config (0-based indices)
    _initial_fixed_vertices = config->fixedVertices();
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
    

    // add the class property to the element mesh, with default value 0
    tetMesh()->template addElementProperty<int>("class", 0);
    tetMesh()->template addFaceProperty<int>("class", 0);
    tetMesh()->template addVertexProperty<int>("class", 0);

    if (_element_classes_filename.has_value())
    {
        Geometry::MeshProperty<int>& elem_class_prop = tetMesh()-> template getElementProperty<int>("class");
        Geometry::MeshProperty<int>& face_class_prop = tetMesh()-> template getFaceProperty<int>("class");
        Geometry::MeshProperty<int>& vert_class_prop = tetMesh()-> template getVertexProperty<int>("class");

        std::vector<int> elem_classes = FileUtils::readVectorFromFile<int>(_element_classes_filename.value());

        assert(elem_classes.size() == (unsigned)tetMesh()->numElements() && "Element classes file has a different number of elements than the mesh!");
        
        // set the class for each element in the mesh
        for (unsigned i = 0; i < elem_classes.size(); i++)
        {
            elem_class_prop.set(i, elem_classes[i]);
        }

        // set the class for each surface face in the mesh, from the element class for the element containing the surface face
        for (int i = 0; i < tetMesh()->numFaces(); i++)
        {
            int elem_index = tetMesh()->elementWithFace(i);
            face_class_prop.set(i, elem_classes[elem_index]);
        }

        // set the class for each vertex in the mesh, based on the element class for the element(s) containing the vertex
        // since multiple elements share the same vertices, the maximum of the element classes is used for each vertex
        for (int i = 0; i < tetMesh()->numVertices(); i++)
        {
            // get elements attached to the vertex
            std::vector<int> attached_elements = tetMesh()->vertexAttachedElements(i);
            // find the max element class of these attached elements
            int max_class = 0;
            for (const auto& elem_index : attached_elements)
            {
                max_class = std::max(max_class, elem_classes[elem_index]);
            }
            vert_class_prop.set(i, max_class);
        }
    }

    _solver.setup();

    // Reserve space for adhesion constraints
    // Estimate: assume each vertex might have adhesion constraints to several triangles
    const int estimated_adhesion_constraints = _mesh->numVertices() * 5; // conservative estimate
    _constraints.template reserve<Solver::NerveTumorAdhesionConstraint>(estimated_adhesion_constraints);

    // initialize the previous vertices matrix once we've loaded the mesh
    _previous_vertices = _mesh->vertices();
    _vertex_velocities = Geometry::Mesh::VerticesMat::Zero(3, _mesh->numVertices());
    _vertex_velocities.colwise() = _initial_velocity;

    _calculatePerVertexQuantities();
    
    // Apply any fixed vertices that were specified in the YAML config
    // MUST be after _calculatePerVertexQuantities() which allocates _is_fixed_vertex
    if (!_initial_fixed_vertices.empty())
    {
        for (const auto& v : _initial_fixed_vertices)
        {
            if (v >= 0 && v < _mesh->numVertices())
            {
                this->fixVertex(v);
                std::cout << "[XPBDMeshObject] Fixed vertex " << v << " from YAML config" << std::endl;
            }
            else
            {
                std::cerr << "[XPBDMeshObject] Warning: fixed-vertex index " << v << " out of range (0.." << _mesh->numVertices()-1 << ")" << std::endl;
            }
        }
    }
    
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
Solver::ConstraintProjectorReference<Solver::ConstraintProjector<IsFirstOrder, Solver::InterObjectDeformableCollisionConstraint>>
XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::addInterObjectCollisionConstraint(
    int vertex_index,
    int other_face_v1, Real* other_v1_ptr, Real other_m1,
    int other_face_v2, Real* other_v2_ptr, Real other_m2,
    int other_face_v3, Real* other_v3_ptr, Real other_m3)
{
    // Get the vertex information from THIS object
    Real* vertex_ptr = _mesh->vertexPointer(vertex_index);
    Real vertex_mass = vertexConstraintInertia(vertex_index);

    // Create the inter-object collision constraint
    // Constraint layout: (vertex from this object, face vertices from other object)
    std::vector<Solver::InterObjectDeformableCollisionConstraint>& constraint_vec = 
        _constraints.template get<Solver::InterObjectDeformableCollisionConstraint>();
    
    constraint_vec.emplace_back(
        vertex_index, vertex_ptr, vertex_mass,           // Vertex from THIS object
        other_face_v1, other_v1_ptr, other_m1,           // Face vertex 1 from OTHER object
        other_face_v2, other_v2_ptr, other_m2,           // Face vertex 2 from OTHER object
        other_face_v3, other_v3_ptr, other_m3            // Face vertex 3 from OTHER object
    );

    using ConstraintRefType = Solver::ConstraintReference<Solver::InterObjectDeformableCollisionConstraint>;
    return _solver.addConstraintProjector(_sim->dt(), ConstraintRefType(constraint_vec, constraint_vec.size()-1));
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::clearCollisionConstraints()
{
    // set any collision constraint projectors in the solver invalid
    // NOTE: because the collision constraint
    using StaticCollisionConstraintType = Solver::ConstraintProjector<IsFirstOrder, Solver::StaticDeformableCollisionConstraint>;
    using DeformableCollisionConstraintType = Solver::ConstraintProjector<IsFirstOrder, Solver::DeformableDeformableCollisionConstraint>;
    using InterObjectCollisionConstraintType = Solver::ConstraintProjector<IsFirstOrder, Solver::InterObjectDeformableCollisionConstraint>;
    using RigidCollisionConstraintType = Solver::RigidBodyConstraintProjector<IsFirstOrder, Solver::RigidDeformableCollisionConstraint>;
    _solver.template clearProjectorsOfType<StaticCollisionConstraintType>();
    _solver.template clearProjectorsOfType<DeformableCollisionConstraintType>();
    _solver.template clearProjectorsOfType<InterObjectCollisionConstraintType>();
    _solver.template clearProjectorsOfType<RigidCollisionConstraintType>();

    // clear the collision constraints lists
    _constraints.template clear<Solver::StaticDeformableCollisionConstraint>();
    _constraints.template clear<Solver::DeformableDeformableCollisionConstraint>();
    _constraints.template clear<Solver::InterObjectDeformableCollisionConstraint>();
    _constraints.template clear<Solver::RigidDeformableCollisionConstraint>();


}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::clearAdhesionConstraints()
{
    // Clear nerve-tumor adhesion constraints
    using NerveTumorAdhesionConstraintType = Solver::ConstraintProjector<IsFirstOrder, Solver::NerveTumorAdhesionConstraint>;
    _solver.template clearProjectorsOfType<NerveTumorAdhesionConstraintType>();
    _constraints.template clear<Solver::NerveTumorAdhesionConstraint>();
    
    // Clear inter-deform adhesion constraints
    using InterDeformAdhesionConstraintType = Solver::ConstraintProjector<IsFirstOrder, Solver::InterDeformDeformAdhesionConstraint>;
    _solver.template clearProjectorsOfType<InterDeformAdhesionConstraintType>();
    _constraints.template clear<Solver::InterDeformDeformAdhesionConstraint>();
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::checkAndBreakAdhesionConstraints(Real break_distance)
{
    static int call_count = 0;
    call_count++;

    // Get nerve-tumor adhesion constraint projectors
    using NerveTumorAdhesionConstraintType = Solver::ConstraintProjector<IsFirstOrder, Solver::NerveTumorAdhesionConstraint>;
    auto& nerve_tumor_projectors = _solver.template getConstraintProjectorsOfType<NerveTumorAdhesionConstraintType>();
    
    // Get inter-deform adhesion constraint projectors
    using InterDeformAdhesionConstraintType = Solver::ConstraintProjector<IsFirstOrder, Solver::InterDeformDeformAdhesionConstraint>;
    auto& inter_deform_projectors = _solver.template getConstraintProjectorsOfType<InterDeformAdhesionConstraintType>();
    
    // Skip if no adhesion constraints
    if (nerve_tumor_projectors.empty() && inter_deform_projectors.empty()) return;
    
    // Count active constraints and gather statistics every 3000 calls
    if (call_count % 3000 == 0) {
        int nerve_tumor_active = 0;
        int inter_deform_active = 0;
        Real min_distance = 1e6;
        Real max_distance = 0.0;
        Real avg_distance = 0.0;
        Real min_ratio = 1e6;
        Real max_ratio = 0.0;
        Real avg_ratio = 0.0;
        int total_active = 0;
        
        // Count nerve-tumor adhesions
        for (size_t i = 0; i < nerve_tumor_projectors.size(); ++i) {
            if (nerve_tumor_projectors[i].isValid()) {
                nerve_tumor_active++;
                total_active++;
                
                const auto& constraint_ref = nerve_tumor_projectors[i].constraint();
                const auto* constraint = &constraint_ref.get();
                if (constraint) {
                    Real dist = constraint->getCurrentDistance();
                    Real rest_gap = constraint->getRestGap();
                    Real ratio = (rest_gap > 0) ? (dist / rest_gap) : 0.0;
                    
                    min_distance = std::min(min_distance, dist);
                    max_distance = std::max(max_distance, dist);
                    avg_distance += dist;
                    
                    min_ratio = std::min(min_ratio, ratio);
                    max_ratio = std::max(max_ratio, ratio);
                    avg_ratio += ratio;
                }
            }
        }
        
        // Count inter-deform adhesions
        for (size_t i = 0; i < inter_deform_projectors.size(); ++i) {
            if (inter_deform_projectors[i].isValid()) {
                inter_deform_active++;
                total_active++;
                
                const auto& constraint_ref = inter_deform_projectors[i].constraint();
                const auto* constraint = &constraint_ref.get();
                if (constraint) {
                    Real dist = constraint->getCurrentDistance();
                    Real rest_gap = constraint->getRestGap();
                    Real ratio = (rest_gap > 0) ? (dist / rest_gap) : 0.0;
                    
                    min_distance = std::min(min_distance, dist);
                    max_distance = std::max(max_distance, dist);
                    avg_distance += dist;
                    
                    min_ratio = std::min(min_ratio, ratio);
                    max_ratio = std::max(max_ratio, ratio);
                    avg_ratio += ratio;
                }
            }
        }
        
        if (total_active > 0) {
            avg_distance /= total_active;
            avg_ratio /= total_active;
            
            std::cout << "[active adhesion counter] Object: " << this->name() 
                      << " | Step #" << call_count 
                      << "\n  | Nerve-tumor: " << nerve_tumor_active << " / " << nerve_tumor_projectors.size()
                      << "\n  | Inter-deform: " << inter_deform_active << " / " << inter_deform_projectors.size()
                      << "\n  | Total active: " << total_active
                      << "\n  | Distances: min=" << min_distance << "m, max=" << max_distance 
                      << "m, avg=" << avg_distance << "m"
                      << "\n  | Strain ratios: min=" << min_ratio << ", max=" << max_ratio 
                      << ", avg=" << avg_ratio << " (strain-based breaking)\n";
        }
    }
    
    // Check and break nerve-tumor adhesion constraints
    std::vector<int> nerve_tumor_to_invalidate;
    for (size_t i = 0; i < nerve_tumor_projectors.size(); ++i) {
        auto& projector = nerve_tumor_projectors[i];
        if (!projector.isValid()) continue;
        
        const auto& constraint_ref = projector.constraint();
        const auto* constraint = &constraint_ref.get();
        if (constraint && constraint->shouldBreak()) {
            nerve_tumor_to_invalidate.push_back(static_cast<int>(i));
        }
    }
    
    // Check and break inter-deform adhesion constraints
    std::vector<int> inter_deform_to_invalidate;
    for (size_t i = 0; i < inter_deform_projectors.size(); ++i) {
        auto& projector = inter_deform_projectors[i];
        if (!projector.isValid()) continue;
        
        const auto& constraint_ref = projector.constraint();
        const auto* constraint = &constraint_ref.get();
        if (constraint && constraint->shouldBreak()) {
            inter_deform_to_invalidate.push_back(static_cast<int>(i));
        }
    }
    
    // Invalidate nerve-tumor adhesion projectors that should break
    for (int idx : nerve_tumor_to_invalidate) {
        _solver.template setProjectorValidity<NerveTumorAdhesionConstraintType>(idx, false);
        
        // Update visualization properties
        auto& projector = nerve_tumor_projectors[idx];
        const auto& constraint_ref = projector.constraint();
        const auto* constraint = &constraint_ref.get();
        if (constraint && this->mesh()->template hasVertexProperty<bool>("has_adhesion_constraint")) {
            int vertex_v = constraint->positions()[0].index;
            auto& adhesion_prop = this->mesh()->template getVertexProperty<bool>("has_adhesion_constraint");
            
            // Check if vertex has any remaining active constraints
            bool has_active = false;
            for (size_t j = 0; j < nerve_tumor_projectors.size(); ++j) {
                if (j != static_cast<size_t>(idx) && nerve_tumor_projectors[j].isValid()) {
                    const auto& other_ref = nerve_tumor_projectors[j].constraint();
                    if (other_ref.get().positions()[0].index == vertex_v) {
                        has_active = true;
                        break;
                    }
                }
            }
            
            if (!has_active) {
                adhesion_prop.set(vertex_v, false);
                std::cout << "[viz] Removed nerve-tumor adhesion marker from vertex " << vertex_v << "\n";
            }
        }
    }
    
    // Invalidate inter-deform adhesion projectors that should break
    for (int idx : inter_deform_to_invalidate) {
        _solver.template setProjectorValidity<InterDeformAdhesionConstraintType>(idx, false);
        
        // Update visualization properties
        auto& projector = inter_deform_projectors[idx];
        const auto& constraint_ref = projector.constraint();
        const auto* constraint = &constraint_ref.get();
        if (constraint && this->mesh()->template hasVertexProperty<bool>("has_adhesion_constraint")) {
            int vertex_v = constraint->positions()[0].index;
            auto& adhesion_prop = this->mesh()->template getVertexProperty<bool>("has_adhesion_constraint");
            
            // Check if vertex has any remaining active constraints
            bool has_active = false;
            for (size_t j = 0; j < inter_deform_projectors.size(); ++j) {
                if (j != static_cast<size_t>(idx) && inter_deform_projectors[j].isValid()) {
                    const auto& other_ref = inter_deform_projectors[j].constraint();
                    if (other_ref.get().positions()[0].index == vertex_v) {
                        has_active = true;
                        break;
                    }
                }
            }
            
            if (!has_active) {
                adhesion_prop.set(vertex_v, false);
                // std::cout << "[viz] Removed inter-deform adhesion marker from vertex " << vertex_v << "\n";
            }
        }
    }
    
    // Print summary
    // if (!nerve_tumor_to_invalidate.empty() || !inter_deform_to_invalidate.empty()) {
    //     std::cout << "[adhesion BREAK] Object: " << this->name()
    //               << " | Broke " << nerve_tumor_to_invalidate.size() << " nerve-tumor"
    //               << " + " << inter_deform_to_invalidate.size() << " inter-deform adhesions\n";
    // }
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

// NEW: addNerveStretchConstraint
template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
Solver::ConstraintProjectorReference<
    Solver::ConstraintProjector<IsFirstOrder, Solver::NerveStretchConstraint>>
XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>
    ::addNerveStretchConstraint(int v0, int v1, Real rest_len, Real alpha)
{
    // 1. Two vertices' pointers
    Real* p0 = _mesh->vertexPointer(v0);
    Real* p1 = _mesh->vertexPointer(v1);

    // 2. Constraint masses (same approach as other constraints)
    Real m0 = vertexConstraintInertia(v0);
    Real m1 = vertexConstraintInertia(v1);

    // 3. Add to constraints array
    auto& vec = _constraints.template get<Solver::NerveStretchConstraint>();
    vec.emplace_back(
        v0, p0, m0,
        v1, p1, m1,
        rest_len,
        alpha
    );

    // 4. Tell solver about the new constraint
    using RefType = Solver::ConstraintReference<Solver::NerveStretchConstraint>;
    return _solver.addConstraintProjector(
        _sim->dt(),
        RefType(vec, vec.size() - 1)
    );
}

// NEW: addNerveBendingConstraint
template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
Solver::ConstraintProjectorReference<
    Solver::ConstraintProjector<IsFirstOrder, Solver::NerveBendingConstraint>>
XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>
    ::addNerveBendingConstraint(int v0, int v1, int v2, Real rest_curvature, Real alpha)
{
    // 1. Three vertices' pointers
    Real* p0 = _mesh->vertexPointer(v0);
    Real* p1 = _mesh->vertexPointer(v1);
    Real* p2 = _mesh->vertexPointer(v2);

    // 2. Constraint masses (same approach as other constraints)
    Real m0 = vertexConstraintInertia(v0);
    Real m1 = vertexConstraintInertia(v1);
    Real m2 = vertexConstraintInertia(v2);

    // 3. Add to constraints array
    auto& vec = _constraints.template get<Solver::NerveBendingConstraint>();
    vec.emplace_back(
        v0, p0, m0,
        v1, p1, m1,
        v2, p2, m2,
        rest_curvature,
        alpha
    );

    // 4. Tell solver about the new constraint
    using RefType = Solver::ConstraintReference<Solver::NerveBendingConstraint>;
    return _solver.addConstraintProjector(
        _sim->dt(),
        RefType(vec, vec.size() - 1)
    );
}

// NEW: addNerveTumorAdhesionConstraint
template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
Solver::ConstraintProjectorReference<
    Solver::ConstraintProjector<IsFirstOrder, Solver::NerveTumorAdhesionConstraint>>
XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>
    ::addNerveTumorAdhesionConstraint(XPBDMeshObject_Base_<IsFirstOrder>* nerve_obj, int nerve_v,
                                     int tri_v1, int tri_v2, int tri_v3, 
                                     Real rest_gap, Real break_ratio, Real alpha)
{
    // 1. Get nerve vertex position pointer and mass from the NERVE object (not tumor!)
    Real* nerve_p = nerve_obj->mesh()->vertexPointer(nerve_v);
    Real nerve_m = nerve_obj->vertexConstraintInertia(nerve_v);
    
    // 2. Get TUMOR vertex position pointers and masses from THIS object
    Real* tri_p1 = _mesh->vertexPointer(tri_v1);
    Real* tri_p2 = _mesh->vertexPointer(tri_v2);
    Real* tri_p3 = _mesh->vertexPointer(tri_v3);
    
    Real tri_m1 = vertexConstraintInertia(tri_v1);
    Real tri_m2 = vertexConstraintInertia(tri_v2);
    Real tri_m3 = vertexConstraintInertia(tri_v3);

    // 2. Add to constraints array
    auto& vec = _constraints.template get<Solver::NerveTumorAdhesionConstraint>();
    vec.emplace_back(
        nerve_v, nerve_p, nerve_m,
        tri_v1, tri_p1, tri_m1,
        tri_v2, tri_p2, tri_m2,
        tri_v3, tri_p3, tri_m3,
        rest_gap,
        break_ratio,
        alpha
    );

    // 3. Mark nerve vertex as having adhesion constraint for visualization
    if (!_mesh->template hasVertexProperty<bool>("has_adhesion_constraint")) {
        _mesh->template addVertexProperty<bool>("has_adhesion_constraint", false);
        std::cout << "[viz] Created adhesion constraint property for mesh " << _mesh.get() << "\n";
    }
    auto& adhesion_prop = _mesh->template getVertexProperty<bool>("has_adhesion_constraint");
    adhesion_prop.set(nerve_v, true);
    std::cout << "[viz] Marked nerve vertex " << nerve_v << " as having adhesion constraint on mesh " << _mesh.get() << "\n";

    // 4. Tell solver about the new constraint
    using RefType = Solver::ConstraintReference<Solver::NerveTumorAdhesionConstraint>;
    return _solver.addConstraintProjector(
        _sim->dt(),
        RefType(vec, vec.size() - 1)
    );
}


// NEW: addInterDeformDeformAdhesionConstraint
template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
Solver::ConstraintProjectorReference<
    Solver::ConstraintProjector<IsFirstOrder, Solver::InterDeformDeformAdhesionConstraint>>
XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>
    ::addInterDeformDeformAdhesionConstraint(XPBDMeshObject_Base_<IsFirstOrder>* other_obj, int vertex_v,
                                            int tri_v1, int tri_v2, int tri_v3, 
                                            Real rest_gap, Real break_ratio, Real alpha)
{
    // 1. Get vertex position pointer and mass from the OTHER object
    Real* vertex_p = other_obj->mesh()->vertexPointer(vertex_v);
    Real vertex_m = other_obj->vertexConstraintInertia(vertex_v);
    
    // 2. Get triangle vertex position pointers and masses from THIS object
    Real* tri_p1 = _mesh->vertexPointer(tri_v1);
    Real* tri_p2 = _mesh->vertexPointer(tri_v2);
    Real* tri_p3 = _mesh->vertexPointer(tri_v3);
    
    Real tri_m1 = vertexConstraintInertia(tri_v1);
    Real tri_m2 = vertexConstraintInertia(tri_v2);
    Real tri_m3 = vertexConstraintInertia(tri_v3);

    // 3. Add to constraints array
    auto& vec = _constraints.template get<Solver::InterDeformDeformAdhesionConstraint>();
    vec.emplace_back(
        vertex_v, vertex_p, vertex_m,
        tri_v1, tri_p1, tri_m1,
        tri_v2, tri_p2, tri_m2,
        tri_v3, tri_p3, tri_m3,
        rest_gap,
        break_ratio,
        alpha
    );

    // 4. Mark vertex as having adhesion constraint for visualization
    if (!_mesh->template hasVertexProperty<bool>("has_adhesion_constraint")) {
        _mesh->template addVertexProperty<bool>("has_adhesion_constraint", false);
        std::cout << "[viz] Created inter-deform adhesion constraint property for mesh " << _mesh.get() << "\n";
    }
    auto& adhesion_prop = _mesh->template getVertexProperty<bool>("has_adhesion_constraint");
    adhesion_prop.set(vertex_v, true);
    // std::cout << "[viz] Marked vertex " << vertex_v << " as having inter-deform adhesion constraint on mesh " << _mesh.get() << "\n";

    // 5. Tell solver about the new constraint
    using RefType = Solver::ConstraintReference<Solver::InterDeformDeformAdhesionConstraint>;
    return _solver.addConstraintProjector(
        _sim->dt(),
        RefType(vec, vec.size() - 1)
    );
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
int XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::numInterDeformAdhesionConstraints() const
{
    // Get the vector of inter-deform adhesion constraints
    const auto& vec = _constraints.template get<Solver::InterDeformDeformAdhesionConstraint>();
    return static_cast<int>(vec.size());
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::collectAdhesionForces(
    std::vector<Vec3r>& vertex_forces, int vertex_offset) const
{
    int num_forces_collected = 0;
    Real max_force = 0.0;
    
    // Collect forces from NerveTumorAdhesionConstraint projectors
    using NerveTumorProjectorType = Solver::ConstraintProjector<IsFirstOrder, Solver::NerveTumorAdhesionConstraint>;
    const auto& nerve_tumor_projectors = _solver.template getConstraintProjectorsOfType<NerveTumorProjectorType>();
    
    for (const auto& projector : nerve_tumor_projectors)
    {
        if (!projector.isValid()) continue;
        
        // Get constraint forces: F = ∇C^T · λ / dt (1st-order) or ∇C^T · λ / dt² (2nd-order)
        const auto forces = projector.constraintForces();
        const auto& positions = projector.constraint()->positions();
        
        // Accumulate forces onto corresponding vertices
        for (size_t i = 0; i < forces.size() && i < positions.size(); ++i)
        {
            // Find the vertex index in the global mesh
            // positions[i].position_ptr points to the vertex data
            // We need to calculate the index from the pointer offset
            const Real* pos_ptr = positions[i].position_ptr;
            const Real* base_ptr = this->_mesh->vertices().data();
            int local_vertex_idx = (pos_ptr - base_ptr) / 3;  // Each vertex has 3 coordinates
            
            if (local_vertex_idx >= 0 && local_vertex_idx < this->_mesh->numVertices())
            {
                int global_vertex_idx = vertex_offset + local_vertex_idx;
                if (global_vertex_idx >= 0 && global_vertex_idx < static_cast<int>(vertex_forces.size()))
                {
                    vertex_forces[global_vertex_idx] += forces[i];
                    Real force_mag = forces[i].norm();
                    if (force_mag > max_force) max_force = force_mag;
                    num_forces_collected++;
                }
            }
        }
    }
    
    // Collect forces from InterDeformDeformAdhesionConstraint projectors
    using InterDeformProjectorType = Solver::ConstraintProjector<IsFirstOrder, Solver::InterDeformDeformAdhesionConstraint>;
    const auto& inter_deform_projectors = _solver.template getConstraintProjectorsOfType<InterDeformProjectorType>();
    
    for (const auto& projector : inter_deform_projectors)
    {
        if (!projector.isValid()) continue;
        
        const auto forces = projector.constraintForces();
        const auto& positions = projector.constraint()->positions();
        
        for (size_t i = 0; i < forces.size() && i < positions.size(); ++i)
        {
            const Real* pos_ptr = positions[i].position_ptr;
            const Real* base_ptr = this->_mesh->vertices().data();
            int local_vertex_idx = (pos_ptr - base_ptr) / 3;
            
            if (local_vertex_idx >= 0 && local_vertex_idx < this->_mesh->numVertices())
            {
                int global_vertex_idx = vertex_offset + local_vertex_idx;
                if (global_vertex_idx >= 0 && global_vertex_idx < static_cast<int>(vertex_forces.size()))
                {
                    vertex_forces[global_vertex_idx] += forces[i];
                    Real force_mag = forces[i].norm();
                    if (force_mag > max_force) max_force = force_mag;
                    num_forces_collected++;
                }
            }
        }
    }
    
    // Debug output (only once every 100 calls to avoid spam)
    static int call_count = 0;
    call_count++;
    if (call_count % 100 == 0)
    {
        std::cout << "[collectAdhesionForces] nerve_tumor_projectors: " << nerve_tumor_projectors.size()
                  << ", inter_deform_projectors: " << inter_deform_projectors.size()
                  << ", forces_collected: " << num_forces_collected
                  << ", max_force: " << max_force << std::endl;
    }
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

    std::vector<Real> vertex_E(_mesh->numVertices());
    std::vector<Real> vertex_nu(_mesh->numVertices());
    
    const Geometry::MeshProperty<int>& class_prop = tetMesh()->template getElementProperty<int>("class"); 
    for (int i = 0; i < tetMesh()->numElements(); i++)
    {
        // get the material for this element
        int material_ind = class_prop.get(i);
        if (static_cast<unsigned>(material_ind) >= _materials.size())
        {
            std::cout << KYEL << BOLD << "WARNING: " << RST << KYEL << "Only " << _materials.size() << " materials were specified, but element " <<
                i << " has class " << material_ind << ". (Specify more materials in the config file)" << RST << std::endl;
            
            // set the material index to the largest valid index
            material_ind = _materials.size() - 1;
        }
        const ElasticMaterial& material = _materials[material_ind];

        const Eigen::Vector4i& element = tetMesh()->element(i);
        // compute volume from X
        const Real volume = tetMesh()->elementVolume(i);
        // _vols(i) = vol;

        // compute mass of element
        const Real element_mass = volume * material.density();
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

        // add material properties to each of its vertices
        vertex_E[element[0]] += material.E() / tetMesh()->vertexAttachedElements(element[0]).size();
        vertex_E[element[1]] += material.E() / tetMesh()->vertexAttachedElements(element[1]).size();
        vertex_E[element[2]] += material.E() / tetMesh()->vertexAttachedElements(element[2]).size();
        vertex_E[element[3]] += material.E() / tetMesh()->vertexAttachedElements(element[3]).size();

        vertex_nu[element[0]] += material.nu() / tetMesh()->vertexAttachedElements(element[0]).size();
        vertex_nu[element[1]] += material.nu() / tetMesh()->vertexAttachedElements(element[1]).size();
        vertex_nu[element[2]] += material.nu() / tetMesh()->vertexAttachedElements(element[2]).size();
        vertex_nu[element[3]] += material.nu() / tetMesh()->vertexAttachedElements(element[3]).size();
    }

    // Check for 1D meshes with no tetrahedral elements - assign fallback masses
    bool has_zero_masses = true;
    for (int i = 0; i < _mesh->numVertices(); i++) {
        if (_vertex_masses[i] > 0) {
            has_zero_masses = false;
            break;
        }
    }
    
    if (has_zero_masses && tetMesh()->numElements() == 0) {
        // For 1D meshes, assign default mass based on material density
        const ElasticMaterial& default_material = _materials[0];
        const Real default_mass = default_material.density() * 1e-6;  // Small unit mass
        
        for (int i = 0; i < _mesh->numVertices(); i++) {
            _vertex_masses[i] = default_mass;
            // Also assign fallback material properties for 1D meshes
            vertex_E[i] = default_material.E();
            vertex_nu[i] = default_material.nu();
        }
    }

    // for 1st-order objects, calculate per-vertex damping
    if constexpr (IsFirstOrder)
    {
        _vertex_B.resize(_mesh->numVertices());
        
        // Check if this is a Nerve-Only configuration (no elastic material constraints)
        if constexpr (std::is_same_v<typename SolverType::projector_type_list, typename XPBDMeshObjectConstraintConfigurations<IsFirstOrder>::NerveOnly::projector_type_list>)
        {
            // For Nerve-Only: use unit damping independent of volume
            for (int i = 0; i < _mesh->numVertices(); i++)
            {
                _vertex_B[i] = _damping_multiplier;  // Simple damping, no volume dependency
            }
        }
        else
        {
            // Standard volume-based damping for elastic materials
            for (int i = 0; i < _mesh->numVertices(); i++)
            {
                if (_adjust_b_to_material)
                {
                    _vertex_B[i] = _vertex_volumes[i] * _damping_multiplier * vertex_E[i] / (1+vertex_nu[i]);
                }
                else
                {
                    _vertex_B[i] = _vertex_volumes[i] * _damping_multiplier;
                }
            }
        }
    }
    
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::_createElasticConstraints()
{
    // Only reserve space for elastic constraints if they're in our constraint configuration
    if constexpr (std::is_same_v<typename SolverType::projector_type_list, typename XPBDMeshObjectConstraintConfigurations<IsFirstOrder>::StableNeohookean::projector_type_list> ||
                  std::is_same_v<typename SolverType::projector_type_list, typename XPBDMeshObjectConstraintConfigurations<IsFirstOrder>::StableNeohookeanCombined::projector_type_list>)
    {
        _constraints.template reserve<Solver::HydrostaticConstraint>(tetMesh()->numElements());
        _constraints.template reserve<Solver::DeviatoricConstraint>(tetMesh()->numElements());
    }

    // create constraint(s) for each element
    const Geometry::MeshProperty<int>& class_prop = tetMesh()->template getElementProperty<int>("class"); 
    for (int i = 0; i < tetMesh()->numElements(); i++)
    {
        // get the material for this element
        int material_ind = class_prop.get(i);
        if ((unsigned)material_ind >= _materials.size())  material_ind = _materials.size()-1;
        const ElasticMaterial& material = _materials[material_ind];

        // get the vertices for the element
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
            hyd_constraint_vec.emplace_back(v0, v0_ptr, m0, v1, v1_ptr, m1, v2, v2_ptr, m2, v3, v3_ptr, m3, material);
            dev_constraint_vec.emplace_back(v0, v0_ptr, m0, v1, v1_ptr, m1, v2, v2_ptr, m2, v3, v3_ptr, m3, material);
            
            using HydConstraintRefType = Solver::ConstraintReference<Solver::HydrostaticConstraint>;
            using DevConstraintRefType = Solver::ConstraintReference<Solver::DeviatoricConstraint>;
            // TODO: support separate constraints - maybe though SeparateConstraintProjector class?.
            _solver.addConstraintProjector(_sim->dt(), HydConstraintRefType(hyd_constraint_vec, hyd_constraint_vec.size()-1));
            _solver.addConstraintProjector(_sim->dt(), DevConstraintRefType(dev_constraint_vec, dev_constraint_vec.size()-1));
            
        }
        // if the constraint configuration is StableNeohookeanCombined, add a combined constraint projector for the hydrostatic and deviatoric constraints
        else if constexpr (std::is_same_v<typename SolverType::projector_type_list, typename XPBDMeshObjectConstraintConfigurations<IsFirstOrder>::StableNeohookeanCombined::projector_type_list>)
        {
            std::vector<Solver::HydrostaticConstraint>& hyd_constraint_vec = _constraints.template get<Solver::HydrostaticConstraint>();
            std::vector<Solver::DeviatoricConstraint>& dev_constraint_vec = _constraints.template get<Solver::DeviatoricConstraint>();
            hyd_constraint_vec.emplace_back(v0, v0_ptr, m0, v1, v1_ptr, m1, v2, v2_ptr, m2, v3, v3_ptr, m3, material);
            dev_constraint_vec.emplace_back(v0, v0_ptr, m0, v1, v1_ptr, m1, v2, v2_ptr, m2, v3, v3_ptr, m3, material);

            using HydConstraintRefType = Solver::ConstraintReference<Solver::HydrostaticConstraint>;
            using DevConstraintRefType = Solver::ConstraintReference<Solver::DeviatoricConstraint>;
            
            _solver.addConstraintProjector(_sim->dt(),
                DevConstraintRefType(dev_constraint_vec, dev_constraint_vec.size()-1), 
                HydConstraintRefType(hyd_constraint_vec, hyd_constraint_vec.size()-1)
            );
        }
        // For NerveOnly configuration, skip adding elastic constraints (hydrostatic/deviatoric)
        else if constexpr (std::is_same_v<typename SolverType::projector_type_list, typename XPBDMeshObjectConstraintConfigurations<IsFirstOrder>::NerveOnly::projector_type_list>)
        {
            // No elastic constraints for NerveOnly configuration
            // Only nerve stretch and bending constraints will be added
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
    // ✅ PERFORMANCE PROFILING: Measure time spent in different parts
    auto start_total = std::chrono::high_resolution_clock::now();
    
    // Reset max distance tracking for all adhesion constraints at the start of each time step
    // This ensures _max_distance_this_step represents "maximum stretch during THIS step"
    // rather than "entire simulation history", preventing false positives in breaking detection
    using AdhesionConstraintType = Solver::ConstraintProjector<IsFirstOrder, Solver::NerveTumorAdhesionConstraint>;
    auto& adhesion_projectors = _solver.template getConstraintProjectorsOfType<AdhesionConstraintType>();
    
    auto start_reset = std::chrono::high_resolution_clock::now();
    int num_adhesion_constraints = 0;
    for (auto& projector : adhesion_projectors) {
        if (projector.isValid()) {
            // Use -> operator on ConstraintReference to access the constraint
            projector.constraint()->resetMaxDistanceThisStep();
            num_adhesion_constraints++;
        }
    }
    auto end_reset = std::chrono::high_resolution_clock::now();

    // set _x_prev to be ready for the next substep
    _previous_vertices = _mesh->vertices();

    auto start_inertia = std::chrono::high_resolution_clock::now();
    _movePositionsInertially();
    auto end_inertia = std::chrono::high_resolution_clock::now();
    
    auto start_projection = std::chrono::high_resolution_clock::now();
    _projectConstraints();
    auto end_projection = std::chrono::high_resolution_clock::now();
    
    auto end_total = std::chrono::high_resolution_clock::now();
    
    // Print timing every 60 frames (once per second at 60 FPS)
    static int frame_count = 0;
    frame_count++;
    if (frame_count % 60 == 0) {
        auto reset_us = std::chrono::duration_cast<std::chrono::microseconds>(end_reset - start_reset).count();
        auto inertia_us = std::chrono::duration_cast<std::chrono::microseconds>(end_inertia - start_inertia).count();
        auto projection_us = std::chrono::duration_cast<std::chrono::microseconds>(end_projection - start_projection).count();
        auto total_us = std::chrono::duration_cast<std::chrono::microseconds>(end_total - start_total).count();
        
        // std::cout << "[PERFORMANCE " << this->name() << " frame " << frame_count << "] "
        //           << "reset=" << reset_us << "us (" << num_adhesion_constraints << " adhesion), "
        //           << "inertia=" << inertia_us << "us, "
        //           << "projection=" << projection_us << "us, "
        //           << "total=" << total_us << "us ("
        //           << (projection_us * 100.0 / total_us) << "% in projection)\n";
    }

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
    Real total_energy = 0;
    
    // Only compute elastic strain energy if we have elastic constraints
    if constexpr (std::is_same_v<typename SolverType::projector_type_list, typename XPBDMeshObjectConstraintConfigurations<IsFirstOrder>::StableNeohookean::projector_type_list> ||
                  std::is_same_v<typename SolverType::projector_type_list, typename XPBDMeshObjectConstraintConfigurations<IsFirstOrder>::StableNeohookeanCombined::projector_type_list>)
    {
        // iterate over all hydrostatic and deviatoric constraints
        _constraints.template for_each_element<Solver::DeviatoricConstraint, Solver::HydrostaticConstraint>([&total_energy](const auto& constraint){
            Real eval;
            constraint.evaluate(&eval);
            total_energy += eval * eval / constraint.alpha();
        });
    }
    // For NerveOnly configuration, strain energy is 0 (no elastic constraints)

    return total_energy;
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
Vec3r XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::elasticForceAtVertex(int index) const
{
    Vec3r total_force = Vec3r::Zero();
    
    // Only compute elastic forces if we have elastic constraints
    if constexpr (std::is_same_v<typename SolverType::projector_type_list, typename XPBDMeshObjectConstraintConfigurations<IsFirstOrder>::StableNeohookean::projector_type_list> ||
                  std::is_same_v<typename SolverType::projector_type_list, typename XPBDMeshObjectConstraintConfigurations<IsFirstOrder>::StableNeohookeanCombined::projector_type_list>)
    {
        // get elements attached to the vertex in the mesh
        const std::vector<int>& _attached_elements = tetMesh()->vertexAttachedElements(index);

        /** TODO: figure out which approach is correct. */
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
    }
    // For NerveOnly configuration, elastic force is 0 (no elastic constraints)

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
    MatXr orig_delC = MatXr::Zero(num_constraints, 3*_mesh->numVertices());
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
    MatXr delC = MatXr::Zero(num_constraints, 3*_mesh->numVertices());
    MatXr grad_delC_i = MatXr::Zero(num_constraints, 3*_mesh->numVertices());
    MatXr hessian_term = MatXr::Zero(3*_mesh->numVertices(), 3*_mesh->numVertices());
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

    // add large values for nodes in contact with the ground (when the ground plane is active)
    // for (int i = 0; i < _mesh->numVertices(); i++)
    // {
    //     if (std::abs(_mesh->vertex(i)[2]) < 1e-10)
    //     {
    //         stiffness_matrix(3*i+2, 3*i+2) += 1e9;
    //     }
    // }

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
            else if constexpr (std::is_same_v<typename SolverType::projector_type_list, typename XPBDMeshObjectConstraintConfigurations<IsFirstOrder>::StableNeohookean::projector_type_list>)
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
            else if constexpr (std::is_same_v<typename SolverType::projector_type_list, typename XPBDMeshObjectConstraintConfigurations<IsFirstOrder>::NerveOnly::projector_type_list>)
            {
                // For NerveOnly configuration, there are no elastic constraints to reproject
                // Only nerve constraints would need reprojection, but they are edge-based not element-based
                // so we skip adding any elastic element constraints here
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

// Nerve-Only constraint config
using SolverTypesNerveOnly = XPBDObjectSolverTypes<false, typename XPBDMeshObjectConstraintConfigurations<false>::NerveOnly::projector_type_list>;
using NerveOnlyConstraints = typename XPBDMeshObjectConstraintConfigurations<false>::NerveOnly::constraint_type_list;

template class XPBDMeshObject_<false, SolverTypesNerveOnly::GaussSeidel, NerveOnlyConstraints>;
template class XPBDMeshObject_<false, SolverTypesNerveOnly::Jacobi, NerveOnlyConstraints>;
template class XPBDMeshObject_<false, SolverTypesNerveOnly::ParallelJacobi, NerveOnlyConstraints>;

// First Order Nerve-Only constraint config
using FirstOrderSolverTypesNerveOnly = XPBDObjectSolverTypes<true, typename XPBDMeshObjectConstraintConfigurations<true>::NerveOnly::projector_type_list>;
using FirstOrderNerveOnlyConstraints = typename XPBDMeshObjectConstraintConfigurations<true>::NerveOnly::constraint_type_list;

template class XPBDMeshObject_<true, FirstOrderSolverTypesNerveOnly::GaussSeidel, FirstOrderNerveOnlyConstraints>;
template class XPBDMeshObject_<true, FirstOrderSolverTypesNerveOnly::Jacobi, FirstOrderNerveOnlyConstraints>;
template class XPBDMeshObject_<true, FirstOrderSolverTypesNerveOnly::ParallelJacobi, FirstOrderNerveOnlyConstraints>;

// CTAD
// template<typename SolverType, typename ...ConstraintTypes> XPBDMeshObject(TypeList<ConstraintTypes...>, const Simulation*, const XPBDMeshObjectConfig* config)
//     -> XPBDMeshObject<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>;

} // namespace Sim