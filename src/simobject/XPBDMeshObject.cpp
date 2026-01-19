#include "simobject/XPBDMeshObject.hpp"

#include "common/colors.hpp"

// OpenMP support for VBD parallel execution
#ifdef ENABLE_VBD_OPENMP
#include <omp.h>
#endif

#include "config/simobject/XPBDMeshObjectConfig.hpp"
#include "config/simobject/FirstOrderXPBDMeshObjectConfig.hpp"

#include "simobject/RigidObject.hpp"
#include "simulation/Simulation.hpp"

#include "solver/xpbd_solver/XPBDGaussSeidelSolver.hpp"
#include "solver/xpbd_solver/XPBDJacobiSolver.hpp"
#include "solver/xpbd_solver/XPBDParallelJacobiSolver.hpp"
#include "solver/constraint/StaticDeformableCollisionConstraint.hpp"
#include "solver/constraint/RigidDeformableCollisionConstraint.hpp"
#include "solver/constraint/InterObjectDeformableCollisionConstraint.hpp"
#include "solver/constraint/DeformableDeformableCollisionConstraint.hpp"
#include "solver/constraint/InterObjectDeformableCollisionConstraint.hpp"
#include "solver/constraint/HydrostaticConstraint.hpp"
#include "solver/constraint/DeviatoricConstraint.hpp"
#include "solver/constraint/NerveStretchConstraint.hpp" 
#include "solver/constraint/NerveTumorAdhesionConstraint.hpp"
#include "solver/constraint/InterDeformDeformAdhesionConstraint.hpp"
#include "solver/constraint/RigidDeformAdhesionConstraint.hpp"

// Graph Coloring for VBD Gauss-Seidel
#include "solver/TetMeshVertexGraph.hpp"
#include "utils/LinearSolver.hpp"

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

    // Save initial rest vertices for VBD elastic force computation
    // This is the TRUE rest configuration (before any deformation)
    _rest_vertices = _mesh->vertices();

    _calculatePerVertexQuantities();
    
    // Initialize tet volumes for energy evaluation in VBD line search
    _tetVolumes.resize(tetMesh()->numElements());
    for (int tet_idx = 0; tet_idx < tetMesh()->numElements(); tet_idx++) {
        const auto& tet = tetMesh()->element(tet_idx);
        Vec3r x0 = _mesh->vertex(tet[0]);
        Vec3r x1 = _mesh->vertex(tet[1]);
        Vec3r x2 = _mesh->vertex(tet[2]);
        Vec3r x3 = _mesh->vertex(tet[3]);
        
        Mat3r edges;
        edges.col(0) = x1 - x0;
        edges.col(1) = x2 - x0;
        edges.col(2) = x3 - x0;
        
        _tetVolumes[tet_idx] = std::abs(edges.determinant()) / 6.0;
    }
    
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
    _vbd_constraints_dirty = true;
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
    _vbd_constraints_dirty = true;
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
    _vbd_constraints_dirty = true;
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
    
    _vbd_constraints_dirty = true;
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
    
    // Get rigid-deform adhesion constraint projectors (needs IsFirstOrder template parameter)
    using RigidDeformAdhesionProjectorType = Solver::RigidBodyConstraintProjector<IsFirstOrder, Solver::RigidDeformAdhesionConstraint>;
    auto& rigid_deform_projectors = _solver.template getConstraintProjectorsOfType<RigidDeformAdhesionProjectorType>();
    
    // Skip if no adhesion constraints
    if (nerve_tumor_projectors.empty() && inter_deform_projectors.empty() && rigid_deform_projectors.empty()) return;
    
    // Count active constraints and gather statistics every 3000 calls
    if (call_count % 3000 == 0) {
        int nerve_tumor_active = 0;
        int inter_deform_active = 0;
        int rigid_deform_active = 0;
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
        
        // Count rigid-deform adhesions
        for (size_t i = 0; i < rigid_deform_projectors.size(); ++i) {
            if (rigid_deform_projectors[i].isValid()) {
                rigid_deform_active++;
                total_active++;
                
                const auto& constraint_ref = rigid_deform_projectors[i].constraint();
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
                      << "\n  | Rigid-deform: " << rigid_deform_active << " / " << rigid_deform_projectors.size()
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
    
    // Check and break rigid-deform adhesion constraints
    std::vector<int> rigid_deform_to_invalidate;
    for (size_t i = 0; i < rigid_deform_projectors.size(); ++i) {
        auto& projector = rigid_deform_projectors[i];
        if (!projector.isValid()) continue;
        
        const auto& constraint_ref = projector.constraint();
        const auto* constraint = &constraint_ref.get();
        if (constraint && constraint->shouldBreak()) {
            rigid_deform_to_invalidate.push_back(static_cast<int>(i));
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
    
    // Invalidate rigid-deform adhesion projectors that should break
    for (int idx : rigid_deform_to_invalidate) {
        _solver.template setProjectorValidity<RigidDeformAdhesionProjectorType>(idx, false);
        // std::cout << "[adhesion BREAK] Broke rigid-deform adhesion constraint #" << idx << "\n";
    }
    
    // Print summary
    // if (!nerve_tumor_to_invalidate.empty() || !inter_deform_to_invalidate.empty() || !rigid_deform_to_invalidate.empty()) {
    //     std::cout << "[adhesion BREAK] Object: " << this->name()
    //               << " | Broke " << nerve_tumor_to_invalidate.size() << " nerve-tumor"
    //               << " + " << inter_deform_to_invalidate.size() << " inter-deform"
    //               << " + " << rigid_deform_to_invalidate.size() << " rigid-deform adhesions\n";
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
    _vbd_constraints_dirty = true;
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
    _vbd_constraints_dirty = true;
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
    _vbd_constraints_dirty = true;
    return _solver.addConstraintProjector(
        _sim->dt(),
        RefType(vec, vec.size() - 1)
    );
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
Solver::ConstraintProjectorReference<
    Solver::ConstraintProjector<IsFirstOrder, Solver::InterDeformDeformAdhesionConstraint>>
XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>
    ::addInterDeformDeformAdhesionConstraintAsVertex(XPBDMeshObject_Base_<IsFirstOrder>* other_obj, int vertex_v,
                                            int tri_v1, int tri_v2, int tri_v3, 
                                            Real rest_gap, Real break_ratio, Real alpha)
{
    // Implementation for when THIS object provides the Vertex (tumor), and OTHER object provides the Triangle (brain/nerve)
    
    // 1. Get vertex position pointer and mass from THIS object
    Real* vertex_p = _mesh->vertexPointer(vertex_v);
    Real vertex_m = vertexConstraintInertia(vertex_v);
    
    // 2. Get triangle vertex position pointers and masses from the OTHER object
    Real* tri_p1 = other_obj->mesh()->vertexPointer(tri_v1);
    Real* tri_p2 = other_obj->mesh()->vertexPointer(tri_v2);
    Real* tri_p3 = other_obj->mesh()->vertexPointer(tri_v3);
    
    Real tri_m1 = other_obj->vertexConstraintInertia(tri_v1);
    Real tri_m2 = other_obj->vertexConstraintInertia(tri_v2);
    Real tri_m3 = other_obj->vertexConstraintInertia(tri_v3);

    // 3. Add to constraints array
    // Note: We use the SAME constraint type. The solver doesn't care who 'owns' it, 
    // it just needs valid pointers to the data. Use of pointers allows cross-object constraints.
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
    
    // Flag dirty so lookup tables are rebuilt
    _vbd_constraints_dirty = true;

    // 4. Mark vertex as having adhesion constraint for visualization
    if (!_mesh->template hasVertexProperty<bool>("has_adhesion_constraint")) {
        _mesh->template addVertexProperty<bool>("has_adhesion_constraint", false);
    }
    auto& adhesion_prop = _mesh->template getVertexProperty<bool>("has_adhesion_constraint");
    if (vertex_v < _mesh->numVertices()) {
        adhesion_prop.set(vertex_v, true);
    }

    // 5. Tell solver about the new constraint
    using RefType = Solver::ConstraintReference<Solver::InterDeformDeformAdhesionConstraint>;
    return _solver.addConstraintProjector(
        _sim->dt(),
        RefType(vec, vec.size() - 1)
    );
}

// NEW: addRigidDeformAdhesionConstraint
template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
Solver::ConstraintProjectorReference<
    Solver::RigidBodyConstraintProjector<IsFirstOrder, Solver::RigidDeformAdhesionConstraint>>
XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>
    ::addRigidDeformAdhesionConstraint(const Geometry::SDF* sdf, Sim::RigidObject* rigid_obj,
                                      const Vec3r& rigid_body_point,
                                      int tri_v1, int tri_v2, int tri_v3,
                                      Real rest_gap, Real break_ratio, Real alpha)
{
    // 1. Get triangle vertex position pointers and masses from THIS object
    Real* tri_p1 = _mesh->vertexPointer(tri_v1);
    Real* tri_p2 = _mesh->vertexPointer(tri_v2);
    Real* tri_p3 = _mesh->vertexPointer(tri_v3);
    
    Real tri_m1 = vertexConstraintInertia(tri_v1);
    Real tri_m2 = vertexConstraintInertia(tri_v2);
    Real tri_m3 = vertexConstraintInertia(tri_v3);

    // 2. Add to constraints array
    auto& vec = _constraints.template get<Solver::RigidDeformAdhesionConstraint>();
    vec.emplace_back(
        sdf, rigid_obj, rigid_body_point,
        tri_v1, tri_p1, tri_m1,
        tri_v2, tri_p2, tri_m2,
        tri_v3, tri_p3, tri_m3,
        rest_gap,
        break_ratio,
        alpha
    );

    // 3. Tell solver about the new constraint
    using RefType = Solver::ConstraintReference<Solver::RigidDeformAdhesionConstraint>;
    _vbd_constraints_dirty = true;
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
    _vbd_constraints_dirty = true;
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
    
    // Reset inter-deform adhesion constraints
    using InterDeformAdhesionConstraintType = Solver::ConstraintProjector<IsFirstOrder, Solver::InterDeformDeformAdhesionConstraint>;
    auto& inter_deform_adhesion_projectors = _solver.template getConstraintProjectorsOfType<InterDeformAdhesionConstraintType>();
    for (auto& projector : inter_deform_adhesion_projectors) {
        if (projector.isValid()) {
            projector.constraint()->resetMaxDistanceThisStep();
            num_adhesion_constraints++;
        }
    }
    
    // Reset rigid-deform adhesion constraints
    using RigidDeformAdhesionConstraintType = Solver::RigidBodyConstraintProjector<IsFirstOrder, Solver::RigidDeformAdhesionConstraint>;
    auto& rigid_deform_adhesion_projectors = _solver.template getConstraintProjectorsOfType<RigidDeformAdhesionConstraintType>();
    for (auto& projector : rigid_deform_adhesion_projectors) {
        if (projector.isValid()) {
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
    
    // For VBD: save the inertial/predicted positions (after velocity and gravity applied)
    // VBD will minimize energy to find equilibrium near these inertial positions
    if (_sim->config()->solverType() == Config::SolverType::VBD) {
        _inertial_vertices = _mesh->vertices();
    }
    
    auto start_projection = std::chrono::high_resolution_clock::now();
    // Choose solver based on config
    if (_sim->config()->solverType() == Config::SolverType::VBD) {
        _solveVBD();
    } else {
        _projectConstraints();
    }
    auto end_projection = std::chrono::high_resolution_clock::now();

    // Check for broken adhesion constraints
    checkAndBreakAdhesionConstraints(0.0);

    // CRITICAL FIX: Update velocities for the next time step!
    // Without this, _vertex_velocities remains 0 (or stale), meaning no inertia
    // is applied in the next step's _movePositionsInertially(), causing the object
    // to appear frozen or overdamped.
    velocityUpdate();
    
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
    
    // Check if we are using VBD solver which requires 2nd order dynamics (inertia)
    // Even if the object is configured as FirstOrder (Quasi-Static), VBD needs explicit inertia
    // to produce dynamic behavior ("spring back").
    bool use_vbd = (_sim->config()->solverType() == Config::SolverType::VBD);

    if constexpr (IsFirstOrder)
    {
        if (!use_vbd)
        {
            // Quasi-Static / First Order update (Overdamped)
            // Only used if IsFirstOrder is TRUE AND NOT using VBD
            for (int i = 0; i < _mesh->numVertices(); i++)
            {
                const Real dz = -_sim->gAccel() * _vertex_masses[i] * dt / _vertex_B[i];
                _mesh->displaceVertex(i, Vec3r(0,0,dz));
            }
            return;
        }
        else
        {
            // [FIXED] VBD First Order Logic (Quasi-Static)
            // Even with VBD, if the object is FirstOrder, we typically don't want 2nd order inertia.
            // However, strictly zeroing velocity inertia (x_pred = x_curr) causes the VBD inertial term
            // (m/dt^2) to act as a massive drag force that prevents ANY movement (manipulation).
            //
            // To allow manipulation (grasping) to work, we MUST allow the object to predict movement 
            // based on current velocity, even if that velocity is heavily damped.
            // 
            // We fall through to the standard 2nd order update below, which uses _vertex_velocities.
            // Since _vertex_velocities is damped at the end of the previous step (based on vbd-damping),
            // this will produce the correct "Overdamped Dynamic" behavior instead of "Frozen Static".
            
            // Explicitly fall through
        }
    }

    // 2nd Order Dynamics (Inertia + Gravity) - Fallthrough
    // Executed if:
    // 1. IsFirstOrder is false (Standard XPBD)
    // 2. IsFirstOrder is true BUT use_vbd is true (VBD override)
    
    // x_pred = x + v*dt + 0.5*a*dt^2 (Here 1.0*g*dt^2 roughly)
    // Note: _vertex_velocities is updated at the end of the previous step
    _mesh->moveSeparate(dt * _vertex_velocities);
    
    // External forces (gravity)
    if (std::abs(_sim->gAccel()) > 1e-6) {
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
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::_accumulateNeoHookeanForce(
    int vertexId, int tetIdx, int localVertexIdx,
    const Mat3r& DmInv, Real restVolume,
    Real mu, Real lambda,
    Vec3r& force, Mat3r& hessian) const
{
    // Following Gaia's VBD_NeoHookean.cpp implementation
    // Neo-Hookean energy: E = mu * ||F||^2 + lambda * (det(F) - alpha)^2
    // where alpha = 1 + mu/lambda
    
    const Real a = 1.0 + mu / lambda;
    const auto& tet = tetMesh()->element(tetIdx);
    
    // Compute current deformation gradient F = Ds * DmInv
    // Ds is the current edge matrix
    const Vec3r& x0 = _mesh->vertex(tet[0]);
    const Vec3r& x1 = _mesh->vertex(tet[1]);
    const Vec3r& x2 = _mesh->vertex(tet[2]);
    const Vec3r& x3 = _mesh->vertex(tet[3]);
    
    Mat3r Ds;
    Ds.col(0) = x0 - x3;
    Ds.col(1) = x1 - x3;
    Ds.col(2) = x2 - x3;
    
    Mat3r F = Ds * DmInv;
    Real detF = F.determinant();
    
    // Skip only invalid numeric values (NaNs/Infs)
    // CRITICAL FIX: Do NOT skip small/negative determinants! 
    // Doing so disables the restoring force that prevents inversion/collapse.
    if (!std::isfinite(detF)) {
        return;
    }
    
    // Compute gradient of energy w.r.t. F
    // Neo-Hookean: E = mu * ||F||^2 + lambda * (det(F) - a)^2
    // dE/dF = 2*mu*F + 2*lambda*(det(F) - a) * d(det(F))/dF
    Eigen::Matrix<Real, 9, 1> dPhi_D_dF;
    dPhi_D_dF << F(0,0), F(1,0), F(2,0), F(0,1), F(1,1), F(2,1), F(0,2), F(1,2), F(2,2);
    
    // Compute d(detF)/dF (cofactor matrix)
    Eigen::Matrix<Real, 9, 1> ddetF_dF;
    ddetF_dF << F(1,1)*F(2,2) - F(1,2)*F(2,1),  // ddetF/dF11
                F(0,2)*F(2,1) - F(0,1)*F(2,2),  // ddetF/dF21
                F(0,1)*F(1,2) - F(0,2)*F(1,1),  // ddetF/dF31
                F(1,2)*F(2,0) - F(1,0)*F(2,2),  // ddetF/dF12
                F(0,0)*F(2,2) - F(0,2)*F(2,0),  // ddetF/dF22
                F(0,2)*F(1,0) - F(0,0)*F(1,2),  // ddetF/dF32
                F(1,0)*F(2,1) - F(1,1)*F(2,0),  // ddetF/dF13
                F(0,1)*F(2,0) - F(0,0)*F(2,1),  // ddetF/dF23
                F(0,0)*F(1,1) - F(0,1)*F(1,0);  // ddetF/dF33
    
    // Energy gradient in F-space (matching Gaia: no factor of 2)
    // Gaia uses: dE/dF = mu*F + lambda*(detF - alpha)*d(detF)/dF
    Eigen::Matrix<Real, 9, 1> dE_dF = restVolume * (mu*dPhi_D_dF + lambda*(detF - a)*ddetF_dF);
    
    // Check for numerical issues
    if (!dE_dF.allFinite()) {
        // Skip this element if forces are not finite
        return;
    }
    
    // Transform from F-space to vertex coordinates using chain rule
    // dE/dxi = (dF/dxi)^T * dE/dF
    // where dF/dxi depends on which vertex corner we are
    
    Real m1, m2, m3;
    // CORRECTED LOGIC for Pivot x3 (Ds = [x0-x3, x1-x3, x2-x3])
    // The previous implementation was copied from Gaia which likely used Pivot x0, causing incorrect forces.
    switch (localVertexIdx) {
        case 0: // x0 uses Row 0 of DmInv
            m1 = DmInv(0,0);
            m2 = DmInv(0,1);
            m3 = DmInv(0,2);
            break;
        case 1: // x1 uses Row 1 of DmInv
            m1 = DmInv(1,0);
            m2 = DmInv(1,1);
            m3 = DmInv(1,2);
            break;
        case 2: // x2 uses Row 2 of DmInv
            m1 = DmInv(2,0);
            m2 = DmInv(2,1);
            m3 = DmInv(2,2);
            break;
        case 3: // x3 uses Negative Sum of Rows
            m1 = -DmInv(0,0) - DmInv(1,0) - DmInv(2,0);
            m2 = -DmInv(0,1) - DmInv(1,1) - DmInv(2,1);
            m3 = -DmInv(0,2) - DmInv(1,2) - DmInv(2,2);
            break;
        default:
            m1 = m2 = m3 = 0;
            break;
    }
    
    // Compute vertex force: dE/dxi = sum_j(dE/dF_ij * dF_ij/dxi)
    Vec3r dE_dxi;
    dE_dxi(0) = dE_dF(0)*m1 + dE_dF(3)*m2 + dE_dF(6)*m3;
    dE_dxi(1) = dE_dF(1)*m1 + dE_dF(4)*m2 + dE_dF(7)*m3;
    dE_dxi(2) = dE_dF(2)*m1 + dE_dF(5)*m2 + dE_dF(8)*m3;
    
    // Debug: Print force contribution for first few vertices
    static int debug_tet_count = 0;
    if (debug_tet_count < 10 && vertexId < 3) {
        std::cout << "[NEO-HOOKEAN DEBUG] Vertex " << vertexId << " Tet " << tetIdx
                  << " | detF=" << detF << " | restVol=" << restVolume
                  << " | mu=" << mu << " | lambda=" << lambda
                  << " | dE_dF.norm()=" << dE_dF.norm()
                  << " | dE_dxi.norm()=" << dE_dxi.norm() << std::endl;
        debug_tet_count++;
    }
    
    // Force is negative gradient
    force -= dE_dxi;
    
    // ==========================================================================================
    // OPTIMIZATION: Gauss-Newton Approximation for Hessian
    // ==========================================================================================
    // Instead of computing the exact non-linear Hessian (which is indefinite) and performing 
    // an expensive 9x9 SVD to project it to PSD, we use the Gauss-Newton approximation.
    //
    // Exact Hessian: H = mu*I + lambda * ( g*g^T + (J-a)*H_J )
    // Gauss-Newton : H = mu*I + lambda * ( g*g^T )
    //
    // The Gauss-Newton approximation drops the second derivative of the volume constraint.
    // dominated closer to equilibrium. It is inherently PSD (Positive Semi-Definite),
    // so we can skip the SVD entirely. This provides a massive speedup (10x-50x less ops).
    // ==========================================================================================
    
    // 1. Volume preservation part (Rank-1 udpate): lambda * (g * g^T)
    // Note: ddetF_dF is the gradient of volume (g)
    Eigen::Matrix<Real, 9, 9> d2E_dF_dF = (lambda * restVolume) * (ddetF_dF * ddetF_dF.transpose());
    
    // 2. Add mu to diagonal (Isotropic part): mu * I
    Real mu_vol = mu * restVolume;
    for (int i = 0; i < 9; i++) {
        d2E_dF_dF(i, i) += mu_vol;
    }
    
    // NO SVD NEEDED! The matrix is constructed from PSD terms (Sum of Squares + Identity).

    // Step 2: Transform to vertex space using chain rule: H = (dF/dx)^T * H_F * (dF/dx)
    // HL = H_F * (dF/dx) where dF/dx is represented by [m1,m2,m3] coefficients
    Eigen::Matrix<Real, 3, 9> HL;
    HL.row(0) = d2E_dF_dF.row(0) * m1 + d2E_dF_dF.row(3) * m2 + d2E_dF_dF.row(6) * m3;
    HL.row(1) = d2E_dF_dF.row(1) * m1 + d2E_dF_dF.row(4) * m2 + d2E_dF_dF.row(7) * m3;
    HL.row(2) = d2E_dF_dF.row(2) * m1 + d2E_dF_dF.row(5) * m2 + d2E_dF_dF.row(8) * m3;
    
    // H = (dF/dx)^T * HL
    Mat3r d2E_dxi_dxi;
    d2E_dxi_dxi.col(0) = HL.col(0) * m1 + HL.col(3) * m2 + HL.col(6) * m3;
    d2E_dxi_dxi.col(1) = HL.col(1) * m1 + HL.col(4) * m2 + HL.col(7) * m3;
    d2E_dxi_dxi.col(2) = HL.col(2) * m1 + HL.col(5) * m2 + HL.col(8) * m3;
    
    hessian += d2E_dxi_dxi;
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::_solveVBD()
{
    // ============================================================================
    // VBD (Vertex Block Descent) Solver - GAUSS-SEIDEL版本 with Graph Coloring
    // 
    // 基于Gaia的实现：
    // 1. 使用Graph Coloring将顶点分组（同组顶点无共享四面体）
    // 2. 按颜色顺序迭代（Gauss-Seidel）：后面的顶点能看到前面的更新
    // 3. 同一颜色内并行更新
    // 4. 每个顶点立即应用更新（不是批量）
    // ============================================================================
    
    const int num_verts = _mesh->numVertices();
    const int num_iters = _sim->config()->vbdIterations();
    const Real step_size = _sim->config()->vbdStepSize();
    const Real dt = _sim->dt();
    const bool use_full_hessian = _sim->config()->vbdUseFullHessian();
    
    const std::vector<Solver::AttachmentConstraint>& attachment_constraints = 
        _constraints.template get<Solver::AttachmentConstraint>();
    
    // REMOVED: Slow repeated allocation
    // Use member variable _vbd_vertex_to_attachments instead
    bool has_attachments = !attachment_constraints.empty();

    static int debug_frame = 0;
    debug_frame++;
    // if (debug_frame % 60 == 0) {
    //     std::cout << "\n[VBD DEBUG] Frame " << debug_frame 
    //               << " | Attachment constraints: " << attachment_constraints.size()
    //               << " | Step size: " << step_size 
    //               << " | Iterations: " << num_iters << "\n";
    // }
    
    // 预计算Graph Coloring（只在第一次调用时）
    if (!_graph_coloring_computed) {
        std::cout << "\n[VBD Setup] Computing graph coloring for Gauss-Seidel...\n";
        auto coloring = Solver::TetMeshVertexGraph::colorMesh(*tetMesh());
        
        // Apply color balancing to improve distribution
        std::cout << "[VBD Setup] Applying color balancing..." << std::endl;
        coloring->balanceColoredCategories(2.0f);  // Allow 2:1 ratio
        
        _vertex_color_categories = coloring->getCategories();
        
        // Create balanced parallel groups (Gaia-style) - inline implementation
        std::cout << "[VBD] Creating balanced parallel groups..." << std::endl;
        _vertex_parallel_groups.clear();
        _vertex_parallel_groups.resize(_vertex_color_categories.size());
        
        // Helper to find smallest group
        auto findSmallestGroup = [&]() -> size_t {
            size_t smallestIdx = 0;
            size_t smallestSize = _vertex_parallel_groups[0].size();
            for (size_t i = 1; i < _vertex_parallel_groups.size(); i++) {
                if (_vertex_parallel_groups[i].size() < smallestSize) {
                    smallestSize = _vertex_parallel_groups[i].size();
                    smallestIdx = i;
                }
            }
            return smallestIdx;
        };
        
        // Distribute vertices from color categories to balanced parallel groups
        for (size_t colorIdx = 0; colorIdx < _vertex_color_categories.size(); colorIdx++) {
            const auto& colorVertices = _vertex_color_categories[colorIdx];
            for (int vertexId : colorVertices) {
                size_t targetGroup = findSmallestGroup();
                _vertex_parallel_groups[targetGroup].push_back(vertexId);
            }
        }
        
        // Print statistics
        size_t minSize = _vertex_parallel_groups[0].size();
        size_t maxSize = _vertex_parallel_groups[0].size();
        for (size_t i = 0; i < _vertex_parallel_groups.size(); i++) {
            size_t groupSize = _vertex_parallel_groups[i].size();
            minSize = std::min(minSize, groupSize);
            maxSize = std::max(maxSize, groupSize);
        }
        double balanceRatio = maxSize > 0 ? double(maxSize) / double(minSize) : 1.0;
        std::cout << "[VBD] 平衡比例: " << balanceRatio << ":1 (理想值: 1:1)" << std::endl;
        
        _graph_coloring_computed = true;
        
        std::cout << "[VBD Setup] Graph coloring完成:\n";
        std::cout << "  总顶点数: " << num_verts << "\n";
        std::cout << "  颜色数量: " << _vertex_color_categories.size() << "\n";
        
        // 添加图统计信息
        auto graph = Solver::TetMeshVertexGraph::buildFromMesh(*tetMesh());
        std::cout << "  图统计:\n";
        std::cout << "    边数: " << graph.edges.size() << "\n";
        std::cout << "    平均度数: " << (2.0 * graph.edges.size() / num_verts) << "\n";
        
        // 计算最大度数
        int max_degree = 0;
        for (const auto& neighbors : graph.adjacencyList) {
            max_degree = std::max(max_degree, static_cast<int>(neighbors.size()));
        }
        std::cout << "    最大度数: " << max_degree << "\n";
        
        for (size_t i = 0; i < _vertex_color_categories.size(); i++) {
            std::cout << "    颜色 " << i << ": " << _vertex_color_categories[i].size() << " 顶点\n";
        }
        std::cout << "  验证: " << (coloring->isValid() ? "通过" : "失败") << "\n";
        
        // Configure OpenMP threading for VBD parallel execution (one-time setup)
        #ifdef ENABLE_VBD_OPENMP
        const int max_threads = std::min(8, static_cast<int>(_vertex_parallel_groups.size()));
        omp_set_num_threads(max_threads);
        std::cout << "[VBD Setup] Parallel execution enabled with " << max_threads << " threads" << std::endl;
        #else
        std::cout << "[VBD Setup] Serial execution (OpenMP disabled)" << std::endl;
        #endif

        // Pre-compute DmInv and Rest Volumes for all tets
        std::cout << "[VBD Setup] Pre-computing DmInv and Volumes..." << std::endl;
        int num_tets = tetMesh()->numElements();
        _vbd_dm_inverses.resize(num_tets);
        _vbd_rest_volumes.resize(num_tets);
        
        for (int i = 0; i < num_tets; i++) {
            const auto& tet = tetMesh()->element(i);
            const Vec3r& X0_rest = _rest_vertices.col(tet[0]);
            const Vec3r& X1_rest = _rest_vertices.col(tet[1]);
            const Vec3r& X2_rest = _rest_vertices.col(tet[2]);
            const Vec3r& X3_rest = _rest_vertices.col(tet[3]);
            
            Mat3r Dm;
            Dm.col(0) = X0_rest - X3_rest;
            Dm.col(1) = X1_rest - X3_rest;
            Dm.col(2) = X2_rest - X3_rest;
            
            _vbd_dm_inverses[i] = Dm.inverse();
            _vbd_rest_volumes[i] = std::abs(Dm.determinant()) / 6.0;
        }
        std::cout << "[VBD Setup] Pre-computation complete." << std::endl;
    }
    

    // Pre-compute Rigid-Deformable Collision Lookup
    // =============================================
    if (_vbd_constraints_dirty) {
        _cacheCollisionConstraints(); 
        
        // Also ensure _vbd_constraints_dirty is cleared AFTER this frame
        // But we can't clear it here because cacheCollisionConstraints might use it?
        // Actually, let's keep it true for this frame and clear it at the end of solve
    }
    
    // Use pre-computed attachment lookup
    // Optimization: Reuse memory
    // Only rebuild if dirty map or new constraints added
    if (_vbd_vertex_to_attachments.size() != num_verts) {
        _vbd_vertex_to_attachments.resize(num_verts);
    }
    
    // Always rebuild attachment lookup if dirty to capture new grasps
    if (_vbd_constraints_dirty) {
        // Clear old pointers (fast, doesn't deallocate)
        for(auto& vec : _vbd_vertex_to_attachments) vec.clear();

        if (!attachment_constraints.empty()) {
            for (const auto& constraint : attachment_constraints) {
                if (constraint.vertexIndex() < num_verts) {
                    _vbd_vertex_to_attachments[constraint.vertexIndex()].push_back(&constraint);
                }
            }
        }
    }
    
    // 保存inertia位置（移动后的位置）
    const MatXr inertia_positions = _mesh->vertices();

    // ============================================================================
    // CHEBYSHEV ACCELERATION SETUP
    // ============================================================================
    if (_vbd_prev_iter_vertices.cols() != num_verts) {
        _vbd_prev_iter_vertices.resize(3, num_verts);
        _vbd_iter_start_vertices.resize(3, num_verts);
    }
    
    // Initialize history with current predicted positions
    _vbd_prev_iter_vertices = _mesh->vertices();
    
    Real omega = 1.0;
    const Real rho = 0.995; // Spectral radius estimate (tuned for stability)
    
    // ============================================================================
    // VBD外层循环：使用平衡并行组 (Gaia-style)
    // ============================================================================
    for (int iter = 0; iter < num_iters; iter++) {
        
        // GAIA-STYLE: Intermediate Collision Detection
        // Update collision constraints every few iterations to effectively handle large deformations/penetrations
        const int intermediate_collision_rate = 5; // Every 5 iterations
        if (iter > 0 && iter % intermediate_collision_rate == 0) {
           if (_sim) const_cast<Simulation*>(_sim)->refreshCollisionScene();
           _cacheCollisionConstraints(); // Re-populate local cache from new constraints
        }

        // 1. Update Chebyshev Omega
        omega = _getChebyshevOmega(iter + 1, rho, omega);
        
        // 2. Save current state x_k (will become x_{k-1} for next iter)
        _vbd_iter_start_vertices = _mesh->vertices();

        // 按平衡并行组处理，而不是严格颜色顺序
        for (size_t group = 0; group < _vertex_parallel_groups.size(); group++) {
            const auto& vertices_in_group = _vertex_parallel_groups[group];
            
            // 同一组的顶点并行更新（OpenMP）
            #ifdef ENABLE_VBD_OPENMP
            #pragma omp parallel for schedule(static)
            #endif
            for (size_t i = 0; i < vertices_in_group.size(); i++) {
                const int vid = vertices_in_group[i];
                
                if (vertexFixed(vid)) continue;
                
                // ========================================
                // GAIA-INSPIRED VBD STEP (Much more efficient!)
                // ========================================
                
                // Assemble force and Hessian directly like Gaia's VBDStep
                const Real dt = _sim->dt();
                const Real mass = vertexMass(vid);
                const Vec3r x_current = _mesh->vertex(vid);
                const Vec3r x_inertia = _inertial_vertices.col(vid);  // Use saved inertial/predicted position
                
                Vec3r force = Vec3r::Zero();
                Mat3r hessian = Mat3r::Zero();
                
                // 1. Inertia force and Hessian (like Gaia's accumlateInertiaForceAndHessian)
                // Force is NEGATIVE gradient: -∇E = -m/(dt²)*(x-x_inertia) = m/(dt²)*(x_inertia-x)
                // x_inertia is the predicted position (with velocity and gravity already applied)
                // VBD brings vertices from current position back toward this inertial equilibrium
                force = mass / (dt * dt) * (x_inertia - x_current);
                hessian = (mass / (dt * dt)) * Mat3r::Identity();
                
                // 2. Elastic forces from Neo-Hookean energy (like Gaia's accumlateMaterialForceAndHessian)
                // For each tetrahedron attached to this vertex, compute elastic restoring force
                const std::vector<int>& attached_tets = tetMesh()->vertexAttachedElements(vid);
                
                // Get material parameters (use first material for simplicity)
                const ElasticMaterial& material = _materials[0];
                const Real mu = material.mu();      // Lamé first parameter (shear modulus)
                const Real lambda = material.lambda(); // Lamé second parameter (bulk modulus)
                
                // Use full material stiffness - PSD filtering in Hessian ensures stability
                
                for (int tet_idx : attached_tets) {
                    const auto& tet = tetMesh()->element(tet_idx);
                    
                    // Find which corner of the tet this vertex is (0-3)
                    int local_vid = -1;
                    if (tet[0] == vid) local_vid = 0;
                    else if (tet[1] == vid) local_vid = 1;
                    else if (tet[2] == vid) local_vid = 2;
                    else if (tet[3] == vid) local_vid = 3;
                    
                    if (local_vid == -1) continue;
                    
                    // Optimization: Use precomputed DmInv and RestVolume
                    const Mat3r& DmInv = _vbd_dm_inverses[tet_idx];
                    const Real restVol = _vbd_rest_volumes[tet_idx];
                    
                    _accumulateNeoHookeanForce(
                        vid, tet_idx, local_vid,
                        DmInv, restVol,
                        mu, lambda,
                        force, hessian
                    );
                }
                
                // DEBUG: Check elastic force magnitude
                if (debug_frame % 60 == 0 && vid < 5 && attached_tets.size() > 0) {
                    Vec3r inertia_force_vec = mass / (dt * dt) * (x_inertia - x_current);
                    Vec3r elastic_force_vec = force - inertia_force_vec;
                    Real inertia_force_mag = inertia_force_vec.norm();
                    Real elastic_force_mag = elastic_force_vec.norm();
                    // std::cout << "[VBD DEBUG] Vertex " << vid 
                    //           << " | #Tets=" << attached_tets.size()
                    //           << " | Inertia=" << inertia_force_mag
                    //           << " | Elastic=" << elastic_force_mag 
                    //           << " | Ratio=" << (elastic_force_mag / (inertia_force_mag + 1e-10))
                    //           << " | Total=" << force.norm() << std::endl;
                }
                
                // 3. Attachment constraint forces (for grasping)
                // Optimized: Use pre-computed lookup table instead of iterating all constraints
                int num_attachments_for_vertex = 0;
                
                if (has_attachments && !_vbd_vertex_to_attachments[vid].empty()) {
                    for (const auto* constraint_ptr : _vbd_vertex_to_attachments[vid]) {
                        num_attachments_for_vertex++;
                        const auto& constraint = *constraint_ptr;
                        
                        // Get target position: attach_pos + offset
                        const Vec3r* attach_pos_ptr = constraint.attachmentPosition();
                        const Vec3r& offset = constraint.attachmentOffset();
                        Vec3r target_pos = *attach_pos_ptr + offset;
                        
                        // Spring force: k * (target - current)
                        // BALANCED: 5e4 provides strong grasp without instability (with line search enabled)
                        // Combined with line search, this prevents overshooting while maintaining responsiveness
                        const Real k_attachment = 5e4;  // 50,000 N/m
                        Vec3r attachment_force = k_attachment * (target_pos - x_current);
                        
                        // DEBUG: Print first constraint for this vertex
                        // if (num_attachments_for_vertex == 1 && debug_frame % 60 == 0 && vid < 10) {
                        //     std::cout << "[VBD DEBUG] Vertex " << vid 
                        //               << " | Target: (" << target_pos.transpose() << ")"
                        //               << " | Current: (" << x_current.transpose() << ")"
                        //               << " | Attach force: " << attachment_force.norm() << "\n";
                        // }
                        
                        force += attachment_force;
                        hessian += k_attachment * Mat3r::Identity();
                    }
                }
                
                // 4. Rigid-Deformable Collision (Penalty Method)
                // ==============================================
                if (!_vbd_rigid_collisions[vid].empty()) {
                    const Real collision_k = 100000.0; 
                    
                    for (const auto* c : _vbd_rigid_collisions[vid]) {
                         const auto& positions = c->positions();
                         int v1_idx = positions[0].index;
                         int v2_idx = positions[1].index;
                         int v3_idx = positions[2].index;
                         
                         Real weight = 0.0;
                         if (vid == v1_idx) weight = c->u();
                         else if (vid == v2_idx) weight = c->v();
                         else if (vid == v3_idx) weight = c->w();
                         else continue;

                         Vec3r p1 = _mesh->vertex(v1_idx);
                         Vec3r p2 = _mesh->vertex(v2_idx);
                         Vec3r p3 = _mesh->vertex(v3_idx);
                         Vec3r p_cur = c->u() * p1 + c->v() * p2 + c->w() * p3;
                         
                         Real dist = c->sdf()->evaluate(p_cur);
                         
                         if (dist < 0) { // Penetrating
                             Vec3r n = c->sdf()->gradient(p_cur);
                             Vec3r F = -collision_k * dist * n;
                             
                             force += weight * F;
                             Mat3r H = collision_k * (n * n.transpose());
                             hessian += (weight * weight) * H;
                         }
                    }
                }

                // 5. Static-Deformable Collision (Penalty Method)
                // ===============================================
                if (!_vbd_static_collisions[vid].empty()) {
                    const Real collision_k = 100000.0;
                    
                    for (const auto* c : _vbd_static_collisions[vid]) {
                         const auto& positions = c->positions();
                         int v1_idx = positions[0].index;
                         int v2_idx = positions[1].index;
                         int v3_idx = positions[2].index;
                         
                         Real weight = 0.0;
                         if (vid == v1_idx) weight = c->u();
                         else if (vid == v2_idx) weight = c->v();
                         else if (vid == v3_idx) weight = c->w();
                         else continue;

                         Vec3r p1 = _mesh->vertex(v1_idx);
                         Vec3r p2 = _mesh->vertex(v2_idx);
                         Vec3r p3 = _mesh->vertex(v3_idx);
                         Vec3r p_cur = c->u() * p1 + c->v() * p2 + c->w() * p3;
                         
                         Real dist = c->sdf()->evaluate(p_cur);
                         
                         if (dist < 0) {
                             Vec3r n = c->sdf()->gradient(p_cur);
                             Vec3r F = -collision_k * dist * n;
                             
                             force += weight * F;
                             Mat3r H = collision_k * (n * n.transpose());
                             hessian += (weight * weight) * H;
                         }
                    }
                }

                // 6. Inter-Object Deformable Collision (Gaia Penalty Method - Improved)
                // =====================================================================
                if (!_vbd_inter_deform_collisions[vid].empty()) {
                    
                    // Stiffness: High to keep objects apart
                    const Real collision_k = 2e5; 
                    
                    // Buffer zone: Reduced to 1mm to avoid fighting with Adhesion (rest_gap = 2mm)
                    // Previously 3mm -> caused oscillation in [2mm, 3mm] range
                    const Real thickness = 0.001; 

                    // Max correction per step: Safety clamp to prevent explosions
                    const Real max_force_mag = 5000.0; 

                    for (const auto* c : _vbd_inter_deform_collisions[vid]) {
                        
                        const auto& positions = c->positions();
                        
                        Eigen::Map<const Vec3r> q(positions[0].position_ptr);
                        Eigen::Map<const Vec3r> p1(positions[1].position_ptr);
                        Eigen::Map<const Vec3r> p2(positions[2].position_ptr);
                        Eigen::Map<const Vec3r> p3(positions[3].position_ptr);

                        Vec3r cross_prod = (p2 - p1).cross(p3 - p1);
                        Real area_sq = cross_prod.squaredNorm();
                        if (area_sq < 1e-12) continue; 
                        
                        Vec3r n = cross_prod / std::sqrt(area_sq);
                        
                        // Distance to plane
                        Real dist = (q - p1).dot(n);
                        
                        // Check against buffer zone
                        if (dist < thickness) {
                             
                             // Depth inside the buffer (positive value)
                             Real penetration = thickness - dist;
                             
                             // ------------------- CRITICAL FIX -------------------
                             // 1. One-sided check: Only push if penetrating from the "outside" (positive normal side)
                             // If dist is largely negative, it means we are deeply inside or on the backface.
                             // Penalty methods often explode if they push "wrongly" when point is behind failure.
                             // Assuming consistent winding, 'n' points outwards.
                             // Limit penetration depth to avoid crazy forces if topology tangles
                             if (penetration > thickness * 2.0) penetration = thickness * 2.0;

                             // 2. Force calculation (Linear Spring)
                             // Increase stiffness significantly if penetration is deep? No, that causes explosions.
                             // Keep stiffness constant.
                             Real lambda = collision_k * penetration;
                             
                             // 3. Safety Clamp: Prevent numerical explosions
                             if (lambda > max_force_mag) lambda = max_force_mag;

                             const Real* vid_ptr = _mesh->vertices().data() + vid * 3;
                             
                             Real b_i = 0.0;
                             Real sign = 0.0;
                             
                             if (positions[0].position_ptr == vid_ptr) {
                                 // q (vertex)
                                 b_i = 1.0;
                                 sign = 1.0; // Force pushes q along n
                             } else {
                                 // p1, p2, p3 (triangle)
                                 // Barycentric check (simplified projection)
                                 Vec3r proj_q = q - dist * n;
                                 Vec3r v0 = p2 - p1;
                                 Vec3r v1 = p3 - p1;
                                 Vec3r v2 = proj_q - p1;
                                 
                                 Real d00 = v0.dot(v0);
                                 Real d01 = v0.dot(v1);
                                 Real d11 = v1.dot(v1);
                                 Real d20 = v2.dot(v0);
                                 Real d21 = v2.dot(v1);
                                 Real denom = d00 * d11 - d01 * d01;
                                 
                                 if (std::abs(denom) < 1e-12) continue;
                                 
                                 Real v = (d11 * d20 - d01 * d21) / denom;
                                 Real w = (d00 * d21 - d01 * d20) / denom;
                                 Real u = 1.0 - v - w;
                                 
                                 // Relaxed barycentric check for edges
                                 if (u < -0.2 || v < -0.2 || w < -0.2) continue; 
                                 
                                 if (positions[1].position_ptr == vid_ptr) b_i = u;
                                 else if (positions[2].position_ptr == vid_ptr) b_i = v;
                                 else b_i = w;
                                 
                                 sign = -1.0; // Force pushes triangle opposite to n
                             }
                             
                             // Force direction: n points OUT of triangle.
                             // If q is at dist < thickness (e.g. 0.0), it should be pushed by +n.
                             // If triangle is pushed, it should be by -n.
                             // sign handles this (+1 for q, -1 for triangle).
                             Vec3r F_collision = sign * b_i * lambda * n;
                             
                             // 4. Stabilizing Hessian
                             // Pure penalty Hessian is K * n * n^T. This is rank-1 and allows sliding.
                             // Deep penetration often causes "fighting" between normal force and friction/adhesion.
                             // We add a significant isotropic term (damping/regularization) to prevent shooting nodes to infinity.
                             
                             // Main normal stiffness
                             Mat3r H_normal = collision_k * b_i * b_i * (n * n.transpose());
                             
                             // Regularization: 10% of stiffness implicitly prevents large steps in ANY direction when colliding
                             // This is the "magic sauce" for stable penalty methods in VBD
                             Mat3r H_stabilize = (collision_k * 0.1) * Mat3r::Identity(); 
                             
                             force += F_collision;
                             hessian += (H_normal + H_stabilize);
                        }
                    }
                }

                // 7. Rigid-Deform Adhesion (Sticky Tumor - Penalty Method)
                // ========================================================
                if (!_vbd_rigid_adhesions[vid].empty()) {
                    
                    for (const auto* c : _vbd_rigid_adhesions[vid]) {
                         if (c->shouldBreak()) continue; 

                         // Use compliance from constraint if available
                         Real adhesion_k = 10000.0;
                         if (c->alpha() > 1e-12) {
                             adhesion_k = 1.0 / c->alpha();
                         }

                         const auto& positions = c->positions();
                         int v1_idx = positions[0].index;
                         int v2_idx = positions[1].index;
                         int v3_idx = positions[2].index;
                         
                         const Vec3r& bary = c->barycentricCoords();
                         
                         Real weight = 0.0;
                         if (vid == v1_idx) weight = bary[0];
                         else if (vid == v2_idx) weight = bary[1];
                         else if (vid == v3_idx) weight = bary[2];
                         else continue;

                         Vec3r p1 = _mesh->vertex(v1_idx);
                         Vec3r p2 = _mesh->vertex(v2_idx);
                         Vec3r p3 = _mesh->vertex(v3_idx);
                         Vec3r p_tri = bary[0] * p1 + bary[1] * p2 + bary[2] * p3;
                         
                         const Sim::RigidObject* rigid_obj = c->rigidObject();
                         Vec3r p_rigid = rigid_obj->bodyToGlobal(c->rigidBodyPoint());
                         
                         Vec3r diff = p_rigid - p_tri;
                         Real dist = diff.norm();
                         c->registerDistance(dist); // CRITICAL for breaking logic
                         
                         if (dist > 1e-12) {
                             Vec3r n = diff / dist; 
                             
                             Real deformation = dist - c->getRestGap();
                             
                             if (deformation > 0) {
                                  Vec3r F = adhesion_k * deformation * n;
                                  
                                  force += weight * F;
                                  
                                  Mat3r H = adhesion_k * (n * n.transpose());
                                  hessian += (weight * weight) * H;
                             }
                         }
                    }
                }

                // 8. Inter-Object Deformable Adhesion (VBD Implementation)
                // ========================================================
                if (!_vbd_inter_deform_adhesions[vid].empty()) {
                     
                     for (const auto* c : _vbd_inter_deform_adhesions[vid]) {
                         // Skip broken bonds
                         if (c->shouldBreak()) continue;
                         
                         // Calculate stiffness from constraint compliance (alpha)
                         // k = 1 / alpha
                         Real adhesion_k = 20000.0; // Default fallback
                         if (c->alpha() > 1e-12) {
                             adhesion_k = 1.0 / c->alpha();
                         }

                         // Evaluate constraint using current VBD positions
                         // This automatically updates the internal breaking tracker
                         Real C_val = 0.0;
                         Real grads[12]; // 4 positions * 3 coords
                         
                         // Cast away constness to call evaluateWithGradient for VBD update
                         auto* mutable_c = const_cast<Solver::InterDeformDeformAdhesionConstraint*>(c);
                         mutable_c->evaluateWithGradient(&C_val, grads);
                         
                         // C_val > 0 means separation > rest_gap (adhesive tension)
                         // VBD Penalty Logic: E = 0.5 * k * C(x)^2 for C > 0
                         // force = -dE/dx = -k * C * dC/dx
                         // hessian = k * (dC/dx * dC/dx^T + C * d2C/dx2)
                         
                         if (C_val > 0) {
                             
                             // 1. Check if strain exceeds break ratio
                             // Using max_distance tracking inside constraint
                             // If it breaks, we shouldn't apply force this step (or maybe apply one last time?)
                             // Current impl applies force then breaks later.
                             
                             const auto& positions = c->positions();
                             
                             // Find my index (0..3) in the constraint
                             int my_idx = -1;
                             const Real* vid_ptr = _mesh->vertices().data() + vid * 3;
                             
                             for(int k=0; k<4; k++) {
                                 if (positions[k].position_ptr == vid_ptr) {
                                     my_idx = k;
                                     break;
                                 }
                             }
                             
                             if (my_idx != -1) {
                                  Vec3r grad_i(grads[my_idx*3+0], grads[my_idx*3+1], grads[my_idx*3+2]);
                                  
                                  // Force = -k * C * grad
                                  Vec3r F = -adhesion_k * C_val * grad_i;
                                  
                                  // Hessian approximation: k * grad * grad^T
                                  // Gauss-Newton approximation (ignoring 2nd derivative of C)
                                  Mat3r H = adhesion_k * (grad_i * grad_i.transpose());
                                  
                                  // STABILITY FIX for Adhesion:
                                  // Adhesion involves small triangles pulling a vertex.
                                  // If the gradient is perpendicular to motion, H becomes singular (rank-1).
                                  // We need 'isotropic' holding power to prevent the vertex from "sliding off" the adhesive bond laterally.
                                  // Increasing stabilization from 1% to 10% prevents "exploding" when stretched.
                                  Mat3r H_damp = (adhesion_k * 0.1) * Mat3r::Identity();

                                  force += F;
                                  hessian += (H + H_damp);
                             }
                         }
                     }
                }

                // 9. Self-Collision (Deformable-Deformable)
                // =========================================
                if (!_vbd_self_collisions[vid].empty()) {
                    const Real collision_k = 2e5; 
                    const Real thickness = 0.001; // Reduced to 1mm
                    const Real max_force_mag = 5000.0; 

                    for (const auto* c : _vbd_self_collisions[vid]) {
                        
                        const auto& positions = c->positions();
                        
                        Eigen::Map<const Vec3r> q(positions[0].position_ptr);
                        Eigen::Map<const Vec3r> p1(positions[1].position_ptr);
                        Eigen::Map<const Vec3r> p2(positions[2].position_ptr);
                        Eigen::Map<const Vec3r> p3(positions[3].position_ptr);
                        
                        // Ignore collisions where vertex is part of the triangle (topology check)
                        // In self-collision, adjacent faces/vertices are often excluded by broadphase
                        
                        Vec3r cross_prod = (p2 - p1).cross(p3 - p1);
                        Real area_sq = cross_prod.squaredNorm();
                        if (area_sq < 1e-12) continue; 
                        
                        Vec3r n = cross_prod / std::sqrt(area_sq);
                        
                        Real dist = (q - p1).dot(n);
                        
                        if (dist < thickness) {
                             Real penetration = thickness - dist;
                             Real lambda = collision_k * penetration;
                             if (lambda > max_force_mag) lambda = max_force_mag;

                             const Real* vid_ptr = _mesh->vertices().data() + vid * 3;
                             
                             Real b_i = 0.0;
                             Real sign = 0.0;
                             
                             if (positions[0].position_ptr == vid_ptr) {
                                 b_i = 1.0;
                                 sign = 1.0;
                             } else {
                                 // I am on the Triangle
                                 Vec3r proj_q = q - dist * n;
                                 Vec3r v0 = p2 - p1;
                                 Vec3r v1 = p3 - p1;
                                 Vec3r v2 = proj_q - p1;
                                 
                                 Real d00 = v0.dot(v0);
                                 Real d01 = v0.dot(v1);
                                 Real d11 = v1.dot(v1);
                                 Real d20 = v2.dot(v0);
                                 Real d21 = v2.dot(v1);
                                 
                                 Real denom = d00 * d11 - d01 * d01;
                                 if (std::abs(denom) < 1e-12) continue;
                                 
                                 Real v = (d11 * d20 - d01 * d21) / denom;
                                 Real w = (d00 * d21 - d01 * d20) / denom;
                                 Real u = 1.0 - v - w;
                                 
                                 if (u < -0.1 || v < -0.1 || w < -0.1) continue;
                                 
                                 if (positions[1].position_ptr == vid_ptr) b_i = u;
                                 else if (positions[2].position_ptr == vid_ptr) b_i = v;
                                 else b_i = w;
                                 
                                 sign = -1.0; 
                             }
                             
                             Vec3r F_collision = sign * b_i * lambda * n;
                             Mat3r H_normal = collision_k * b_i * b_i * (n * n.transpose());
                             Mat3r H_stabilize = (collision_k * 0.01) * Mat3r::Identity();
                             
                             force += F_collision;
                             hessian += (H_normal + H_stabilize);
                        }
                    }
                }

                // 5. Solve for descent direction (like Gaia's CuMatrix::solve3x3_psd_stable)
                if (force.squaredNorm() > 1e-12) {
                    Vec3r descentDirection;
                    bool solverSuccess = Utils::solve3x3PSD(hessian.data(), force.data(), descentDirection.data());
                    
                    // DEBUG: Print movement for first few vertices with attachments
                    // if (num_attachments_for_vertex > 0 && debug_frame % 60 == 0 && vid < 5) {
                    //     std::cout << "[VBD DEBUG] Vertex " << vid 
                    //               << " | Total force: " << force.norm()
                    //               << " | Descent dir: " << descentDirection.norm()
                    //               << " | Step: " << (step_size * descentDirection).norm() << "\n";
                    // }
                    
                    if (solverSuccess) {
                        // STABILITY IMPROVEMENT: Enable line search for adaptive step sizing
                        // This prevents overshooting in steep energy landscapes (especially during grasping)
                        // #ifdef ENABLE_VBD_LINE_SEARCH
                        // // Gaia-style backtracking line search
                        // Real initialEnergy = _evaluateVertexEnergy(vid);
                        // Real optimalStepSize = _vbdLineSearch(vid, descentDirection, initialEnergy, step_size);
                        // 
                        // // Apply optimal step found by line search
                        // _mesh->setVertex(vid, _mesh->vertex(vid));  // Already set by line search
                        // #else
                        // Simple fixed step size (faster but less stable)
                        _mesh->displaceVertex(vid, step_size * descentDirection);
                        // #endif
                    } else {
                        // Fallback: gradient descent when solver fails
                        _mesh->displaceVertex(vid, step_size * 0.1 * force.normalized());
                    }
                }
            }
        }

        // ============================================================================
        // CHEBYSHEV ACCELERATION STEP
        // ============================================================================
        // x_{k+1} = omega * (x_{k+1}^* - x_{k-1}) + x_{k-1}
        // Where x_{k+1}^* is current _mesh->vertices()
        // and x_{k-1} is _vbd_prev_iter_vertices
        
        if (omega != 1.0) {
            #ifdef ENABLE_VBD_OPENMP
            #pragma omp parallel for schedule(static)
            #endif
            for (int vid = 0; vid < num_verts; vid++) {
                 if (vertexFixed(vid)) continue;
                 
                 Vec3r current_pos = _mesh->vertex(vid);
                 const Vec3r& prev_iter_pos = _vbd_prev_iter_vertices.col(vid);
                 
                 // Apply Chebyshev formula:
                 // pos = prev + omega * (current - prev)
                 Vec3r new_pos = prev_iter_pos + omega * (current_pos - prev_iter_pos);
                 
                 _mesh->vertex(vid) = new_pos;
            }
        }
        
        // Update history: x_{k-1} for next iter becomes x_k (which we saved at start)
        _vbd_prev_iter_vertices = _vbd_iter_start_vertices;

    }
    
    // NOTE: Do NOT reset fixed vertices to _previous_vertices!
    // Fixed vertices (from grasping) should stay at their CURRENT position,
    // not be pulled back to the position from BEFORE inertial motion.
    // The "continue" at line 1402 already handles skipping fixed vertices during VBD.

    // Clear dirty flag after all processing
    _vbd_constraints_dirty = false;
}

// ============================================================================
// VBD Line Search Implementation (Gaia-style)
// ============================================================================

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
Real XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::_evaluateVertexEnergy(int vertexId) const {
    const Real dt = _sim->dt();
    Real total_energy = 0.0;
    
    // 1. Inertia energy (matches Gaia's computeInertiaEnergy implementation)
    const Real mass = vertexMass(vertexId);
    const Vec3r x_current = _mesh->vertex(vertexId);
    const Vec3r x_inertia = _previous_vertices.col(vertexId);  // Inertia position
    
    Real inertia_energy = 0.5 * mass / (dt * dt) * (x_current - x_inertia).squaredNorm();
    total_energy += inertia_energy;
    
    // 2. Elastic energy (approximate contribution from attached tetrahedra)
    // For each tet containing this vertex, compute its elastic energy contribution
    for (int tet_idx = 0; tet_idx < tetMesh()->numElements(); tet_idx++) {
        const auto& tet = tetMesh()->element(tet_idx);
        bool vertex_in_tet = false;
        for (int i = 0; i < 4; i++) {
            if (tet[i] == vertexId) {
                vertex_in_tet = true;
                break;
            }
        }
        
        if (vertex_in_tet) {
            // Simple approximation: 1/4 of the tet's elastic energy
            // This is a reasonable approximation for line search purposes
            Vec3r x0 = _mesh->vertex(tet[0]);
            Vec3r x1 = _mesh->vertex(tet[1]);
            Vec3r x2 = _mesh->vertex(tet[2]);
            Vec3r x3 = _mesh->vertex(tet[3]);
            
            // Compute deformation gradient and simple elastic energy
            // Using a simplified Neo-Hookean energy approximation
            Mat3r F;
            F.col(0) = x1 - x0;
            F.col(1) = x2 - x0; 
            F.col(2) = x3 - x0;
            
            Real det_F = F.determinant();
            if (det_F > 1e-10) { // Avoid singularities
                Real I1 = F.squaredNorm();
                Real I3 = det_F * det_F;
                
                // Simplified Neo-Hookean energy (without proper material parameters)
                Real elastic_contribution = 0.25 * (I1 + 1.0 / I3 - 3.0) * _tetVolumes[tet_idx];
                total_energy += elastic_contribution * 0.25;  // 1/4 contribution per vertex
            }
        }
    }
    
    return total_energy;
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
Real XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::_vbdLineSearch(
    int vertexId, const Vec3r& descentDirection, Real initialEnergy, Real maxStepSize) {
    
    // Fast Line Search parameters for performance
    const Real c = 0.1;           // Wolfe condition parameter
    const Real tau = 0.7;         // Less aggressive reduction (was 0.5)
    const int maxIters = 3;       // Minimal iterations for speed (was 8)
    const Real minStepSize = 1e-4; // Larger minimum step (was 1e-6)
    
    const Vec3r originalPos = _mesh->vertex(vertexId);
    const Real m = descentDirection.squaredNorm();
    
    Real alpha = maxStepSize;
    Real bestAlpha = 0.0;
    Real bestEnergy = initialEnergy;
    
    for (int iter = 0; iter < maxIters; iter++) {
        // Test this step size
        _mesh->setVertex(vertexId, originalPos + alpha * descentDirection);
        
        Real currentEnergy = _evaluateVertexEnergy(vertexId);
        
        // Track best energy found
        if (currentEnergy < bestEnergy) {
            bestAlpha = alpha;
            bestEnergy = currentEnergy;
        }
        
        // Check Wolfe condition (sufficient decrease)
        if (currentEnergy < initialEnergy - alpha * c * m) {
            // Found good step size
            break;
        }
        
        // Reduce step size
        alpha *= tau;
        
        // Stop if step size too small
        if (alpha < minStepSize) {
            break;
        }
    }
    
    // Restore original position
    _mesh->setVertex(vertexId, originalPos);
    
    // Return best step size found (fallback to small step if no improvement)
    return (bestAlpha > 0) ? bestAlpha : minStepSize;
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::velocityUpdate()
{
    // Check if using VBD or standard XPBD
    bool use_vbd = (_sim->config()->solverType() == Config::SolverType::VBD);
    const Real dt = _sim->dt();

    if constexpr (IsFirstOrder)
    {
        if (!use_vbd)
        {
            // For Quasi-Static simulation (FirstOrder without VBD), velocity is not tracked for dynamics.
            // But we might want to update it for visualization or other logic.
            // However, the "inertial update" uses _vertex_B (overdamping).
            
            // Standard FirstOrder update usually zeros velocity or sets it based on displacement.
            // Setting it to (x_new - x_old)/dt is fine, but it shouldn't be used for next step's inertia
            // unless we are in 2nd order mode.
            _vertex_velocities = (_mesh->vertices() - _previous_vertices) / dt;
            return;
        }
    }

    const Geometry::Mesh::VerticesMat& cur_vertices = _mesh->vertices();
    // velocities are simply (cur_pos - last_pos) / deltaT
    _vertex_velocities = (cur_vertices - _previous_vertices) / dt;
    
    // Explicit Damping Control
    // ========================
    // If not using VBD (i.e. Standard XPBD), apply simpler damping.
    // If using VBD, we might want cleaner damping control.
    
    Real damping_factor = 1.0;

    if (use_vbd) {
        // VBD damping controlled by config
        // Default vbd-damping is 0.0 (no damping)
        // Set vbd-damping: 0.01 in YAML for 1% damping per step
        // CRITICAL FIX: Clamp input BEFORE calculation to prevent negative damping_factor
        Real vbd_damping = _sim->config()->vbdDamping();
        vbd_damping = std::clamp(vbd_damping, 0.0, 0.99);  // Limit to reasonable range [0%, 99%]
        damping_factor = 1.0 - vbd_damping;
        
        // Safety clamp (should be unnecessary now, but kept for robustness)
        if (damping_factor < 0.0) damping_factor = 0.0;
        if (damping_factor > 1.0) damping_factor = 1.0;
    } else {
        // Standard XPBD damping
        damping_factor = 0.998; 
    }

    // Apply global damping
    _vertex_velocities *= damping_factor;
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
    
    // Struct to hold collision data temporarily
    struct SelfCollisionInfo {
        int vertex_idx;
        int face_idx; 
        // We will fetch pointers/masses serially to be safe or parallel if careful, 
        // but fetching indices is enough.
    };

    #ifdef ENABLE_VBD_OPENMP
    
    int max_threads = omp_get_max_threads();
    std::vector<std::vector<SelfCollisionInfo>> thread_results(max_threads);
    
    // 1. Parallel Query (The Heavy Part)
    #pragma omp parallel for schedule(dynamic, 64)
    for (int i = 0; i < _mesh->numVertices(); i++)
    {
        if (!_mesh->vertexOnSurface(i))
            continue;

        std::set<Geometry::EmbreeHit> hits = embree_scene->tetMeshSelfCollisionQuery(i, this);
        if (hits.size() > 0)
        {
            int face_index = _sdf->closestSurfaceFaceToPointInTet(_mesh->vertex(i), hits.begin()->prim_index);

            if (face_index >= 0) {
                int tid = omp_get_thread_num();
                thread_results[tid].push_back({i, face_index});
            }
        }
    }
    
    // 2. Serial Commit (Fast enough, avoids synchronization issues)
    std::vector<Solver::DeformableDeformableCollisionConstraint>& constraint_vec = _constraints.template get<Solver::DeformableDeformableCollisionConstraint>();
    bool any_added = false;

    for (const auto& t_results : thread_results) {
        for (const auto& info : t_results) {
            int i = info.vertex_idx;
            int face_idx = info.face_idx;
            
            const Eigen::Vector3i& face = _mesh->face(face_idx);

            Real* q_ptr = _mesh->vertexPointer(i);
            Real* p1_ptr = _mesh->vertexPointer(face[0]);
            Real* p2_ptr = _mesh->vertexPointer(face[1]);
            Real* p3_ptr = _mesh->vertexPointer(face[2]);

            Real qm = vertexConstraintInertia(i);
            Real p1m = vertexConstraintInertia(face[0]);
            Real p2m = vertexConstraintInertia(face[1]);
            Real p3m = vertexConstraintInertia(face[2]);

            constraint_vec.emplace_back(i, q_ptr, qm, face[0], p1_ptr, p1m, face[1], p2_ptr, p2m, face[2], p3_ptr, p3m);

            using ConstraintRefType = Solver::ConstraintReference<Solver::DeformableDeformableCollisionConstraint>;
            _solver.addConstraintProjector(_sim->dt(), ConstraintRefType(constraint_vec, constraint_vec.size()-1));
            
            any_added = true;
        }
    }
    
    if (any_added) {
        _vbd_constraints_dirty = true;
    }

    #else
    // Serial Fallback for non-OpenMP builds
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

            std::vector<Solver::DeformableDeformableCollisionConstraint>& constraint_vec = _constraints.template get<Solver::DeformableDeformableCollisionConstraint>();
            constraint_vec.emplace_back(i, q_ptr, qm, face[0], p1_ptr, p1m, face[1], p2_ptr, p2m, face[2], p3_ptr, p3m);

            using ConstraintRefType = Solver::ConstraintReference<Solver::DeformableDeformableCollisionConstraint>;
            _solver.addConstraintProjector(_sim->dt(), ConstraintRefType(constraint_vec, constraint_vec.size()-1));
            
            _vbd_constraints_dirty = true;
        }
    }
    #endif
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


template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::_cacheCollisionConstraints()
{
    const int num_verts = _mesh->numVertices();
    const Real* my_vertices_start = _mesh->vertices().data();
    const Real* my_vertices_end = my_vertices_start + _mesh->vertices().size();

    // Pre-compute Rigid-Deformable Collision Lookup
    if (_vbd_rigid_collisions.size() != num_verts) _vbd_rigid_collisions.resize(num_verts);
    for(auto& vec : _vbd_rigid_collisions) vec.clear();

    using RigidCollisionProjectorType = Solver::RigidBodyConstraintProjector<IsFirstOrder, Solver::RigidDeformableCollisionConstraint>;
    const auto& rigid_coll_projectors = _solver.template getConstraintProjectorsOfType<RigidCollisionProjectorType>();

    for(const auto& projector : rigid_coll_projectors) {
        if(!projector.isValid()) continue;
        const auto* c = &projector.constraint().get(); 
        if(!c) continue; 
        
        const auto& positions = c->positions();
        for(const auto& pos_ref : positions) {
            if(pos_ref.index < num_verts) {
                _vbd_rigid_collisions[pos_ref.index].push_back(c);
            }
        }
    }

    // Pre-compute Static-Deformable Collision Lookup
    if (_vbd_static_collisions.size() != num_verts) _vbd_static_collisions.resize(num_verts);
    for(auto& vec : _vbd_static_collisions) vec.clear();

    using StaticCollisionProjectorType = Solver::ConstraintProjector<IsFirstOrder, Solver::StaticDeformableCollisionConstraint>;
    const auto& static_coll_projectors = _solver.template getConstraintProjectorsOfType<StaticCollisionProjectorType>();

    for(const auto& projector : static_coll_projectors) {
        if(!projector.isValid()) continue;
        const auto* c = &projector.constraint().get(); 
        if(!c) continue; 
        
        const auto& positions = c->positions();
        for(const auto& pos_ref : positions) {
            if(pos_ref.index < num_verts) {
                _vbd_static_collisions[pos_ref.index].push_back(c);
            }
        }
    }

    // Pre-compute Rigid-Deform Adhesion Lookup
    if (_vbd_rigid_adhesions.size() != num_verts) _vbd_rigid_adhesions.resize(num_verts);
    for(auto& vec : _vbd_rigid_adhesions) vec.clear();

    using AdhesionProjectorType = Solver::RigidBodyConstraintProjector<IsFirstOrder, Solver::RigidDeformAdhesionConstraint>;
    const auto& adhesion_projectors = _solver.template getConstraintProjectorsOfType<AdhesionProjectorType>();

    for(const auto& projector : adhesion_projectors) {
        if(!projector.isValid()) continue;
        const auto* c = &projector.constraint().get(); 
        if(!c) continue; 
        
        const auto& positions = c->positions();
        for(const auto& pos_ref : positions) {
            if(pos_ref.index < num_verts) {
                _vbd_rigid_adhesions[pos_ref.index].push_back(c);
            }
        }
    }

    // Pre-compute Inter-Deformable Collision Lookup
    if (_vbd_inter_deform_collisions.size() != num_verts) _vbd_inter_deform_collisions.resize(num_verts);
    for(auto& vec : _vbd_inter_deform_collisions) vec.clear();

    using InterDeformCollisionProjectorType = Solver::ConstraintProjector<IsFirstOrder, Solver::InterObjectDeformableCollisionConstraint>;
    const auto& inter_deform_coll_projectors = _solver.template getConstraintProjectorsOfType<InterDeformCollisionProjectorType>();

    for(const auto& projector : inter_deform_coll_projectors) {
        if(!projector.isValid()) continue;
        const auto* c = &projector.constraint().get(); 
        if(!c) continue; 
        
        const auto& positions = c->positions();
        for(const auto& pos_ref : positions) {
            const Real* ptr = pos_ref.position_ptr;
            if(ptr >= my_vertices_start && ptr < my_vertices_end) {
                if (pos_ref.index < num_verts) {
                    _vbd_inter_deform_collisions[pos_ref.index].push_back(c);
                }
            }
        }
    }

    // Pre-compute Inter-Deformable Adhesion Lookup
    if (_vbd_inter_deform_adhesions.size() != num_verts) _vbd_inter_deform_adhesions.resize(num_verts);
    for(auto& vec : _vbd_inter_deform_adhesions) vec.clear();

    using InterDeformAdhesionProjectorType = Solver::ConstraintProjector<IsFirstOrder, Solver::InterDeformDeformAdhesionConstraint>;
    const auto& inter_deform_adhesion_projectors = _solver.template getConstraintProjectorsOfType<InterDeformAdhesionProjectorType>();

    for(const auto& projector : inter_deform_adhesion_projectors) {
        if(!projector.isValid()) continue;
        const auto* c = &projector.constraint().get(); 
        if(!c) continue; 
        
        const auto& positions = c->positions();
        for(const auto& pos_ref : positions) {
            const Real* ptr = pos_ref.position_ptr;
            if(ptr >= my_vertices_start && ptr < my_vertices_end) {
                if (pos_ref.index < num_verts) {
                    _vbd_inter_deform_adhesions[pos_ref.index].push_back(c);
                }
            }
        }
    }

    // Pre-compute Self-Collision Lookup
    if (_vbd_self_collisions.size() != num_verts) _vbd_self_collisions.resize(num_verts);
    for(auto& vec : _vbd_self_collisions) vec.clear();

    using SelfCollisionProjectorType = Solver::ConstraintProjector<IsFirstOrder, Solver::DeformableDeformableCollisionConstraint>;
    const auto& self_coll_projectors = _solver.template getConstraintProjectorsOfType<SelfCollisionProjectorType>();

    for(const auto& projector : self_coll_projectors) {
        if(!projector.isValid()) continue;
        const auto* c = &projector.constraint().get(); 
        if(!c) continue; 
        
        const auto& positions = c->positions();
        for(const auto& pos_ref : positions) {
            const Real* ptr = pos_ref.position_ptr;
            if(ptr >= my_vertices_start && ptr < my_vertices_end) {
                if (pos_ref.index < num_verts) {
                    _vbd_self_collisions[pos_ref.index].push_back(c);
                }
            }
        }
    }

    _vbd_constraints_dirty = false;
}
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

// NerveOnly constraint config  
using SolverTypesNerveOnly = XPBDObjectSolverTypes<false, typename XPBDMeshObjectConstraintConfigurations<false>::NerveOnly::projector_type_list>;
using NerveOnlyConstraints = typename XPBDMeshObjectConstraintConfigurations<false>::NerveOnly::constraint_type_list;

template class XPBDMeshObject_<false, SolverTypesNerveOnly::GaussSeidel, NerveOnlyConstraints>;
template class XPBDMeshObject_<false, SolverTypesNerveOnly::Jacobi, NerveOnlyConstraints>;
template class XPBDMeshObject_<false, SolverTypesNerveOnly::ParallelJacobi, NerveOnlyConstraints>;

// First Order Stable Neohookean constraint config
using FirstOrderSolverTypesStableNeohookean = XPBDObjectSolverTypes<true, typename XPBDMeshObjectConstraintConfigurations<true>::StableNeohookean::projector_type_list>;
using FirstOrderStableNeohookeanConstraints = typename XPBDMeshObjectConstraintConfigurations<true>::StableNeohookean::constraint_type_list;

template class XPBDMeshObject_<true, FirstOrderSolverTypesStableNeohookean::GaussSeidel, FirstOrderStableNeohookeanConstraints>;
template class XPBDMeshObject_<true, FirstOrderSolverTypesStableNeohookean::Jacobi, FirstOrderStableNeohookeanConstraints>;
template class XPBDMeshObject_<true, FirstOrderSolverTypesStableNeohookean::ParallelJacobi, FirstOrderStableNeohookeanConstraints>;

// First Order Stable Neohookean Combined constraint config
using FirstOrderSolverTypesStableNeohookeanCombined = XPBDObjectSolverTypes<true, typename XPBDMeshObjectConstraintConfigurations<true>::StableNeohookeanCombined::projector_type_list>;
using FirstOrderStableNeohookeanCombinedConstraints = typename XPBDMeshObjectConstraintConfigurations<true>::StableNeohookeanCombined::constraint_type_list;

template class XPBDMeshObject_<true, FirstOrderSolverTypesStableNeohookeanCombined::GaussSeidel, FirstOrderStableNeohookeanCombinedConstraints>;
template class XPBDMeshObject_<true, FirstOrderSolverTypesStableNeohookeanCombined::Jacobi, FirstOrderStableNeohookeanCombinedConstraints>;
template class XPBDMeshObject_<true, FirstOrderSolverTypesStableNeohookeanCombined::ParallelJacobi, FirstOrderStableNeohookeanCombinedConstraints>;

// First Order Nerve-Only constraint config
using FirstOrderSolverTypesNerveOnly = XPBDObjectSolverTypes<true, typename XPBDMeshObjectConstraintConfigurations<true>::NerveOnly::projector_type_list>;
using FirstOrderNerveOnlyConstraints = typename XPBDMeshObjectConstraintConfigurations<true>::NerveOnly::constraint_type_list;


template class XPBDMeshObject_<true, FirstOrderSolverTypesNerveOnly::GaussSeidel, FirstOrderNerveOnlyConstraints>;
template class XPBDMeshObject_<true, FirstOrderSolverTypesNerveOnly::Jacobi, FirstOrderNerveOnlyConstraints>;
template class XPBDMeshObject_<true, FirstOrderSolverTypesNerveOnly::ParallelJacobi, FirstOrderNerveOnlyConstraints>;

} // namespace Sim


