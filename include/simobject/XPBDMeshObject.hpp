#ifndef __XPBD_MESH_OBJECT_HPP
#define __XPBD_MESH_OBJECT_HPP

// #include "config/XPBDMeshObjectConfig.hpp"
// #include "simobject/Object.hpp"
// #include "simobject/MeshObject.hpp"
#include "simobject/XPBDMeshObjectBase.hpp"
#include "simobject/ElasticMaterial.hpp"
#include "common/XPBDTypedefs.hpp"

// #include "solver/XPBDSolverUpdates.hpp"

#include "common/VariadicVectorContainer.hpp"
#include "common/TypeList.hpp"

#include "geometry/AABB.hpp"
#include "geometry/Mesh.hpp"

#ifdef HAVE_CUDA
#include "gpu/resource/XPBDMeshObjectGPUResource.hpp"
#endif

namespace Sim
{

class RigidObject;

/** A class for solving the dynamics of elastic, highly deformable materials with the XPBD method described in
 *  "A Constraint-based Formulation of Stable Neo-Hookean Materials" by Macklin and Muller (2021).
 *  Refer to the paper and preceding papers for details on the XPBD approach.
 */
template<bool IsFirstOrder, typename SolverType, typename ConstraintTypeList> class XPBDMeshObject_;

template<typename SolverType, typename ConstraintTypeList>
using XPBDMeshObject = XPBDMeshObject_<false, SolverType, ConstraintTypeList>;

template<typename SolverType, typename ConstraintTypeList>
using FirstOrderXPBDMeshObject = XPBDMeshObject_<true, SolverType, ConstraintTypeList>;

// TODO: should the template parameters be SolverType, XPBDMeshObjectConstraintConfiguration?
// if we have an XPBDObject base class that is templated with <SolverType, ...ConstraintTypes>, we can get constraints from XPBDMeshObjectConstraintConfiguration
// this way, we can use if constexpr (std::is_same_v<XPBDMeshObjectConstraintConfiguration, XPBDMeshObjectConstraintConfigurations::StableNeohookean) which is maybe a more direct comparison
//  instead of using a variant variable
template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
class XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>> : public XPBDMeshObject_Base_<IsFirstOrder>
{
    using Base = XPBDMeshObject_Base_<IsFirstOrder>;
    // bring members and methods of base class into current scope (then we don't have to use this-> everywhere)
    // methods
    using Base::fixVertex;
    using Base::vertexFixed;
    using Base::vertexMass;
    using Base::vertexVelocity;
    using Base::vertexPreviousPosition;
    using Base::vertexConstraintInertia;

    using Base::tetMesh;
    using Base::loadAndConfigureMesh;
    // members
    using Base::_previous_vertices;
    using Base::_vertex_velocities;
    using Base::_initial_velocity;
    using Base::_materials;
    using Base::_vertex_masses;
    using Base::_vertex_volumes;
    using Base::_is_fixed_vertex;
    using Base::_sdf;
    using Base::_damping_multiplier;
    using Base::_vertex_B;

    using Base::_mesh;

    using Base::_sim;
   #ifdef HAVE_CUDA
    friend class XPBDMeshObjectGPUResource;
   #endif
    public:
    using SDFType = typename Base::SDFType;
    using ConfigType =  typename Base::ConfigType;

    public:
    /** Creates a new XPBDMeshObject from a YAML config node
     * @param name : the name of the new XPBDMeshObject
     * @param config : the YAML node dictionary describing the parameters for the new XPBDMeshObject
     */
    // TODO: parameter pack in constructor for ConstraintTypes type deduction. Maybe move this to XPBDMeshObjectConfig?
    explicit XPBDMeshObject_(const Simulation* sim, const ConfigType* config);

    virtual ~XPBDMeshObject_();

    virtual std::string toString(const int indent) const override;
    virtual std::string type() const override { return "XPBDMeshObject"; }

    /** Performs any one-time setup that needs to be done outside the constructor. */
    virtual void setup() override;

    /** Steps forward one time step. */
    virtual void update() override;

    virtual void velocityUpdate() override;

    /** Returns the AABB around this object. */
    virtual Geometry::AABB boundingBox() const override;

    /** === Adding/removing additional constraints === */

    /** Adds a collision constraint between a face on this object and a point on a static object in the scene.
     * @param sdf : the SDF of the static object
     * @param surface_point : the surface point on the static object
     * @param collision_normal : the collision normal
     * @param face_ind : the index of the face in collision
     * @param u,v,w : the barycentric coordinates of the point on the face in collision
     * @returns a reference to the constraint projector that was added for the collision constraint
     */
    virtual Solver::ConstraintProjectorReference<Solver::ConstraintProjector<IsFirstOrder, Solver::StaticDeformableCollisionConstraint>>
    addStaticCollisionConstraint(const Geometry::SDF* sdf, const Vec3r& surface_point, const Vec3r& collision_normal,
        int face_ind, const Real u, const Real v, const Real w) override;

    /** Adds a collision constraint between a face on this object and a point on a rigid object in the scene.
     * @param sdf : the SDF of the rigid object
     * @param rigid_obj : a pointer to the rigid object in collision
     * @param rigid_body_point : the surface point on the rigid object in collision (global coordinates)
     * @param collision_normal : the collision normal
     * @param face_ind : the index of the face in collision
     * @param u,v,w : the barycentric coordinates of the point on the face in collision
     * @returns a reference to the constraint projector that was added for the collision constraint
     */
    virtual Solver::ConstraintProjectorReference<Solver::RigidBodyConstraintProjector<IsFirstOrder, Solver::RigidDeformableCollisionConstraint>>
    addRigidDeformableCollisionConstraint(const Geometry::SDF* sdf, Sim::RigidObject* rigid_obj, const Vec3r& rigid_body_point, const Vec3r& collision_normal,
        int face_ind, const Real u, const Real v, const Real w) override;

    /** Clears all collision constraints that are on this object. */
    virtual void clearCollisionConstraints() override;
    
    /** Adds an attachment constraint applied to the vertex at the specified index. TODO: clean this up a bit? The Vec3r pointer is a bit gross.
     * @param v_ind : the index of the vertex
     * @param attach_pos_ptr : a pointer to the position for the vertex to be attached to
     * @param attachment_offset : an optional offset between the attachment position and the vertex. The vertex position will be (*attach_pos_ptr + attachment_offset).
     */
    virtual Solver::ConstraintProjectorReference<Solver::ConstraintProjector<IsFirstOrder, Solver::AttachmentConstraint>>  
    addAttachmentConstraint(int v_ind, const Vec3r* attach_pos_ptr, const Vec3r& attachment_offset) override;

    /** Clears all attachment constraint that are on this object. */
    virtual void clearAttachmentConstraints() override;

    /** === Querying the solver === */

    /** @returns the most recently calculated primary residual from the solver object */
    virtual VecXr lastPrimaryResidual() const override { return _solver.primaryResidual(); };

    /** @returns the most recently calculated constraint residual from the solver object */
    virtual VecXr lastConstraintResidual() const override { return _solver.constraintResidual(); }

    /** === Miscellaneous useful methods === */

    /** Computes the total strain energy associated with elastic deformation.
    */
    virtual Real totalStrainEnergy() const override;

    /** Computes the elastic force on the vertex at the specified index. This is essentially just the current constraint force for all "elastic" constraints
     * that affect the specified vertex. An "elastic" constraint is one that is internal to the mesh and corresponds to the mechanics of the mesh material.
     * @param index : the index of the vertex
     * @returns the elastic force vector on the vertex at the specified index
     */
    virtual Vec3r elasticForceAtVertex(int index) const override;

    /** Computes the current global stiffness matrix of the mesh. This is done with a first-order approximation of delC^T * alpha * delC.
     * @returns the global stiffness matrix
     */
    virtual MatXr stiffnessMatrix() const override;

    /** Performs a check for self collision.
     * If any surface vertices are inside tetrahedra (queries made using Embree), add a collision constraint to fix that.
     * Assumes that the Embree scene is up to date.
     */
    virtual void selfCollisionCheck() override;


 #ifdef HAVE_CUDA
    virtual void createGPUResource() override;
    virtual XPBDMeshObjectGPUResource* gpuResource() override;
    virtual const XPBDMeshObjectGPUResource* gpuResource() const override;
 #endif

    protected:
    /** Moves the vertices in the absence of constraints.
     * i.e. according to their current velocities and the external forces applied to them
     */
    virtual void _movePositionsInertially();

    /** Projects the constraints onto the inertial positions and updates the mesh positions accordingly to satisfy the constraints.
     * Uses the XPBD algorithm to perform the constraint projection.
     */
    void _projectConstraints();

    /** Update the velocities based on the updated positions.
     */
    // void _updateVelocities();

    virtual void _calculatePerVertexQuantities();

    /** Creates constraints according to the constraint type.
     */
    void _createElasticConstraints();

    /** Assembles a vector of constraint projector references corresponding to constraints that are nearby active collision constraints.
     * These are then passed to the solver to re-project and update the mesh.
     * We also include all collision constraints (including currently inactive ones) to maintain a consistent contact manifold.
     * 
     * "Nearby" constraints are those that share a vertex with an active collision constraint. E.g., the hydrostatic and deviatoric constraints
     * for all elements that share a vertex with an active collision constraint.
     * 
     * We are not concerned with duplicate constraint projectors in this vector since we are doing multiple iterations anyways - probably just
     * faster to add all constraint projectors than try and make a unique set.
     */
    typename SolverType::projector_reference_container_type _gatherProjectorsForLocalCollisionIterations();

    protected:
    /** The specific constraint configuration used to define internal constraints for the XPBD mesh. Set by the Config object
     * TODO: is this necessary? Should XPBDMeshObjectConstraintConfiguration be a struct that can create the elastic constraints for the mesh?
     */
    XPBDMeshObjectConstraintConfigurationEnum _constraint_type;

    /** The XPBD solver. Responsible for iterating through constraints and computing the XPBD positional updates.
     * The XPBD projection is implemented in ConstraintProjector. The solver is just responsible for iterating/aggregating and 
     * applying the results from the XPBD projections.
     */
    SolverType _solver;

    /** A heterogeneous container of all the constraints.
     */
    VariadicVectorContainer<ConstraintTypes...> _constraints;

    /** The number of local iterations for collision area.
     * Constraint projectors in the vicinity of active collision constraints (see _gatherProjectorsForLocalCollisionIterations) are assembled
     * and re-projected multiple times, which helps propagate the deformation imposed by collision constraints to the rest of the mesh.
     * Called "local" iterations since only a subset of the constraint projectors are being re-projected.
     * 
     * This is set by the config object.
     */
    int _num_local_collision_iters;

    /** The filename that has information about which class each element belongs to. Set by the config. */
    std::optional<std::string> _element_classes_filename;
};

} // namespace Sim

#endif // __XPBD_MESH_OBJECT_HPP