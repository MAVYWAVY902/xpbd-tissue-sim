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
// #include "solver/XPBDSolver.hpp"
// #include "solver/Constraint.hpp"

// TODO: fix circular dependency and remove the need for this forward declaration
namespace Solver
{
    class DeviatoricConstraint;
    class HydrostaticConstraint;
    class StaticDeformableCollisionConstraint;
    class RigidDeformableCollisionConstraint;
    class CollisionConstraint;
    class AttachmentConstraint;

    template<bool, class T>
    class ConstraintProjector;

    template<bool, class T1, class T2>
    class CombinedConstraintProjector;

    template<bool, class... Ts>
    class XPBDSolver;
}

namespace Geometry
{
    class SDF;
}

namespace Sim
{

class RigidObject;

struct XPBDCollisionConstraint
{
    std::unique_ptr<Solver::CollisionConstraint> constraint;
    int projector_index;
    int num_steps_unused;
};

struct XPBDAttachmentConstraint
{
    std::unique_ptr<Solver::AttachmentConstraint> constraint;
    int projector_index;
};

/** A class for solving the dynamics of elastic, highly deformable materials with the XPBD method described in
 *  "A Constraint-based Formulation of Stable Neo-Hookean Materials" by Macklin and Muller (2021).
 *  Refer to the paper and preceding papers for details on the XPBD approach.
 */
template<typename SolverType, typename ConstraintTypeList> class XPBDMeshObject;

// TODO: should the template parameters be SolverType, XPBDMeshObjectConstraintConfiguration?
// if we have an XPBDObject base class that is templated with <SolverType, ...ConstraintTypes>, we can get constraints from XPBDMeshObjectConstraintConfiguration
// this way, we can use if constexpr (std::is_same_v<XPBDMeshObjectConstraintConfiguration, XPBDMeshObjectConstraintConfigurations::StableNeohookean) which is maybe a more direct comparison
//  instead of using a variant variable
template<typename SolverType, typename... ConstraintTypes>
class XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>> : public XPBDMeshObject_Base
{
   #ifdef HAVE_CUDA
    friend class XPBDMeshObjectGPUResource;
   #endif

    public:
    /** Creates a new XPBDMeshObject from a YAML config node
     * @param name : the name of the new XPBDMeshObject
     * @param config : the YAML node dictionary describing the parameters for the new XPBDMeshObject
     */
    // TODO: parameter pack in constructor for ConstraintTypes type deduction. Maybe move this to XPBDMeshObjectConfig?
    explicit XPBDMeshObject(const Simulation* sim, const XPBDMeshObjectConfig* config);

    virtual ~XPBDMeshObject();

    virtual std::string toString(const int indent) const override;
    virtual std::string type() const override { return "XPBDMeshObject"; }

    const ElasticMaterial& material() const { return _material; }

    // int numConstraints() const { return _elastic_constraints.size() + _collision_constraints.size(); }
    // const std::vector<std::unique_ptr<Solver::Constraint>>& elasticConstraints() const { return _elastic_constraints; }

    Real* vertexPreviousPositionPointer(const int index) const { return const_cast<Real*>(_previous_vertices.col(index).data()); }

    void fixVertex(int index) { _is_fixed_vertex[index] = true; }

    bool vertexFixed(int index) const { return _is_fixed_vertex[index]; }

    Real vertexMass(int index) const { return _vertex_masses[index]; }

    Real vertexInvMass(int index) const { return _vertex_inv_masses[index]; }

    int vertexAttachedElements(int index) const { return _vertex_attached_elements[index]; }

    Vec3r vertexVelocity(int index) const { return _vertex_velocities.col(index); }

    Vec3r vertexPreviousPosition(int index) const { return _previous_vertices.col(index); }

    /** Performs any one-time setup that needs to be done outside the constructor. */
    virtual void setup() override;

    /** Steps forward one time step. */
    virtual void update() override;

    virtual void velocityUpdate() override;

    /** Returns the AABB around this object. */
    virtual Geometry::AABB boundingBox() const override;

    /** Returns the number of constraints that share the position at the specified index.
     * This is the factor by which to scale the residual in the distributed primary residual update methods.
     */
    int numConstraintsForPosition(const int index) const;

    void addStaticCollisionConstraint(const Geometry::SDF* sdf, const Vec3r& p, const Vec3r& n,
                                    int face_ind, const Real u, const Real v, const Real w);

    void addVertexStaticCollisionConstraint(const Geometry::SDF* sdf, const Vec3r& p, const Vec3r& n, int vert_ind);

    void addRigidDeformableCollisionConstraint(const Geometry::SDF* sdf, Sim::RigidObject* rigid_obj, const Vec3r& rigid_body_point, const Vec3r& collision_normal,
                                       int face_ind, const Real u, const Real v, const Real w);

    void clearCollisionConstraints();

    void removeOldCollisionConstraints(const int threshold);

    void addAttachmentConstraint(int v_ind, const Vec3r* attach_pos_ptr, const Vec3r& attachment_offset);

    void clearAttachmentConstraints();

    Vec3r elasticForceAtVertex(int index);

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

    protected:
    Geometry::Mesh::VerticesMat _previous_vertices;
    Geometry::Mesh::VerticesMat _vertex_velocities;

    Vec3r _initial_velocity;

    ElasticMaterial _material;

    std::vector<Real> _vertex_masses;
    std::vector<Real> _vertex_inv_masses;
    std::vector<Real> _vertex_volumes;
    std::vector<int> _vertex_attached_elements;
    std::vector<bool> _is_fixed_vertex;

    Real _damping_gamma;                  // the amount of damping per constraint. gamma = alpha_tilde * beta_tilde / dt (see Equation (26) in the XPBD paper for more details.)
    
    XPBDMeshObjectConstraintConfigurationEnum _constraint_type;    // the type of constraints to create - set by the Config object
    bool _constraints_with_residual;        // whether or not the constraints should include the primary residual in their update - set by the Config object
    bool _constraints_with_damping;         // whether or not the constraints should include damping in their update - set by the Config object

    // TODO: generalize constraints somehow
    // std::unique_ptr<    Solver::XPBDSolver<Solver::CombinedConstraintProjector<Solver::DeviatoricConstraint, Solver::HydrostaticConstraint>,
    //                     Solver::ConstraintProjector<Solver::StaticDeformableCollisionConstraint>,
    //                     Solver::ConstraintProjector<Solver::RigidDeformableCollisionConstraint> > > _solver;

    SolverType _solver;

    VariadicVectorContainer<ConstraintTypes...> _constraints;

    // std::unique_ptr<Solver::XPBDSolver> _solver;       // the XPBDSolver that will project the constraints
    // std::vector<std::unique_ptr<Solver::Constraint>> _elastic_constraints;  // the array of constraints applied to the elements of the mesh
    // std::vector<std::unique_ptr<Solver::CollisionConstraint>> _collision_constraints;
    // std::vector<int> _collision_constraint_projector_indices;
    // std::vector<XPBDCollisionConstraint> _collision_constraints;
};

} // namespace Sim

#endif // __XPBD_MESH_OBJECT_HPP