#ifndef __XPBD_MESH_OBJECT_HPP
#define __XPBD_MESH_OBJECT_HPP

#include "config/XPBDMeshObjectConfig.hpp"
#include "simobject/Object.hpp"
#include "simobject/MeshObject.hpp"
#include "simobject/ElasticMaterial.hpp"
// #include "solver/XPBDSolver.hpp"
// #include "solver/Constraint.hpp"

// TODO: fix circular dependency and remove the need for this forward declaration
namespace Solver
{
    class Constraint;
    class CollisionConstraint;
    class ConstraintProjector;
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

/** A class for solving the dynamics of elastic, highly deformable materials with the XPBD method described in
 *  "A Constraint-based Formulation of Stable Neo-Hookean Materials" by Macklin and Muller (2021).
 *  Refer to the paper and preceding papers for details on the XPBD approach.
 */
class XPBDMeshObject : public Object, public TetMeshObject
{
    public:
    /** Creates a new XPBDMeshObject from a YAML config node
     * @param name : the name of the new XPBDMeshObject
     * @param config : the YAML node dictionary describing the parameters for the new XPBDMeshObject
     */
    explicit XPBDMeshObject(const Simulation* sim, const XPBDMeshObjectConfig* config);

    virtual ~XPBDMeshObject();

    virtual std::string toString(const int indent) const override;
    virtual std::string type() const override { return "XPBDMeshObject"; }

    const ElasticMaterial& material() const { return _material; }
    Solver::XPBDSolver const* solver() const { return _solver.get(); }

    int numConstraints() const { return _elastic_constraints.size() + _collision_constraints.size(); }
    const std::vector<std::unique_ptr<Solver::Constraint>>& elasticConstraints() const { return _elastic_constraints; }

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
                                    const XPBDMeshObject* obj, const int v1, const int v2, const int v3, const Real u, const Real v, const Real w);

    void addRigidDeformableCollisionConstraint(const Geometry::SDF* sdf, Sim::RigidObject* rigid_obj, const Vec3r& rigid_body_point, const Vec3r& collision_normal,
                                       const Sim::XPBDMeshObject* deformable_obj, const int v1, const int v2, const int v3, const Real u, const Real v, const Real w);

    void clearCollisionConstraints();

    void removeOldCollisionConstraints(const int threshold);

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

    /** Creates constraints according to the specified constraint type and options.
     * @param constraint_type - the type of constraints to create and apply to the elements of the mesh. One of the options specified in the XPBDConstraintType enum.
     * @param with_residual - whether or not to include the primary residual in the update formula for constraint projection.
     * @param with_damping - whether or not to include XPBD damping (as proposed in the original XPBD paper) in the update formula for constraint projection.
     * @param first_order - whether or not to reformulate constraints as "first order". Should only be true for FirstOrderXPBDMeshObjects.
     */
    void _createConstraints(XPBDConstraintType constraint_type, bool with_residual, bool with_damping, bool first_order);

    /** Helper method to "decorate" (i.e. extend) a ConstraintProjector with the specified options.
     * @param projector - the ConstraintProjector to decorate
     * @param with_residual - whether or not to include the primary residual in the update formula for constraint projection.
     * @param with_damping - whether or not to include XPBD damping (as proposed in the original XPBD paper) in the update formula for constraint projection.
     * @param first_order - whether or not to reformulate constraints as "first order". Should only be true for FirstOrderXPBDMeshObjects.
     * 
     * @returns the decorated ConstraintProjector 
     */
    std::unique_ptr<Solver::ConstraintProjector> _decorateConstraintProjector(std::unique_ptr<Solver::ConstraintProjector> projector, bool with_residual, bool with_damping, bool first_order);

    /** Creates a XPBDSolver based on the specified solver type and options. The XPBDSolver is responsible for performing the constraint projection.
     * @param solver_type - the type of solver to create. One of the options specified in the XPBDSolverType enum.
     * @param num_solver_iters - the number of solver iterations the solver should perform each time step.
     * @param residual_policy - dictates how often the solver should compute the residuals. The residuals are useful for measuring how accurate the solver is.
     */
    void _createSolver(XPBDSolverType solver_type, int num_solver_iters, XPBDResidualPolicy residual_policy);

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

    XPBDSolverType _solver_type;            // the type of solver to create - set by the Config object
    XPBDResidualPolicy _residual_policy;    // how often the solver should compute the residuals - set by the Config object
    int _num_solver_iters;             // number of iterations the solver should have - set by the Config object

    Real _damping_gamma;                  // the amount of damping per constraint. gamma = alpha_tilde * beta_tilde / dt (see Equation (26) in the XPBD paper for more details.)
    
    XPBDConstraintType _constraint_type;    // the type of constraints to create - set by the Config object
    bool _constraints_with_residual;        // whether or not the constraints should include the primary residual in their update - set by the Config object
    bool _constraints_with_damping;         // whether or not the constraints should include damping in their update - set by the Config object

    std::unique_ptr<Solver::XPBDSolver> _solver;       // the XPBDSolver that will project the constraints
    std::vector<std::unique_ptr<Solver::Constraint>> _elastic_constraints;  // the array of constraints applied to the elements of the mesh
    // std::vector<std::unique_ptr<Solver::CollisionConstraint>> _collision_constraints;
    // std::vector<int> _collision_constraint_projector_indices;
    std::vector<XPBDCollisionConstraint> _collision_constraints;
};

} // namespace Sim

#endif // __XPBD_MESH_OBJECT_HPP