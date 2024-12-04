#ifndef __XPBD_MESH_OBJECT_HPP
#define __XPBD_MESH_OBJECT_HPP

#include "config/XPBDMeshObjectConfig.hpp"
#include "simobject/ElasticMeshObject.hpp"
// #include "solver/XPBDSolver.hpp"
// #include "solver/Constraint.hpp"

// TODO: fix circular dependency and remove the need for this forward declaration
namespace Solver
{
    class Constraint;
    class ConstraintProjector;
    class XPBDSolver;
}

/** A class for solving the dynamics of elastic, highly deformable materials with the XPBD method described in
 *  "A Constraint-based Formulation of Stable Neo-Hookean Materials" by Macklin and Muller (2021).
 *  Refer to the paper and preceding papers for details on the XPBD approach.
 */
class XPBDMeshObject : public ElasticMeshObject
{
    public:
    /** Creates a new XPBDMeshObject from a YAML config node
     * @param name : the name of the new XPBDMeshObject
     * @param config : the YAML node dictionary describing the parameters for the new XPBDMeshObject
     */
    explicit XPBDMeshObject(const XPBDMeshObjectConfig* config);

    virtual ~XPBDMeshObject();

    virtual std::string toString() const override;
    virtual std::string type() const override { return "XPBDMeshObject"; }

    Solver::XPBDSolver const* solver() const { return _solver.get(); }

    unsigned numConstraints() const { return _constraints.size(); }
    const std::vector<std::unique_ptr<Solver::Constraint>>& constraints() const { return _constraints; }

    /** Performs any one-time setup that needs to be done outside the constructor. */
    virtual void setup() override;

    /** Steps forward one time step. */
    virtual void update() override;

    /** Returns the number of constraints that share the position at the specified index.
     * This is the factor by which to scale the residual in the distributed primary residual update methods.
     */
    unsigned numConstraintsForPosition(const unsigned index) const;

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
    void _updateVelocities();

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
    void _createSolver(XPBDSolverType solver_type, unsigned num_solver_iters, XPBDResidualPolicy residual_policy);

    private:
    /** Helper method to initialize upon instantiation */
    void _init();

    protected:
    XPBDSolverType _solver_type;            // the type of solver to create - set by the Config object
    XPBDResidualPolicy _residual_policy;    // how often the solver should compute the residuals - set by the Config object
    unsigned _num_solver_iters;             // number of iterations the solver should have - set by the Config object

    double _damping_gamma;                  // the amount of damping per constraint. gamma = alpha_tilde * beta_tilde / dt (see Equation (26) in the XPBD paper for more details.)
    
    XPBDConstraintType _constraint_type;    // the type of constraints to create - set by the Config object
    bool _constraints_with_residual;        // whether or not the constraints should include the primary residual in their update - set by the Config object
    bool _constraints_with_damping;         // whether or not the constraints should include damping in their update - set by the Config object

    std::unique_ptr<Solver::XPBDSolver> _solver;       // the XPBDSolver that will project the constraints
    std::vector<std::unique_ptr<Solver::Constraint>> _constraints;  // the array of constraints applied to the elements of the mesh
    

};

#endif // __XPBD_MESH_OBJECT_HPP