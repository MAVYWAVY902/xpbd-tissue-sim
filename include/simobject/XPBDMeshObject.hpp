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

    unsigned numConstraintProjectors() const { return _constraint_projectors.size(); }
    const std::vector<std::unique_ptr<Solver::ConstraintProjector>>& constraintProjectors() const { return _constraint_projectors; }

    virtual void setup() override;

    virtual void update() override;

    virtual unsigned numConstraintsForPosition(const unsigned index) const;

    protected:
    /** Moves the vertices in the absence of constraints.
     * i.e. according to their current velocities and the forces applied to them
     */
    virtual void _movePositionsInertially();

    void _projectConstraints();

    /** Update the velocities based on the updated positions.
     */
    void _updateVelocities();

    void _createConstraints(XPBDConstraintType constraint_type, bool with_residual, bool with_damping, bool first_order);

    std::unique_ptr<Solver::ConstraintProjector> _decorateConstraintProjector(std::unique_ptr<Solver::ConstraintProjector> projector, bool with_residual, bool with_damping, bool first_order);

    void _createSolver(XPBDSolverType solver_type, unsigned num_solver_iters, XPBDResidualPolicy residual_policy);

    private:
    /** Helper method to initialize upon instantiation */
    void _init();

    

    /** Precomputes static quantities */
    // void _precomputeQuantities();

    protected:
    XPBDSolverType _solver_type;
    XPBDResidualPolicy _residual_policy;
    unsigned _num_solver_iters;

    double _damping_gamma;
    
    XPBDConstraintType _constraint_type;
    bool _constraints_with_residual;
    bool _constraints_with_damping;

    std::unique_ptr<Solver::XPBDSolver> _solver;
    std::vector<std::unique_ptr<Solver::Constraint>> _constraints;
    std::vector<std::unique_ptr<Solver::ConstraintProjector>> _constraint_projectors;
    

};

#endif // __XPBD_MESH_OBJECT_HPP