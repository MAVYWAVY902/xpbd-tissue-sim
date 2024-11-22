#ifndef __XPBD_MESH_OBJECT_HPP
#define __XPBD_MESH_OBJECT_HPP

#include "config/XPBDMeshObjectConfig.hpp"
#include "simobject/ElasticMeshObject.hpp"
#include "solver/XPBDSolver.hpp"
#include "solver/Constraint.hpp"

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

    virtual std::string toString() const override;
    virtual std::string type() const override { return "XPBDMeshObject"; }

    Solver::XPBDSolver const* solver() const { return _solver.get(); }

    unsigned numConstraints() const { return _constraints.size(); }
    const std::vector<std::unique_ptr<Solver::Constraint>>& constraints() const { return _constraints; }

    virtual void update() override;

    protected:
    virtual void _createConstraints(XPBDConstraintType constraint_type, bool with_residual, bool with_damping);
    // virtual std::unique_ptr<Solver::Constraint> _createConstraintForElement();

    /** Moves the vertices in the absence of constraints.
     * i.e. according to their current velocities and the forces applied to them
     */
    virtual void _movePositionsInertially();

    void _projectConstraints();

    /** Update the velocities based on the updated positions.
     */
    void _updateVelocities();

    private:
    /** Helper method to initialize upon instantiation */
    void _init();

    /** Precomputes static quantities */
    // void _precomputeQuantities();

    protected:
    XPBDSolverType _solver_type;
    // XPBDConstraintType _constraint_type;

    double _damping_gamma;

    std::unique_ptr<Solver::XPBDSolver> _solver;
    std::vector<std::unique_ptr<Solver::Constraint>> _constraints;
    

};

#endif // __XPBD_MESH_OBJECT_HPP