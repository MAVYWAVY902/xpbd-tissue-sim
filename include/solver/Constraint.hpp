#ifndef __CONSTRAINT_HPP
#define __CONSTRAINT_HPP

#include <vector>
#include <Eigen/Dense>

class XPBDMeshObject;
class FirstOrderXPBDMeshObject;

namespace Solver
{

/** Struct for storing references to positions in MeshObjects.
 * Used by Constraints to access node properties dynamically.
 */
struct PositionReference
{
    XPBDMeshObject* obj;        // pointer to the MeshObject
    unsigned index;             // vertex index

    Eigen::Vector3d position() const;
    Eigen::Vector3d previousPosition() const;
    Eigen::Vector3d velocity() const;
    double mass() const;
    unsigned attachedElements() const;
};



class Constraint
{
    friend class ConstraintDecorator;
    public:

    typedef std::pair<double, Eigen::VectorXd> ValueAndGradient;
    typedef std::pair<PositionReference, Eigen::Vector3d> PositionUpdate;

    explicit Constraint(const double dt);

    /** Initializes this constraint before the solver loop. */
    virtual void initialize();

    /** Projects this constraint.
     * @returns a vector of position updates resulting from the projection
     */
    virtual std::vector<PositionUpdate> project();

    /** Evaluates the current value of this constraint.
     * i.e. returns C(x)
     */
    virtual double evaluate() const = 0;

    /** Returns the gradient of this constraint in vector form.
     * i.e. returns delC(x)
     */
    virtual Eigen::VectorXd gradient() const = 0;

    /** Returns the value and gradient of this constraint.
     * i.e. returns C(x) and delC(x) together.
     */
    virtual ValueAndGradient evaluateWithGradient() const = 0;

    /** Returns the value of the Lagrange multiplier associated with this constraint. */
    double lambda() const { return _lambda; }
    
    /** Returns the compliance for this constraint. */
    double alpha() const { return _alpha; }

    /** Returns the alpha tilde for this constraint. */
    virtual double alphaTilde() const { return _alpha / (_dt*_dt); }

    /** Returns the inverse mass for the position at the specified index. */
    virtual double positionInvMass(const unsigned position_index) const { return 1.0/_positions[position_index].mass(); }

    const std::vector<PositionReference> positions() const { return _positions; }

    /** Whether or not this constraint needs the primary residual to do its constraint projection.
     * By default, this is false, but can be overridden by derived classes to be true if a constraint needs the primary residual.
     * 
     * The Solver class will query each constraint to see if it needs to compute the primary residual is needs to be calculated before each GS iteration.
     */
    virtual bool usesPrimaryResidual() const { return false; }

    /** Whether or not this constraint uses damping in its constraint projection.
     * By default, this is false, but can be overridden by derived classes to be true if the constraint uses damping.
     */
    virtual bool usesDamping() const { return false; }

    protected:
    /** Computes the LHS (denominator) of the lambda udpate. */
    virtual double _LHS(const ValueAndGradient& val_and_grad) const;

    /** Computes the RHS (numerator) of the lambda update. */
    virtual double _RHS(const ValueAndGradient& val_and_grad) const;

    /** Computes the position update given the change in lambda and constraint gradient.
     * @param position_index : the index in the _positions vector of the node to compute the update for.
     * @param dlam : the change in Lagrange multiplier
     * @param grad : the constraint gradient
      */
    virtual PositionUpdate _getPositionUpdate(const unsigned position_index, const double dlam, const Eigen::VectorXd& grad) const;

    protected:
    double _dt;         // the size of the time step used during constraint projection
    double _lambda;     // Lagrange multiplier for this constraint
    double _alpha;      // Compliance for this constraint
    std::vector<PositionReference> _positions;  // the positions associated with this constraint
};  


} // namespace Solver

#endif // __CONSTRAINT_HPP