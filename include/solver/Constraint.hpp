#ifndef __CONSTRAINT_HPP
#define __CONSTRAINT_HPP

#include <vector>
#include <Eigen/Dense>

#include "simobject/XPBDMeshObject.hpp"

namespace Solver
{

/** Struct for storing references to positions in MeshObjects.
 * Used by Constraints to access node properties dynamically.
 */
struct PositionReference
{
    XPBDMeshObject* obj;        // pointer to the MeshObject
    unsigned index;             // vertex index
    
    /** Helper function to get the vertex position. */
    inline Eigen::Vector3d position() const
    {
        return obj->getVertex(index);
    }

    inline Eigen::Vector3d previousPosition() const
    {
        return obj->vertexPreviousPosition(index);
    }

    /** Helper function to get the vertex velocity. */
    inline Eigen::Vector3d velocity() const
    {
        return obj->vertexVelocity(index);
    }

    /** Helper function to get the vertex mass. */
    inline double mass() const
    {
        return obj->vertexMass(index);
    }

    inline double invMass() const
    {
        return obj->vertexInvMass(index);
    }

    /** Helper function to get number of attached elements to the vertex. */
    inline unsigned attachedElements() const
    {
        return obj->vertexAttachedElements(index);
    }
};



class Constraint
{
    friend class ConstraintDecorator;
    public:

    typedef std::pair<double, Eigen::VectorXd> ValueAndGradient;
    typedef std::pair<PositionReference, Eigen::Vector3d> PositionUpdate;

    Constraint(const double dt)
        : _dt(dt), _alpha(0), _lambda(0)
    {

    }

    inline virtual void initialize()
    {
        _lambda = 0;
    }

    /** Evaluates the current value of this constraint.
     * i.e. returns C(x)
     */
    inline virtual double evaluate() const = 0;

    /** Returns the gradient of this constraint in vector form.
     * i.e. returns delC(x)
     */
    inline virtual Eigen::VectorXd gradient() const = 0;

    /** Returns the value and gradient of this constraint.
     * i.e. returns C(x) and delC(x) together.
     */
    inline virtual ValueAndGradient evaluateWithGradient() const = 0;

    inline virtual std::vector<PositionUpdate> project()
    {
        const ValueAndGradient val_and_grad = evaluateWithGradient();
        const double dlam = _RHS(val_and_grad) / _LHS(val_and_grad);

        std::vector<PositionUpdate> position_updates(_positions.size());
        for (unsigned i = 0; i < _positions.size(); i++)
        {
            position_updates.at(i) = std::move(_getPositionUpdate(i, dlam, val_and_grad.second));
        }

        _lambda += dlam;

        return position_updates;
    }

    /** Returns the value of the Lagrange multiplier associated with this constraint. */
    double lambda() const { return _lambda; }
    
    /** Returns the compliance for this constraint. */
    double alpha() const { return _alpha; }

    /** Returns the alpha tilde for this constraint. */
    virtual double alphaTilde() const { return _alpha / (_dt*_dt); }

    /** Returns the inverse mass for the position at the specified index. */
    virtual double positionInvMass(const unsigned position_index) const { return _positions[position_index].invMass(); }

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
    inline virtual double _LHS(const ValueAndGradient& val_and_grad) const
    {
        const double alpha_tilde = alphaTilde();
        const Eigen::VectorXd& grads = val_and_grad.second;
        double lhs = alpha_tilde;
        for (unsigned i = 0; i < _positions.size(); i++)
        {
            const double inv_m = positionInvMass(i);
            lhs += inv_m * grads(Eigen::seq(3*i, 3*i+2)).squaredNorm();
        }

        return lhs;
    }

    inline virtual double _RHS(const ValueAndGradient& val_and_grad) const
    {
        const double alpha_tilde = alphaTilde();
        const double C = val_and_grad.first;
        double rhs = -C - alpha_tilde * _lambda;

        return rhs;
    }

    inline virtual PositionUpdate _getPositionUpdate(const unsigned position_index, const double dlam, const Eigen::VectorXd& grads) const
    {
        PositionUpdate position_update;
        position_update.first = _positions[position_index];
        position_update.second = positionInvMass(position_index) * dlam * grads(Eigen::seq(3*position_index, 3*position_index+2));
        return position_update;
    }

    protected:
    double _dt;         // the size of the time step used during constraint projection
    double _lambda;     // Lagrange multiplier for this constraint
    double _alpha;      // Compliance for this constraint
    std::vector<PositionReference> _positions;  // the positions associated with this constraint
};  


} // namespace Solver

#endif // __CONSTRAINT_HPP