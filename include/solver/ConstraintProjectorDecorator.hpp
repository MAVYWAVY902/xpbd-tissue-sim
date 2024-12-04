#ifndef __CONSTRAINT_PROJECTOR_DECORATOR_HPP
#define __CONSTRAINT_PROJECTOR_DECORATOR_HPP

#include "solver/ConstraintProjector.hpp"
#include "simobject/FirstOrderXPBDMeshObject.hpp"
#include <memory>
#include <iomanip>

namespace Solver
{

/** Extends the functionality of ConstraintProjector using the decorator design pattern. https://en.wikipedia.org/wiki/Decorator_pattern
 * This works by essentially wrapping the original ConstraintProjector class, and overriding certain methods to augment functionality.
 * 
 * For example, the WithDamping decorator will override the _LHS() and _RHS() methods to use the XPBD update formula that includes damping.
 * 
 * Because this is done generally, any ConstraintProjector with any type of constraints can be augmented, removing the need for defining classes like:
 *  ConstraintProjectorWithDamping, ConstraintProjectorWithDampingAndPrimaryResidual, FirstOrderConstraintProjectorWithPrimaryResidual, etc.
 * 
 * The base class ConstraintProjectorDecorator merely templates this wrapping behavior and provides helper methods for accessing the wrapped component's protected methods
 * (since ConstraintProjectorDecorator is a friend of ConstraintProjector). 
 */
class ConstraintProjectorDecorator : public ConstraintProjector
{
    public:
    /** Initialize this ConstraintProjector to be the same as the wrapped component. */
    explicit ConstraintProjectorDecorator(std::unique_ptr<ConstraintProjector> component)
        : ConstraintProjector(*component), 
        _component(std::move(component))
    {
    }

    /** Sets the Lagrange multiplier for the constraint at the specified index.
     * This will not only set the Lagrange multiplier for this ConstraintProjector but also the component's, because
     * there are (at least) two versions of the lambda array that need to be updated together, since the component's methods will use the lambdas of the component while the
     * decorator's methods will use the lambdas of the decorator.
     * 
     * @param index - the constraint index
     * @param val - the new Lagrange multiplier
     */
    virtual void setLambda(const unsigned index, const double val) override { _component->setLambda(index, val); _lambda[index] = val; }

    protected:
    /** Give access to the component's _LHS() method. Normally this is a protected helper method but ConstraintProjectorDecorator is a friend of ConstraintProjector. */
    void _componentLHS(const double* delC_ptr, const double* M_inv_ptr, const double* alpha_tilde_ptr, double* lhs_ptr) const
    {
        return _component->_LHS(delC_ptr, M_inv_ptr, alpha_tilde_ptr, lhs_ptr);
    }

    /** Give access to the component's _RHS() method. Normally this is a protected helper method but ConstraintProjectorDecorator is a friend of ConstraintProjector. */
    void _componentRHS(const double* C_ptr, const double* delC_ptr, const double* alpha_tilde_ptr, double* rhs_ptr) const
    {
        return _component->_RHS(C_ptr, delC_ptr, alpha_tilde_ptr, rhs_ptr);
    }

    /** Give access to the component's _getPositionUpdate method. Normally this is a protected helper method but ConstraintProjectorDecorator is a friend of ConstraintProjector. */
    void _componentGetPositionUpdate(const unsigned position_index, const double* delC_ptr, const double inv_m, const double* dlam_ptr, double* pos_update_ptr) const
    {
        return _component->_getPositionUpdate(position_index, delC_ptr, inv_m, dlam_ptr, pos_update_ptr);
    }

    std::unique_ptr<ConstraintProjector> _component;        // the original ConstraintProjector whose behavior is being augmented
};


/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

/** Extends a ConstraintProjector to use the XPBD update formula with damping terms included (Equation (25) in the XPBD paper). 
 * Each constraint projected will have the same gamma (where gamma = alpha_tilde * beta_tilde / Delta t, beta_tilde being the damping stiffness), that is passed into the constructor.
*/
class WithDamping : public ConstraintProjectorDecorator
{ 
    public:
    /** The additional parameter gamma is the gamma for damping that appears in Equation (26) in the XPBD paper. */
    WithDamping(std::unique_ptr<ConstraintProjector> component, double gamma)
        : ConstraintProjectorDecorator(std::move(component)), _damping_gamma(gamma)
    {
    }

    virtual bool usesDamping() const override { return true; }

    protected:
    /** Override the _LHS method to include damping in the system matrix (i.e. as in the LHS of Equation (25) in the XPBD paper).
     * This involves each term in the already computed LHS matrix by a factor (1 + gamma).
     */
    inline virtual void _LHS(const double* delC_ptr, const double* M_inv_ptr, const double* alpha_tilde_ptr, double* lhs_ptr) override
    {
        // first, compute the LHS matrix using the wrapped component's method
        _componentLHS(delC_ptr, M_inv_ptr, alpha_tilde_ptr, lhs_ptr);

        // now, multiply each component in LHS by (1 + gamma)
        for (unsigned ci = 0; ci < numConstraints(); ci++)          // ci is row index (LHS matrix is column-major)
        {
            for (unsigned cj = 0; cj < numConstraints(); cj++)      // cj is column index
            {
                // if gammas are different for each constraint, this will be the damping gamma for constraint ci (gammas are the same across a row of LHS)
                const double gamma = _damping_gamma;    // right now, gamma is the same for all constraints
                if (ci == cj)
                {
                    // if on the diagonal, we need to subtract alpha_tilde here and add it back outside of the multiplication
                    lhs_ptr[cj*numConstraints() + ci] = (1 + gamma) * (lhs_ptr[cj*numConstraints() + ci] - alpha_tilde_ptr[ci]) + alpha_tilde_ptr[ci];
                }
                else
                {
                    lhs_ptr[cj*numConstraints() + ci] *= (1 + gamma);
                }
            }
        }
    }

    /** Override the _RHS method to include damping (i.e. as in the RHS of Equation (25) in the XPBD paper).
     * This involves subtracting an additional term that depends on gamma, delC, and the previous positions. 
     */
    inline virtual void _RHS(const double* C_ptr, const double* delC_ptr, const double* alpha_tilde_ptr, double* rhs_ptr) override
    {
        // first, compute the RHS vector using the wrapped component's method
        _componentRHS(C_ptr, delC_ptr, alpha_tilde_ptr, rhs_ptr);

        // now, compute the additional term to be subtracted - i.e. gamma * delC * (x - x_prev)
        for (unsigned ci = 0; ci < numConstraints(); ci++)
        {
            double delC_x_prev = 0;                                     // accumulate contributions from delC * (x - x_prev)
            const double* delC_i = delC_ptr + numCoordinates()*ci;      // pointer to constraint gradient of the ci'th constraint
            for (unsigned pi = 0; pi < numPositions(); pi++)
            {
                const double* pos = _positions[pi].position_ptr;            // current position
                const double* prev_pos = _positions[pi].prev_position_ptr;  // previous position
                delC_x_prev += delC_i[3*pi]*(pos[0] - prev_pos[0]) + delC_i[3*pi+1]*(pos[1] - prev_pos[1]) + delC_i[3*pi+2]*(pos[2] - prev_pos[2]);      // delC_i * (x_i - x_prev_i)
            }

            rhs_ptr[ci] -= _damping_gamma * delC_x_prev;    // subtract gamma * delC * (x - x_prev)
        }
    }

    private:
    double _damping_gamma;      // the amount of damping, same gamma as in Equation (26) of XPBD paper
};


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

/** Extends a ConstraintProjector to use an XPBD update formula that includes g, the primary residual, and "distributes" it by dividing by the number of constraint updates a position is involved in. 
 * By including the primary residual in the XPBD update, the primary residual is driven to 0 over multiple iterations (which is not true if you omit it in the update).
*/
class WithDistributedPrimaryResidual : public ConstraintProjectorDecorator
{
    
    public:
    /** Wrap the component */
    WithDistributedPrimaryResidual(std::unique_ptr<ConstraintProjector> component)
        : ConstraintProjectorDecorator(std::move(component))
    {
    }

    /** Sets the pointer to the primary residual vector. */
    void setPrimaryResidual(const double* res_ptr) { _res_ptr = res_ptr; }

    /** Indicates that this ConstraintProjector uses the primary residual in its update, which tells the XPBDSolver that it needs to compute the primary residual every iteration. */
    virtual bool usesPrimaryResidual() const override { return true; }

    protected:
    /** Override the _RHS method to include the distributed primary residual.
     * This involves adding an additional term (i.e. delC * M^-1 *  scaled_g).
     */
    inline virtual void _RHS(const double* C_ptr, const double* delC_ptr, const double* alpha_tilde_ptr, double* rhs_ptr) override
    {
        // first, compute the RHS vector using the wrapped component's method
        _componentRHS(C_ptr, delC_ptr, alpha_tilde_ptr, rhs_ptr);

        // then, add delC * M^-1 * scaled_g to the RHS
        for (unsigned ci = 0; ci < numConstraints(); ci++)
        {
            const double* delC_i = delC_ptr + numCoordinates()*ci;        // pointer to the delC vector of the ith constraint (1 x numCoordinates)
            for (unsigned pi = 0; pi < numPositions(); pi++)
            {
                const double inv_m = _positions[pi].inv_mass;               // inverse mass of pi'th position
                const unsigned index = _positions[pi].index;                // vertex index of pi'th position - used to index the primary residual vector
                const double g_scaling = static_cast<double>(_positions[pi].num_constraints);   // divisor for the primary residual for the pi'th position
                rhs_ptr[ci] += inv_m * (delC_ptr[3*pi]*_res_ptr[3*index]/g_scaling + delC_ptr[3*pi+1]*_res_ptr[3*index+1]/g_scaling + delC_ptr[3*pi+2]*_res_ptr[3*index+2]/g_scaling);
            }
        }
    }

    /** Override the _getPositionUpdate method to include the distributed primary residual.
     * This involves subtracting an additional term from the position update (i.e. M^-1 * scaled_g)
     */
    inline virtual void _getPositionUpdate(const unsigned position_index, const double* delC_ptr, const double inv_m, const double* dlam_ptr, double* pos_update_ptr) const override
    {
        // first, compute the position update using the wrapped component's method
        _componentGetPositionUpdate(position_index, delC_ptr, inv_m, dlam_ptr, pos_update_ptr);


        const unsigned index = _positions[position_index].index;    // vertex index of this position - used to index the primary residual vector
        const double g_scaling = static_cast<double>(_positions[position_index].num_constraints);   // divisor for the primary residual for this position

        // subtract M^-1 * scaled_g
        pos_update_ptr[0] -= inv_m * _res_ptr[3*index]/g_scaling;
        pos_update_ptr[1] -= inv_m * _res_ptr[3*index+1]/g_scaling;
        pos_update_ptr[2] -= inv_m * _res_ptr[3*index+2]/g_scaling;
    }

    private:

    /** Pointer to the primary residual vector.
     * Since the primary residual is something that is aggregated over all constraints, it is a quantity that is computed by the XPBDSolver.
     * The setPrimaryResidual() method is used to point this ConstraintProjector to where the primary residual vector is.
     * It is assumed that the primary residual is up to date when the constraint projection occurs.
     */
    const double* _res_ptr;
};


////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

/** Extends a ConstraintProjector to perform a first order XPBD update.
 * This involves using an alpha_tilde = alpha / dt and using the damping matrix B instead of M.
 */
class FirstOrder : public ConstraintProjectorDecorator
{
    public:
    /** Wrap the component */
    FirstOrder(std::unique_ptr<ConstraintProjector> component)
        : ConstraintProjectorDecorator(std::move(component))
    {
        for (unsigned i = 0; i < numPositions(); i++)
        {
            // try casting to FirstOrderXPBDMeshObject
            if (FirstOrderXPBDMeshObject* fo_obj = dynamic_cast<FirstOrderXPBDMeshObject*>(_positions[i].obj))
            {
                _positions[i].inv_mass = fo_obj->vertexInvDamping(_positions[i].index);     // instead of inverse mass, use inverse damping - still use the same "inv_mass" variable
            }
            else
            {
                // if cast was unsuccessful, throw an error
                std::cout << _positions[i].obj->name() << " of type " << _positions[i].obj->type() << " is not a FirstOrderXPBDMeshObject!" << std::endl;
                assert(0);
            }
        }
    }

    /** Override the alphaTilde to use alpha_tilde = alpha / dt instead of alpha_tilde = alpha / dt^2. */
    inline virtual void alphaTilde(double* alpha_tilde_ptr) const override
    {
        for (unsigned i = 0; i < numConstraints(); i++)
        {
            alpha_tilde_ptr[i] = _constraints[i]->alpha() / _dt;
        }
    }
};

} // namespace Solver


#endif // __CONSTRAINT_DECORATOR_HPP