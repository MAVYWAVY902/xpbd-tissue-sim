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

    inline virtual void initialize() override { _component->initialize(); }

    inline virtual void alphaTilde(Real* alpha_tilde_ptr) const override { _component->alphaTilde(alpha_tilde_ptr); }

    inline virtual bool usesPrimaryResidual() const override { return _component->usesPrimaryResidual(); }

    inline virtual bool usesDamping() const override { return _component->usesDamping(); }

    protected:
    inline virtual void _evaluateConstraintsAndGradients(Real* C_ptr, Real* delC_ptr) override { _component->_evaluateConstraintsAndGradients(C_ptr, delC_ptr); }

    inline virtual void _LHS(const Real* delC_ptr, const Real* M_inv_ptr, const Real* alpha_tilde_ptr, Real* lhs_ptr) override { _component->_LHS(delC_ptr, M_inv_ptr, alpha_tilde_ptr, lhs_ptr); }

    inline virtual void _RHS(const Real* C_ptr, const Real* delC_ptr, const Real* alpha_tilde_ptr, Real* rhs_ptr) override { _component->_RHS(C_ptr, delC_ptr, alpha_tilde_ptr, rhs_ptr); }

    inline virtual void _getPositionUpdate(const int position_index, const Real* delC_ptr, const Real inv_m, const Real* dlam_ptr, Real* pos_update_ptr) const override
    {
        _component->_getPositionUpdate(position_index, delC_ptr, inv_m, dlam_ptr, pos_update_ptr);
    }


    protected:
    /** Give access to the component's _LHS() method. Normally this is a protected helper method but ConstraintProjectorDecorator is a friend of ConstraintProjector. */
    void _componentLHS(const Real* delC_ptr, const Real* M_inv_ptr, const Real* alpha_tilde_ptr, Real* lhs_ptr) const
    {
        return _component->_LHS(delC_ptr, M_inv_ptr, alpha_tilde_ptr, lhs_ptr);
    }

    /** Give access to the component's _RHS() method. Normally this is a protected helper method but ConstraintProjectorDecorator is a friend of ConstraintProjector. */
    void _componentRHS(const Real* C_ptr, const Real* delC_ptr, const Real* alpha_tilde_ptr, Real* rhs_ptr) const
    {
        return _component->_RHS(C_ptr, delC_ptr, alpha_tilde_ptr, rhs_ptr);
    }

    /** Give access to the component's _getPositionUpdate method. Normally this is a protected helper method but ConstraintProjectorDecorator is a friend of ConstraintProjector. */
    void _componentGetPositionUpdate(const int position_index, const Real* delC_ptr, const Real inv_m, const Real* dlam_ptr, Real* pos_update_ptr) const
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
    WithDamping(std::unique_ptr<ConstraintProjector> component, Real gamma)
        : ConstraintProjectorDecorator(std::move(component)), _damping_gamma(gamma)
    {
    }

    virtual bool usesDamping() const override { return true; }

    protected:
    /** Override the _LHS method to include damping in the system matrix (i.e. as in the LHS of Equation (25) in the XPBD paper).
     * This involves each term in the already computed LHS matrix by a factor (1 + gamma).
     */
    inline virtual void _LHS(const Real* delC_ptr, const Real* M_inv_ptr, const Real* alpha_tilde_ptr, Real* lhs_ptr) override
    {
        // first, compute the LHS matrix using the wrapped component's method
        _componentLHS(delC_ptr, M_inv_ptr, alpha_tilde_ptr, lhs_ptr);

        // now, multiply each component in LHS by (1 + gamma)
        for (int ci = 0; ci < numConstraints(); ci++)          // ci is row index (LHS matrix is column-major)
        {
            for (int cj = 0; cj < numConstraints(); cj++)      // cj is column index
            {
                // if gammas are different for each constraint, this will be the damping gamma for constraint ci (gammas are the same across a row of LHS)
                const Real gamma = _damping_gamma;    // right now, gamma is the same for all constraints
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
    inline virtual void _RHS(const Real* C_ptr, const Real* delC_ptr, const Real* alpha_tilde_ptr, Real* rhs_ptr) override
    {
        // first, compute the RHS vector using the wrapped component's method
        _componentRHS(C_ptr, delC_ptr, alpha_tilde_ptr, rhs_ptr);

        // now, compute the additional term to be subtracted - i.e. gamma * delC * (x - x_prev)
        for (int ci = 0; ci < numConstraints(); ci++)
        {
            Real delC_x_prev = 0;                                     // accumulate contributions from delC * (x - x_prev)
            const Real* delC_i = delC_ptr + numCoordinates()*ci;      // pointer to constraint gradient of the ci'th constraint
            for (int pi = 0; pi < numPositions(); pi++)
            {
                const Real* pos = _state->_positions[pi].position_ptr;            // current position
                const Real* prev_pos = _state->_positions[pi].prev_position_ptr;  // previous position
                delC_x_prev += delC_i[3*pi]*(pos[0] - prev_pos[0]) + delC_i[3*pi+1]*(pos[1] - prev_pos[1]) + delC_i[3*pi+2]*(pos[2] - prev_pos[2]);      // delC_i * (x_i - x_prev_i)
            }

            rhs_ptr[ci] -= _damping_gamma * delC_x_prev;    // subtract gamma * delC * (x - x_prev)
        }
    }

    private:
    Real _damping_gamma;      // the amount of damping, same gamma as in Equation (26) of XPBD paper
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
    void setPrimaryResidual(const Real* res_ptr) { _res_ptr = res_ptr; }

    /** Indicates that this ConstraintProjector uses the primary residual in its update, which tells the XPBDSolver that it needs to compute the primary residual every iteration. */
    virtual bool usesPrimaryResidual() const override { return true; }

    protected:
    /** Override the _RHS method to include the distributed primary residual.
     * This involves adding an additional term (i.e. delC * M^-1 *  scaled_g).
     */
    inline virtual void _RHS(const Real* C_ptr, const Real* delC_ptr, const Real* alpha_tilde_ptr, Real* rhs_ptr) override
    {
        // first, compute the RHS vector using the wrapped component's method
        _componentRHS(C_ptr, delC_ptr, alpha_tilde_ptr, rhs_ptr);

        // then, add delC * M^-1 * scaled_g to the RHS
        for (int ci = 0; ci < numConstraints(); ci++)
        {
            for (int pi = 0; pi < numPositions(); pi++)
            {
                const Real inv_m = _state->_positions[pi].inv_mass;               // inverse mass of pi'th position
                const int index = _state->_positions[pi].index;                // vertex index of pi'th position - used to index the primary residual vector
                const Real g_scaling = static_cast<Real>(_state->_positions[pi].num_constraints);   // divisor for the primary residual for the pi'th position
                rhs_ptr[ci] += inv_m * (delC_ptr[3*pi]*_res_ptr[3*index]/g_scaling + delC_ptr[3*pi+1]*_res_ptr[3*index+1]/g_scaling + delC_ptr[3*pi+2]*_res_ptr[3*index+2]/g_scaling);
            }
        }
    }

    /** Override the _getPositionUpdate method to include the distributed primary residual.
     * This involves subtracting an additional term from the position update (i.e. M^-1 * scaled_g)
     */
    inline virtual void _getPositionUpdate(const int position_index, const Real* delC_ptr, const Real inv_m, const Real* dlam_ptr, Real* pos_update_ptr) const override
    {
        // first, compute the position update using the wrapped component's method
        _componentGetPositionUpdate(position_index, delC_ptr, inv_m, dlam_ptr, pos_update_ptr);


        const int index = _state->_positions[position_index].index;    // vertex index of this position - used to index the primary residual vector
        const Real g_scaling = static_cast<Real>(_state->_positions[position_index].num_constraints);   // divisor for the primary residual for this position

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
    const Real* _res_ptr;
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
        for (int i = 0; i < numPositions(); i++)
        {
            // try casting to FirstOrderXPBDMeshObject
            if (const Sim::FirstOrderXPBDMeshObject* fo_obj = dynamic_cast<const Sim::FirstOrderXPBDMeshObject*>(_state->_positions[i].obj))
            {
                _state->_positions[i].inv_mass = fo_obj->vertexInvDamping(_state->_positions[i].index);     // instead of inverse mass, use inverse damping - still use the same "inv_mass" variable
            }
            else
            {
                // if cast was unsuccessful, throw an error
                std::cout << _state->_positions[i].obj->name() << " of type " << _state->_positions[i].obj->type() << " is not a FirstOrderXPBDMeshObject!" << std::endl;
                assert(0);
            }
        }
    }

    /** Override the alphaTilde to use alpha_tilde = alpha / dt instead of alpha_tilde = alpha / dt^2. */
    inline virtual void alphaTilde(Real* alpha_tilde_ptr) const override
    {
        for (int i = 0; i < numConstraints(); i++)
        {
            alpha_tilde_ptr[i] = _state->_constraints[i]->alpha() / _state->_dt;
        }
    }
};

} // namespace Solver


#endif // __CONSTRAINT_DECORATOR_HPP