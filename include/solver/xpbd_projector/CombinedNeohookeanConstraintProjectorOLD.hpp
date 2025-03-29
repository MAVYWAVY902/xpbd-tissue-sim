#ifndef __COMBINED_NEOHOOKEAN_CONSTRAINT_PROJECTOR
#define __COMBINED_NEOHOOKEAN_CONSTRAINT_PROJECTOR

#include "solver/ConstraintProjector.hpp"
#include "solver/HydrostaticConstraint.hpp"
#include "solver/DeviatoricConstraint.hpp"

namespace Solver
{

/** Implements a ConstraintProjector that is specifically designed to efficiently project a Hydrostatic and Deviatoric constraint together.
 * Assuming the Hydrostatic and Deviatoric constraints are applied to the same mesh element, the deformation gradient F is the same and does not need to be recomputed.
 */
class CombinedNeohookeanConstraintProjector : public ConstraintProjector
{
    public:
    CombinedNeohookeanConstraintProjector(std::vector<Constraint*> constraints, const Real dt)
        : ConstraintProjector(constraints, dt)
    {
        assert(numConstraints() == 2);      // there should only be 2 constraints
        assert(numPositions() == 4);        // the 2 constraints should share all 4 positions

        // the first constraint should be a DeviatoricConstraint and the second should be a HydrostaticConstraint
        // the order of the constraints matters for simplicity (and speed) of implementation
        if ((_dev_constraint = dynamic_cast<DeviatoricConstraint*>(_state->_constraints[0])) && (_hyd_constraint = dynamic_cast<HydrostaticConstraint*>(_state->_constraints[1])))
        {
            // all good
        }
        else
        {
            std::cout << "ERROR: Constraints vector must have 1 Deviatoric constraint and 1 Hydrostatic constraint (in that particular order)!" << std::endl;
            assert(0);
        }
    }

    protected:

    /** Override the evaluation of the constraints and their gradients.
     * The deviatoric and hydrostatic constraints both involve the same F (when projected simultaneously) in their evaluation of the constraint and gradient.
     * Since F does not change, we should only evaluate it once to save time.
     * 
     * @param C_ptr - the pointer to the constraint vector. Expects it to be numConstraints x 1.
     * @param delC_ptr - the pointer to the delC matrix. Expects it to be row-major and numConstraints x numCoordinates.
     * @param C_mem_ptr - the pointer to additional memory for the constraints to store intermediate calculations 
     */
    inline virtual void _evaluateConstraintsAndGradients(Real* C_ptr, Real* delC_ptr) override
    {
        Real F[9];
        Real X[9];
        _dev_constraint->_computeF(F, X);
        _dev_constraint->_evaluate(C_ptr, F);
        _dev_constraint->_gradient(delC_ptr, C_ptr, F);

        _hyd_constraint->_evaluate(C_ptr+1, F);
        _hyd_constraint->_gradient(delC_ptr+numCoordinates(), F);        
    }

    private:
    DeviatoricConstraint* _dev_constraint;      // the deviatoric constraint - store it under its specific type for convenience
    HydrostaticConstraint* _hyd_constraint;     // the hydrostatic constraint - store it under its specific type for convenience
};

}

#endif // __COMBINED_NEOHOOKEAN_CONSTRAINT_PROJECTOR