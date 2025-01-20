#ifndef __RIGID_BODY_CONSTRAINT_PROJECTOR_HPP
#define __RIGID_BODY_CONSTRAINT_PROJECTOR_HPP

#include "solver/ConstraintProjector.hpp"
#include "solver/RigidBodyConstraint.hpp"
#include "utils/GeometryUtils.hpp"

namespace Solver
{

/** A specialized XPBD constraint projector that knows specifically how to deal with constraints involving rigid bodies.
 * 
 * Most parts of the constraint projection are the same, except:
 *  - only one constraint can be projected at a time (no simultaneous solves)
 *  - the LHS computation (i.e. the denominator in the lambda update expression) involves rigid body inertial weights
 *  - the position update now involves a specialized position+orientation update for rigid bodies
 */
class RigidBodyConstraintProjector : public ConstraintProjector
{

    public:
    explicit RigidBodyConstraintProjector(Constraint* constraint, const double dt)
        : ConstraintProjector(std::vector<Constraint*>({constraint}), dt), rb_constraint(dynamic_cast<RigidBodyConstraint*>(constraint))
    {
        // make sure the constraint that was passed into the constructor is a RigidBodyConstraint
        assert(rb_constraint);
    }

    /** Specialized project method that takes an additional output argument for rigid body updates.
     * Calls the ConstraintProjector::project method and then computes updates for the rigid bodies.
     * 
     * @param data_ptr - a pointer to a block of pre-allocated data, assumed to be at least as large as that given by memoryNeeded(). Used to store results from computations, but not necessarily to be used as an output parameter.
     * @param coordinate_updates_ptr (OUTPUT) - a pointer to an array of "coordinate updates" with structure [Delta x1, Delta y1, Delta z1, Delta x2, Delta y2, Delta z2, etc.). Assumed to be at least numCoordintes() x 1.
     * @param rigid_body_updates_ptr (OUTPUT) - a pointer to an array of rigid body updates with structure [Delta position 1, Delta orientation 1, Delta position 2, Delta orientation 2, etc.]. Assumed to be at least 7 x numRigidBodies().
     */
    inline void project(double* data_ptr, double* coordinate_updates_ptr, double* rigid_body_updates_ptr)
    {
        // project the constraint as normal
        ConstraintProjector::project(data_ptr, coordinate_updates_ptr);

        // if the constraint is an inequality constraint and C(x) > 0, do not enforce the constraint
        if (_C_ptr()[0] > 0 && _state->_constraints[0]->isInequality())
        {
            for (int i = 0; i < numRigidBodies()*7; i++)
            {
                rigid_body_updates_ptr[i] = 0;
            }
            return;
        }

        const double dlam = _dlam_ptr()[0];
        // compute the update for each rigid body using the corresponding RigidBodyXPBDHelper object
        for (int ri = 0; ri < numRigidBodies(); ri++)
        {
            rb_constraint->rigidBodyHelpers()[ri]->update(dlam, rigid_body_updates_ptr + 7*ri);
        }
    }

    std::vector<Sim::RigidObject*> rigidBodies() const { return rb_constraint->rigidBodies(); }
    int numRigidBodies() const { return rb_constraint->numRigidBodies(); }

    protected:
    /** Specialized _LHS method that includes the weights of the rigid bodies in the denominator.
     * @param delC_ptr - the pointer to the delC matrix. Expects it to be row-major and numConstraints x numCoordinates.
     * @param M_inv_ptr - the pointer to the M^-1 "matrix". Expects it to be a vector that is numPositions x 1.
     * @param alpha_tilde_ptr - the pointer to the alpha_tilde "matrix". Expects it to be a vector and numConstraints x 1.
     * @param lhs_ptr (OUTPUT) - the pointer to the (currently empty) LHS matrix. Expects it to be column-major and numConstraints x numConstraints.
     */
    inline virtual void _LHS(const double* delC_ptr, const double* M_inv_ptr, const double* alpha_tilde_ptr, double* lhs_ptr) override
    {
        lhs_ptr[0] = alpha_tilde_ptr[0];
        for (int pi = 0; pi < numPositions(); pi++)    
        {
            const double inv_m = M_inv_ptr[pi];
            // add the contribution for each position (inv_m times the dot product of the constraint gradient vectors)
            lhs_ptr[0] += inv_m * (delC_ptr[3*pi]*delC_ptr[3*pi] + delC_ptr[3*pi+1]*delC_ptr[3*pi+1] + delC_ptr[3*pi+2]*delC_ptr[3*pi+2]);
        }

        // add contribution from rigid body - computed with the RigidBodyXBPDHelper class
        for (int ri = 0; ri < numRigidBodies(); ri++)
        {
            lhs_ptr[0] += rb_constraint->rigidBodyHelpers()[ri]->weight();
        }
    }

    protected:
    const RigidBodyConstraint* rb_constraint;   // store the constraint as a RigidBodyConstraint for convenience

};

} // namespace Solver

#endif // __RIGID_DEFORMABLE_COLLISION_CONSTRAINT_PROJECTOR_HPP