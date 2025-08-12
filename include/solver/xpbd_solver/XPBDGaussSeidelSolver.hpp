#ifndef __XPBD_GAUSS_SEIDEL_SOLVER_HPP
#define __XPBD_GAUSS_SEIDEL_SOLVER_HPP

#include "solver/xpbd_solver/XPBDSolver.hpp"

namespace Solver
{

/** Enforces constraints using the XPBD algorithm with a Gauss-Seidel update strategy, where the state is updated after every constraint projection.
 * This is the algorithm/update strategy proposed by the original XPBD paper.
 */
template <bool IsFirstOrder, typename ...ConstraintProjectors>
class XPBDGaussSeidelSolver : public XPBDSolver<IsFirstOrder, ConstraintProjectors...>
{
    public:
    /** Same constructor as XPBDSolver */
    explicit XPBDGaussSeidelSolver(Sim::XPBDMeshObject_Base_<IsFirstOrder>* obj, int num_iter, XPBDSolverResidualPolicyEnum residual_policy)
        : XPBDSolver<IsFirstOrder, ConstraintProjectors...>(obj, num_iter, residual_policy)
    {
    }

    protected:
    /** Implements a Gauss-Seidel update strategy for constraint projection.
     * @param data - the pre-allocated data block to use for evaluating the constraints and their gradients. Assumes that it is large enough to accomodate the ConstraintProjector with the largest memory requirement.
     */
    virtual void _iterateConstraints() override
    {

        this->_constraint_projectors.for_each_element([&](auto& proj)
        {
            if (!proj.isValid())
                return;

            _projectAndUpdate(proj);
        });
    }


    template<class ProjectorType>
    void _projectAndUpdate(ProjectorType& projector)
    {
        projector.project(this->_coordinate_updates.data());
        _applyPositionUpdates(projector);
    }

    template<class ...Constraints>
    void _projectAndUpdate(RigidBodyConstraintProjector<IsFirstOrder, Constraints...>& projector)
    {
        projector.project(this->_coordinate_updates.data(), this->_rigid_body_updates.data());
        _applyPositionUpdates(projector);
        _applyRigidBodyUpdates(projector);
    }

    template<class ProjectorType>
    void _applyPositionUpdates(ProjectorType& projector)
    {
        // apply the position updates
        for (int i = 0; i < projector.numCoordinates(); i++)
        {
            if (this->_coordinate_updates[i].ptr)
                *(this->_coordinate_updates[i].ptr) += this->_coordinate_updates[i].update;
        }
    }

    template<class ...Constraints>
    void _applyRigidBodyUpdates(RigidBodyConstraintProjector<IsFirstOrder, Constraints...>&)
    {
        using ProjectorType = RigidBodyConstraintProjector<IsFirstOrder, Constraints...>;
        // apply the rigid body updates
        for (unsigned i = 0; i < ProjectorType::NUM_RIGID_BODIES; i++)
        {
            const RigidBodyUpdate& rb_update = this->_rigid_body_updates[i];
            if (rb_update.obj_ptr)
            {
                rb_update.obj_ptr->setPosition(rb_update.obj_ptr->position() + Eigen::Map<const Vec3r>(rb_update.position_update));
                rb_update.obj_ptr->setOrientation(rb_update.obj_ptr->orientation() + Eigen::Map<const Vec4r>(rb_update.orientation_update));
            }
        }
    }
};

} // namespace Solver


#endif // __XPBD_GAUSS_SEIDEL_SOLVER_HPP