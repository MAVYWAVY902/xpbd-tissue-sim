#ifndef __XPBD_JACOBI_SOLVER_HPP
#define __XPBD_JACOBI_SOLVER_HPP

#include "solver/xpbd_solver/XPBDSolver.hpp"

namespace Solver
{

/** Enforces constraints using the XPBD algorithm with Jacobi iterations, where the state is updated after all constraint projections.
 */
template <bool IsFirstOrder, typename ...ConstraintProjectors>
class XPBDJacobiSolver : public XPBDSolver<IsFirstOrder, ConstraintProjectors...>
{
    public:
    /** Same constructor as XPBDSolver */
    explicit XPBDJacobiSolver(Sim::XPBDMeshObject_Base* obj, int num_iter, XPBDSolverResidualPolicyEnum residual_policy)
        : XPBDSolver<IsFirstOrder, ConstraintProjectors...>(obj, num_iter, residual_policy)
    {}

    void setup()
    {
        
    }

    // TODO: figure out how to "override" XPBDSolver functionality for addConstraintProjector
    template<class... Constraints>
    void addConstraintProjector(Real dt, Constraints* ... constraints)
    {
        using ProjectorType = typename ConstraintProjectorTraits<IsFirstOrder, Constraints...>::type;
        ProjectorType projector(dt, constraints...);
    
        // make sure that the data buffer of the coordinate updates vector is large enough to accomodate the new projector
        this->_coordinate_updates.resize(this->_coordinate_updates.size() + projector.numCoordinates());
        this->_rigid_body_updates.resize(this->_rigid_body_updates.size() + ProjectorType::NUM_RIGID_BODIES);
    
        // increase the total number of constraints (needed for the constraint residual size)
        this->_num_constraints += ProjectorType::NUM_CONSTRAINTS;
            
        this->_constraint_projectors.template push_back<ProjectorType>(std::move(projector));
    }


    protected:
    /** Implements a Jacobi update strategy for constraint projection.
     */
    void _iterateConstraints()
    {
        int total_coord_updates = 0;
        int total_rb_updates = 0;
        this->_constraint_projectors.for_each_element([&](auto& proj)
        {
            using ProjectorType = std::remove_cv_t<std::remove_reference_t<decltype(proj)>>;
            if (!proj.isValid())
                return;

            // TODO: check if proj is a RigidBodyConstraintProjector
            // if constexpr ()

            _projectorProject(proj, this->_coordinate_updates.data() + total_coord_updates, this->_rigid_body_updates.data() + total_rb_updates);
            total_coord_updates += proj.numCoordinates();
            total_rb_updates += ProjectorType::NUM_RIGID_BODIES;
        });

        // apply the position updates
        for (int i = 0; i < total_coord_updates; i++)
        {
            if (this->_coordinate_updates[i].ptr)
                *(this->_coordinate_updates[i].ptr) += this->_coordinate_updates[i].update;
        }

        // apply the rigid body updates
        for (int i = 0; i < total_rb_updates; i++)
        {
            const RigidBodyUpdate& rb_update = this->_rigid_body_updates[i];
            if (rb_update.obj_ptr)
            {
                rb_update.obj_ptr->setPosition(rb_update.obj_ptr->position() + Eigen::Map<const Vec3r>(rb_update.position_update));
                rb_update.obj_ptr->setOrientation(rb_update.obj_ptr->orientation() + Eigen::Map<const Vec4r>(rb_update.orientation_update));
            }
        }
    }

    template<class ProjectorType>
    void _projectorProject(ProjectorType& projector, CoordinateUpdate* coordinate_updates, RigidBodyUpdate*)
    {
        projector.project(coordinate_updates);
    }

    template<class ...Constraints>
    void _projectorProject(RigidBodyConstraintProjector<IsFirstOrder, Constraints...>& projector, CoordinateUpdate* coordinate_updates, RigidBodyUpdate* rigid_body_updates)
    {
        projector.project(coordinate_updates, rigid_body_updates);
    }
};

} // namespace Solver


#endif // __XPBD_JACOBI_SOLVER_HPP