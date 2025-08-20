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
    using projector_reference_container_type = typename XPBDSolver<IsFirstOrder, ConstraintProjectors...>::projector_reference_container_type;

    public:
    /** Same constructor as XPBDSolver */
    explicit XPBDJacobiSolver(Sim::XPBDMeshObject_Base_<IsFirstOrder>* obj, int num_iter, XPBDSolverResidualPolicyEnum residual_policy)
        : XPBDSolver<IsFirstOrder, ConstraintProjectors...>(obj, num_iter, residual_policy)
    {}

    void setup()
    {
        
    }

    // TODO: figure out how to "override" XPBDSolver functionality for addConstraintProjector
    template<class... Constraints>
    ConstraintProjectorReference<typename ConstraintProjectorTraits<IsFirstOrder, Constraints...>::type> 
    addConstraintProjector(Real dt, ConstraintReference<Constraints>&&... constraints)
    {
        using ProjectorType = typename ConstraintProjectorTraits<IsFirstOrder, Constraints...>::type;
        ProjectorType projector(dt, std::forward<ConstraintReference<Constraints>>(constraints)...);
    
        // make sure that the data buffer of the coordinate updates vector is large enough to accomodate the new projector
        this->_coordinate_updates.resize(this->_coordinate_updates.size() + projector.numCoordinates());
        this->_rigid_body_updates.resize(this->_rigid_body_updates.size() + ProjectorType::NUM_RIGID_BODIES);
    
        // increase the total number of constraints (needed for the constraint residual size)
        this->_num_constraints += ProjectorType::NUM_CONSTRAINTS;
            
        this->_constraint_projectors.template push_back<ProjectorType>(std::move(projector));

        std::vector<ProjectorType>& proj_vec = this->_constraint_projectors.template get<ProjectorType>();
        return ConstraintProjectorReference<ProjectorType>(proj_vec, proj_vec.size()-1);
    }

    /** Creates a projector for the passed in constraints and adds it to the associated projector array.
     * @param dt - the time step used by the constraint projector (i.e. the simulation time step)
     * @param B_e_inv - inverse damping matrix for the element
     * @param constraints - variadic list of constraint pointers (right now, only 1 or 2 constraints are supported)
     * @returns a reference to the added projector as a ConstraintProjectorReference
     */
    template<class... Constraints>
    ConstraintProjectorReference<typename ConstraintProjectorTraits<IsFirstOrder, Constraints...>::type> 
    addConstraintProjector(Real dt, const Eigen::Matrix<Real,12,12>& B_e_inv, ConstraintReference<Constraints>&&... constraints)
    {
        using ProjectorType = typename ConstraintProjectorTraits<IsFirstOrder, Constraints...>::type;
        ProjectorType projector(dt, std::forward<ConstraintReference<Constraints>>(constraints)..., B_e_inv);
    
        // make sure that the data buffer of the coordinate updates vector is large enough to accomodate the new projector
        this->_coordinate_updates.resize(this->_coordinate_updates.size() + projector.numCoordinates());
        this->_rigid_body_updates.resize(this->_rigid_body_updates.size() + ProjectorType::NUM_RIGID_BODIES);
    
        // increase the total number of constraints (needed for the constraint residual size)
        this->_num_constraints += ProjectorType::NUM_CONSTRAINTS;
            
        this->_constraint_projectors.template push_back<ProjectorType>(std::move(projector));

        std::vector<ProjectorType>& proj_vec = this->_constraint_projectors.template get<ProjectorType>();
        return ConstraintProjectorReference<ProjectorType>(proj_vec, proj_vec.size()-1);
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

    /** Helper function that will perform 1 iteration of the solver for the specified list of projectors.
     * This method is pure virtual because its implementation depends on the solver type (Gauss-Seidel, Jacobi, etc.) to know what to do with the position updates given by the ConstraintProjectors.
     */
    virtual void _iterateConstraints(projector_reference_container_type& projector_references) override
    {
        int total_coord_updates = 0;
        int total_rb_updates = 0;
        projector_references.for_each_element([&](auto& proj_ref)
        {
            using ProjectorType = typename std::remove_cv_t<std::remove_reference_t<decltype(proj_ref)>>::constraint_projector_type;
            if (!proj_ref->isValid())
                return;

            // TODO: check if proj is a RigidBodyConstraintProjector
            // if constexpr ()

            _projectorProject(*proj_ref, this->_coordinate_updates.data() + total_coord_updates, this->_rigid_body_updates.data() + total_rb_updates);
            total_coord_updates += proj_ref->numCoordinates();
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