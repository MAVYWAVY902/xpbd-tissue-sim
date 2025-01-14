#ifndef __RIGID_DEFORMABLE_COLLISION_CONSTRAINT_PROJECTOR_HPP
#define __RIGID_DEFORMABLE_COLLISION_CONSTRAINT_PROJECTOR_HPP

#include "solver/ConstraintProjector.hpp"
#include "solver/RigidDeformableCollisionConstraint.hpp"
#include "utils/GeometryUtils.hpp"

namespace Solver
{

class RigidDeformableCollisionConstraintProjector : public ConstraintProjector
{

    public:
    explicit RigidDeformableCollisionConstraintProjector(RigidDeformableCollisionConstraint* constraint, const double dt)
        : ConstraintProjector(std::vector<Constraint*>({constraint}), dt), rdc_constraint(constraint)
    {
    }

    inline void project(double* data_ptr, double* coordinate_updates_ptr, Eigen::Vector<double, 7>& rigid_body_update_ptr)
    {
        ConstraintProjector::project(data_ptr, coordinate_updates_ptr);

        if (_C_ptr()[0] > 0)
        {
            rigid_body_update_ptr = Eigen::Vector<double,7>::Zero();
            return;
        }

        const Eigen::Vector3d n = -rdc_constraint->collisionNormal();
        const Eigen::Vector3d r = rdc_constraint->pointOnRigidBody() - rdc_constraint->rigidObj()->position();
        const Eigen::Matrix3d inv_I = rdc_constraint->rigidObj()->invI();
        const double m = rdc_constraint->rigidObj()->mass();

        const Eigen::Vector3d impulse = _dlam_ptr()[0] * n;
        const Eigen::Vector3d position_update = impulse / m;

        const Eigen::Vector3d I_r_cross_p = 0.5 * inv_I * (r.cross(impulse));
        const Eigen::Vector4d orientation_update = GeometryUtils::quatMult(Eigen::Vector4d(I_r_cross_p[0], I_r_cross_p[1], I_r_cross_p[2], 0), rdc_constraint->rigidObj()->orientation());
    
        rigid_body_update_ptr(Eigen::seq(0,2)) = position_update;
        rigid_body_update_ptr(Eigen::seq(3,6)) = orientation_update;
    }

    Sim::RigidObject* rigidObject() const { return rdc_constraint->rigidObj(); }

    protected:
    inline virtual void _LHS(const double* delC_ptr, const double* M_inv_ptr, const double* alpha_tilde_ptr, double* lhs_ptr) override
    {
        lhs_ptr[0] = alpha_tilde_ptr[0];
        for (int pi = 0; pi < numPositions(); pi++)    
        {
            const double inv_m = M_inv_ptr[pi];
            // add the contribution for each position (inv_m times the dot product of the constraint gradient vectors)
            lhs_ptr[0] += inv_m * (delC_ptr[3*pi]*delC_ptr[3*pi] + delC_ptr[3*pi+1]*delC_ptr[3*pi+1] + delC_ptr[3*pi+2]*delC_ptr[3*pi+2]);
        }

        // add contribution from rigid body
        // 1/m1 + (r1 x n)^T I1^-1 (r1 x n)
        const Eigen::Vector3d n = -rdc_constraint->collisionNormal();
        const Eigen::Vector3d r = rdc_constraint->pointOnRigidBody() - rdc_constraint->rigidObj()->position();
        const Eigen::Matrix3d inv_I = rdc_constraint->rigidObj()->invI();
        const double m = rdc_constraint->rigidObj()->mass();
        const Eigen::Vector3d r_cross_n = r.cross(n);
        lhs_ptr[0] += 1.0/m + r_cross_n.transpose() * inv_I * r_cross_n;
    }

    protected:
    const RigidDeformableCollisionConstraint* rdc_constraint;

};

} // namespace Solver

#endif // __RIGID_DEFORMABLE_COLLISION_CONSTRAINT_PROJECTOR_HPP