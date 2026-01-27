#include "solver/constraint/RigidDeformableCollisionConstraint.hpp"

namespace Solver
{
RigidDeformableCollisionConstraint::RigidDeformableCollisionConstraint(const Geometry::SDF* sdf, Sim::RigidObject* rigid_obj, const Vec3r& rigid_body_point, const Vec3r& collision_normal,
                                    int v1, Real* p1, Real* prev_p1, Real m1,
                                    int v2, Real* p2, Real* prev_p2, Real m2,
                                    int v3, Real* p3, Real* prev_p3, Real m3,
                                    Real u, Real v, Real w)
: CollisionConstraint(std::vector<PositionReference>({
    PositionReference(v1, p1, m1),
    PositionReference(v2, p2, m2),
    PositionReference(v3, p3, m3)}), collision_normal),
    RigidBodyConstraint(std::vector<Sim::RigidObject*>({rigid_obj})),
    _sdf(sdf), _point_on_rigid_body(rigid_obj->globalToBody(rigid_body_point)), _u(u), _v(v), _w(w),
    _prev_p1(prev_p1), _prev_p2(prev_p2), _prev_p3(prev_p3)
{
    // Set compliance for collision constraint to prevent infinite stiffness
    // Small compliance value (1e-8 to 1e-6) provides regularization and prevents severe penetration
    // Larger values = softer collision response, smaller = harder (but more stable than alpha=0)
    _alpha = 1e-8;  // Compliance parameter for collision constraint
    
    // create the Helper class that will evaluate the rigid body "weight" and the rigid body update when this constraint is projected
    // it is created here because the Helper needs info from the collision itself, such as the normal and the point on the rigid body
    std::unique_ptr<RigidBodyXPBDHelper> helper = std::make_unique<PositionalRigidBodyXPBDHelper>(rigid_obj, -collision_normal, rigid_body_point);
    _rigid_body_helpers.push_back(std::move(helper));
}

void RigidDeformableCollisionConstraint::evaluate(Real* C) const
{
    // get the point on the deformable body from the barycentric coordinates
    const Vec3r a = _u*Eigen::Map<Vec3r>(_positions[0].position_ptr) + _v*Eigen::Map<Vec3r>(_positions[1].position_ptr) + _w*Eigen::Map<Vec3r>(_positions[2].position_ptr);
    
    // constraint value is the penetration distance, which we can get from the rigid body SDF
    *C = _sdf->evaluate(a);
    
    // ✅ BUG FIX #8: Update collision normal dynamically from current SDF gradient!
    // Just like adhesion Bug #1, the collision normal can change as the rigid body rotates/moves.
    // The helper uses this direction to compute rigid body position/orientation updates.
    // If we use a stale normal, the rigid body gets pushed in the wrong direction → jiggling!
    if (*C <= 0 && !_rigid_body_helpers.empty()) {  // Only update when penetrating
        // Get current collision normal from SDF gradient at current position
        Vec3r current_collision_normal = _sdf->gradient(a);
        
        // Update the rigid body helper with current normal
        // Note: We pass -current_collision_normal because we want to push rigid body
        // in the opposite direction of the SDF gradient (away from deformable)
        auto* positional_helper = dynamic_cast<PositionalRigidBodyXPBDHelper*>(_rigid_body_helpers[0].get());
        if (positional_helper) {
            const Sim::RigidObject* rigid_obj = _rigid_bodies[0];
            const Vec3r rigid_point_global = rigid_obj->bodyToGlobal(_point_on_rigid_body);
            *positional_helper = PositionalRigidBodyXPBDHelper(rigid_obj, -current_collision_normal, rigid_point_global);
        }
        
        // Also update stored collision normal for gradient computation
        _collision_normal = current_collision_normal;
    }
}

void RigidDeformableCollisionConstraint::gradient(Real* delC) const
{
    // gradient w.r.t each deformable position involved is simply just the collision normal multiplied by the corresponding barycentric coordinate
    // i.e. C = (u*v1 + v*v2 + w*v3 - p1) * n ==> dC/dv1 = u*n, dC/dv2 = v*n, dC/dv3 = w*n
    delC[0] = _u*_collision_normal[0];
    delC[1] = _u*_collision_normal[1];
    delC[2] = _u*_collision_normal[2];

    delC[3] = _v*_collision_normal[0];
    delC[4] = _v*_collision_normal[1];
    delC[5] = _v*_collision_normal[2];
    
    delC[6] = _w*_collision_normal[0];
    delC[7] = _w*_collision_normal[1];
    delC[8] = _w*_collision_normal[2];
}


void RigidDeformableCollisionConstraint::evaluateWithGradient(Real* C, Real* grad) const
{
    evaluate(C);
    gradient(grad);
}

#ifdef HAVE_CUDA
RigidDeformableCollisionConstraint::GPUConstraintType RigidDeformableCollisionConstraint::createGPUConstraint() const
{
    GPUConstraintType gpu_constraint = GPUConstraintType(_positions[0].index, _positions[0].inv_mass,
                                                            _positions[1].index, _positions[1].inv_mass,
                                                            _positions[2].index, _positions[2].inv_mass,
                                                            _u, _v, _w,
                                                            _point_on_rigid_body, _collision_normal);
    return gpu_constraint;
}
#endif


// Enable Friction
void RigidDeformableCollisionConstraint::applyFriction(Real lam, Real mu_s, Real mu_k) const 
{
    // since we are colliding with a rigid body, we need to apply frictional forces to both the deformable body and the rigid body

    // get Eigen vectors of positions and previous positions - just easier to work with
    const Vec3r p1 = Eigen::Map<Vec3r>(_positions[0].position_ptr);
    const Vec3r p2 = Eigen::Map<Vec3r>(_positions[1].position_ptr);
    const Vec3r p3 = Eigen::Map<Vec3r>(_positions[2].position_ptr);

    const Vec3r p1_prev = Eigen::Map<Vec3r>(_prev_p1);
    const Vec3r p2_prev = Eigen::Map<Vec3r>(_prev_p2);
    const Vec3r p3_prev = Eigen::Map<Vec3r>(_prev_p3);

    const Vec3r p_cur = _u*p1 + _v*p2 + _w*p3;    // current colliding point on the deformable body
    const Vec3r p_prev = _u*p1_prev + _v*p2_prev + _w*p3_prev;    // previous colliding point on the deformable body
    // get the movement of the colliding point in the directions tangent to the collision normal
    // this is directly related to the amount of tangential force felt by the colliding point
    const Vec3r dp = p_cur - p_prev;

    const Sim::RigidObject* obj = _rigid_bodies[0];
    const Vec3r rigid_p_cur = obj->bodyToGlobal(_point_on_rigid_body);    // current colliding point on the rigid body (global coords)
    const Vec3r rigid_p_prev = obj->prevPosition() + GeometryUtils::rotateVectorByQuat(_point_on_rigid_body, obj->prevOrientation()); // previous colliding point on the rigid body (global coords)
    // get the movement of the colliding point in the directions tangent to the collision normal
    const Vec3r rigid_dp = rigid_p_cur - rigid_p_prev;

    // get the relative movement of the two colliding points...
    const Vec3r relative_dp = dp - rigid_dp;
    const Vec3r dp_tan = relative_dp - (relative_dp.dot(_collision_normal))*_collision_normal;    // in the directions tangent to the collision normal

    // the mass at the deformable colliding point - weighted average of masses of the face vertices depending on barycentric coords
    const Real m = _u*(1.0/_positions[0].inv_mass) + _v*(1.0/_positions[1].inv_mass) + _w*(1.0/_positions[2].inv_mass);

    // check to see if static friction should be enforced
    if (dp_tan.norm() < mu_s*lam/m)
    {
        // static friction is applied by undoing tangential movement this frame 
        // only do it for the vertices on the deformable object that have barycentric coordinates > 0

        if (_u>1e-4)
        {
            const Vec3r dp1 = (p1 - p1_prev) - rigid_dp;
            const Vec3r dp1_tan = dp1 - (dp1.dot(_collision_normal))*_collision_normal;
            _positions[0].position_ptr[0] -= dp1_tan[0];
            _positions[0].position_ptr[1] -= dp1_tan[1];
            _positions[0].position_ptr[2] -= dp1_tan[2];
        }
            
        if (_v>1e-4)
        {
            const Vec3r dp2 = (p2 - p2_prev) - rigid_dp;
            const Vec3r dp2_tan = dp2 - (dp2.dot(_collision_normal))*_collision_normal;
            _positions[1].position_ptr[0] -= dp2_tan[0];
            _positions[1].position_ptr[1] -= dp2_tan[1];
            _positions[1].position_ptr[2] -= dp2_tan[2];

        }

        if (_w>1e-4)
        {
            const Vec3r dp3 = (p3 - p3_prev) - rigid_dp;
            const Vec3r dp3_tan = dp3 - (dp3.dot(_collision_normal))*_collision_normal;
            _positions[2].position_ptr[0] -= dp3_tan[0];
            _positions[2].position_ptr[1] -= dp3_tan[1];
            _positions[2].position_ptr[2] -= dp3_tan[2];
        }
        
        // Rigid body feedback is handled by integration, but explicit force/torque application 
        // would be needed here for full two-way coupling. 
        // For static friction, we just stop the deformable sliding.
    }
    // if not static friction, apply dynamic friction
    else if (dp_tan.norm() > 1e-9) // Prevent division by zero
    {
        // calculate the positional correction for each vertex - never greater than the size of the tangential movement this time step
        const Vec3r corr_v1 = -_u * dp_tan * std::min(_positions[0].inv_mass*mu_k*lam/dp_tan.norm(), Real(1.0));
        const Vec3r corr_v2 = -_v * dp_tan * std::min(_positions[1].inv_mass*mu_k*lam/dp_tan.norm(), Real(1.0));
        const Vec3r corr_v3 = -_w * dp_tan * std::min(_positions[2].inv_mass*mu_k*lam/dp_tan.norm(), Real(1.0));
        
        _positions[0].position_ptr[0] += corr_v1[0];
        _positions[0].position_ptr[1] += corr_v1[1];
        _positions[0].position_ptr[2] += corr_v1[2];

        _positions[1].position_ptr[0] += corr_v2[0];
        _positions[1].position_ptr[1] += corr_v2[1];
        _positions[1].position_ptr[2] += corr_v2[2];

        _positions[2].position_ptr[0] += corr_v3[0];
        _positions[2].position_ptr[1] += corr_v3[1];
        _positions[2].position_ptr[2] += corr_v3[2];
    }
}
// OLD COMMENTED OUT CODE REMOVED

//     const Vec3r body_torque = GeometryUtils::rotateVectorByQuat(torque, GeometryUtils::inverseQuat(_rigid_bodies[0]->orientation()));
//     const Vec3r body_omega = _rigid_bodies[0]->invI() * body_torque;
//     const Vec3r global_omega = GeometryUtils::rotateVectorByQuat(body_omega, _rigid_bodies[0]->orientation());
//     const Vec4r w4({global_omega[0], global_omega[1], global_omega[2], 0.0});
//     const Vec4r q_update = 0.5 * (GeometryUtils::quatMult(w4, _rigid_bodies[0]->orientation()));
//     _rigid_bodies[0]->setOrientation(_rigid_bodies[0]->orientation() + q_update);

//     // update position
//     _rigid_bodies[0]->setPosition(_rigid_bodies[0]->position() + force_dtdt / _rigid_bodies[0]->mass() );
// }

} // namespace Solver