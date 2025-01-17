#ifndef __RIGID_DEFORMABLE_COLLISION_CONSTRAINT_HPP
#define __RIGID_DEFORMABLE_COLLISION_CONSTRAINT_HPP

#include "solver/RigidBodyConstraint.hpp"
#include "solver/CollisionConstraint.hpp"
#include "simobject/RigidObject.hpp"
#include "simobject/XPBDMeshObject.hpp"
#include "geometry/SDF.hpp"

namespace Solver
{

/** Defines a collision constraint between a rigid body and a deformable body.
 * The collision is defined to be between two points - one on the deformable body (p1) and one on the rigid body (p2) - these are obtained from collision detection.
 * (The point on the deformable body may be between vertices in the middle of an edge or a face, so we represent p1 in barycentric coordinates (u,v,w) )
 * The collision also has a collision normal (n), which is the direction of minimum separation - also obtained from collision detection.
 * We then define the constraint C(x) = (p1 - p2) * n, where * is the dot product. In other words, the constraint value is the penetration distance (negative if penetrating, positive if not).
 * 
 * If there is no penetration, then this constraint is not enforced (i.e it is an XPBD inequality constraint)
 * 
 * This collision constraint may be used for multiple frames, and thus the colliding bodies will evolve in time past the point of collision. It's possible that the original
 * collision parameters p1, p2, and n are no longer accurate after the object has moved, meaning that the expression (p1 - p2) * n may be negative even though the objects are no
 * longer penetrating. Instead, we can simply use the Signed Distance Function (SDF) that was used for collision detection to get the penetration distance, so that we
 * are completely certain when the objects are penetrating or not (and thus whether or not we should enforce this constraint).
 * 
 * As of right now, the points on the body that are penetrating (p1 and p2) and the collision normal (n) do not evolve in time and are taken to be fixed (despite the penetration
 * distance given by the SDF evolving in time). So far, this seems to be a reasonable assumption - i.e. if the bodies are still colliding after multiple frames, it is highly
 * likely that the penetrating points and collision normal are pretty similar to what they were when this collision constraint was created.
 */
class RigidDeformableCollisionConstraint : public RigidBodyConstraint, public CollisionConstraint
{
    public:
    /** 
     * @param sdf : the SDF for the rigid object
     * @param rigid_obj : pointer to the rigid object
     * @param p : the point on the rigid object in global coordinates
     * @param n : the collision normal in global coordinates
     * @param deformable_obj : pointer to the deformable object
     * @param v1,v2,v3 : the vertex indices of the colliding face on the deformable object
     * @param u,v,w : the barycentric coordinates of the the colliding point on the colliding face of the deformable object
     */
    RigidDeformableCollisionConstraint(const Geometry::SDF* sdf, Sim::RigidObject* rigid_obj, const Eigen::Vector3d& rigid_body_point, const Eigen::Vector3d& collision_normal,
                                       const Sim::XPBDMeshObject* deformable_obj, const int v1, const int v2, const int v3, const double u, const double v, const double w)
    : CollisionConstraint(std::vector<PositionReference>({
        PositionReference(deformable_obj, v1),
        PositionReference(deformable_obj, v2),
        PositionReference(deformable_obj, v3)}), collision_normal),
        RigidBodyConstraint(std::vector<Sim::RigidObject*>({rigid_obj})),
        _sdf(sdf), _point_on_rigid_body(rigid_obj->globalToBody(rigid_body_point)), _u(u), _v(v), _w(w)
    {
        // create the Helper class that will evaluate the rigid body "weight" and the rigid body update when this constraint is projected
        // it is created here because the Helper needs info from the collision itself, such as the normal and the point on the rigid body
        std::unique_ptr<RigidBodyXPBDHelper> helper = std::make_unique<PositionalRigidBodyXPBDHelper>(rigid_obj, -collision_normal, rigid_body_point);
        _rigid_body_helpers.push_back(std::move(helper));
    }

    /** Evaluates the current value of this constraint.
     * i.e. returns C(x)
     */
    inline double evaluate() const override
    {
        assert(0); // not implemented
        return 0;
    }

    /** Returns the gradient of this constraint in vector form.
     * i.e. returns delC(x)
     */
    inline Eigen::VectorXd gradient() const override
    {
        assert(0); // not implemented
        return Eigen::VectorXd();
    }

    /** Returns the value and gradient of this constraint.
     * i.e. returns C(x) and delC(x) together.
     * 
     * This may be desirable when there would be duplicate work involved to evaluate constraint and its gradient separately.
     */
    inline Constraint::ValueAndGradient evaluateWithGradient() const override
    {
        assert(0);  // not implemented
        return ValueAndGradient(evaluate(), gradient());
    }

    /** Evaluates the current value of this constraint with pre-allocated memory.
     * i.e. returns C(x)
     * 
     * @param C (OUTPUT) - the pointer to the (currently empty) value of the constraint
     */
    inline void evaluate(double* C) const override
    {
        // get the point on the deformable body from the barycentric coordinates
        const Eigen::Vector3d a = _u*Eigen::Map<Eigen::Vector3d>(_positions[0].position_ptr) + _v*Eigen::Map<Eigen::Vector3d>(_positions[1].position_ptr) + _w*Eigen::Map<Eigen::Vector3d>(_positions[2].position_ptr);
        
        // constraint value is the penetration distance, which we can get from the rigid body SDF
        *C = _sdf->evaluate(a);
    }

    /** Computes the gradient of this constraint in vector form with pre-allocated memory.
     * i.e. returns delC(x)
     * 
     * @param grad (OUTPUT) - the pointer to the (currently empty) constraint gradient vector. Expects it to be _gradient_vector_size x 1.
     */
    inline void gradient(double* grad) const override
    {
        // gradient w.r.t each deformable position involved is simply just the collision normal multiplied by the corresponding barycentric coordinate
        // i.e. C = (u*v1 + v*v2 + w*v3 - p1) * n ==> dC/dv1 = u*n, dC/dv2 = v*n, dC/dv3 = w*n
        grad[_gradient_vector_index[0]] = _u*_collision_normal[0];
        grad[_gradient_vector_index[1]] = _u*_collision_normal[1];
        grad[_gradient_vector_index[2]] = _u*_collision_normal[2];

        grad[_gradient_vector_index[3]] = _v*_collision_normal[0];
        grad[_gradient_vector_index[4]] = _v*_collision_normal[1];
        grad[_gradient_vector_index[5]] = _v*_collision_normal[2];
        
        grad[_gradient_vector_index[6]] = _w*_collision_normal[0];
        grad[_gradient_vector_index[7]] = _w*_collision_normal[1];
        grad[_gradient_vector_index[8]] = _w*_collision_normal[2];
    }


    /** Computes the value and gradient of this constraint with pre-allocated memory.
     * i.e. returns C(x) and delC(x) together.
     * 
     * This may be desirable when there would be duplicate work involved to evaluate constraint and its gradient separately.
     * 
     * @param C (OUTPUT) - the pointer to the (currently empty) value of the constraint
     * @param grad (OUTPUT) - the pointer to the (currently empty) constraint gradient vector. Expects it to be _gradient_vector_size x 1.
     */
    void evaluateWithGradient(double* C, double* grad) const override
    {
        evaluate(C);
        gradient(grad);
    }


    /** Collision constraints should be implemented as inequalities, i.e. as C(x) >= 0. */
    inline virtual bool isInequality() const override { return true; }

    inline virtual void applyFriction(double lam, double mu_s, double mu_k) const override
    {
        const Eigen::Vector3d p1 = Eigen::Map<Eigen::Vector3d>(_positions[0].position_ptr);
        const Eigen::Vector3d p2 = Eigen::Map<Eigen::Vector3d>(_positions[1].position_ptr);
        const Eigen::Vector3d p3 = Eigen::Map<Eigen::Vector3d>(_positions[2].position_ptr);

        const Eigen::Vector3d p1_prev = Eigen::Map<Eigen::Vector3d>(_positions[0].prev_position_ptr);
        const Eigen::Vector3d p2_prev = Eigen::Map<Eigen::Vector3d>(_positions[1].prev_position_ptr);
        const Eigen::Vector3d p3_prev = Eigen::Map<Eigen::Vector3d>(_positions[2].prev_position_ptr);

        const Eigen::Vector3d p_cur = _u*p1 + _v*p2 + _w*p3;
        const Eigen::Vector3d p_prev = _u*p1_prev + _v*p2_prev + _w*p3_prev;
        const Eigen::Vector3d dp = p_cur - p_prev;

        const Sim::RigidObject* obj = _rigid_bodies[0];
        // TODO: make point_on_rigid_body in body coords
        const Eigen::Vector3d rigid_p_cur = obj->bodyToGlobal(_point_on_rigid_body);
        const Eigen::Vector3d rigid_p_prev = obj->prevPosition() + GeometryUtils::rotateVectorByQuat(_point_on_rigid_body, obj->prevOrientation());
        const Eigen::Vector3d rigid_dp = rigid_p_cur - rigid_p_prev;

        // std::cout << "position: " << obj->position()[0] << ", " << obj->position()[1] << ", " << obj->position()[2] << std::endl;
        // std::cout << "prev_position: " << obj->prevPosition()[0] << ", " << obj->prevPosition()[1] << ", " << obj->prevPosition()[2] << std::endl;
        // std::cout << "rigid_p_cur: " << rigid_p_cur[0] << ", " << rigid_p_cur[1] << ", " << rigid_p_cur[2] << std::endl;
        // std::cout << "rigid_p_prev: " << rigid_p_prev[0] << ", " << rigid_p_prev[1] << ", " << rigid_p_prev[2] << std::endl;
        // std::cout << "rigid_dp:\n" << rigid_dp << std::endl;

        const Eigen::Vector3d relative_dp = dp - rigid_dp;

        const Eigen::Vector3d dp_tan = relative_dp - (relative_dp.dot(_collision_normal))*_collision_normal;

        const double m = _u*(1.0/_positions[0].inv_mass) + _v*(1.0/_positions[1].inv_mass) + _w*(1.0/_positions[2].inv_mass);

        Eigen::Vector3d force_dtdt = Eigen::Vector3d::Zero();
        if (dp_tan.norm() < mu_s*lam/m)
        {
            if (_u>1e-4 && dp_tan.norm() < mu_s*lam*_positions[0].inv_mass)
            {
                const Eigen::Vector3d dp1 = (p1 - p1_prev) - rigid_dp;
                const Eigen::Vector3d dp1_tan = dp1 - (dp1.dot(_collision_normal))*_collision_normal;
                _positions[0].position_ptr[0] -= dp1_tan[0];
                _positions[0].position_ptr[1] -= dp1_tan[1];
                _positions[0].position_ptr[2] -= dp1_tan[2];
            }
                
            if (_v>1e-4 && dp_tan.norm() < mu_s*lam*_positions[1].inv_mass)
            {
                const Eigen::Vector3d dp2 = (p2 - p2_prev) - rigid_dp;
                const Eigen::Vector3d dp2_tan = dp2 - (dp2.dot(_collision_normal))*_collision_normal;
                _positions[1].position_ptr[0] -= dp2_tan[0];
                _positions[1].position_ptr[1] -= dp2_tan[1];
                _positions[1].position_ptr[2] -= dp2_tan[2];

            }

            if (_w>1e-4 && dp_tan.norm() < mu_s*lam*_positions[2].inv_mass)
            {
                const Eigen::Vector3d dp3 = (p3 - p3_prev) - rigid_dp;
                const Eigen::Vector3d dp3_tan = dp3 - (dp3.dot(_collision_normal))*_collision_normal;
                _positions[2].position_ptr[0] -= dp3_tan[0];
                _positions[2].position_ptr[1] -= dp3_tan[1];
                _positions[2].position_ptr[2] -= dp3_tan[2];
            }

            force_dtdt = dp_tan;

        }
        else
        {
            const Eigen::Vector3d corr_v1 = -_u * dp_tan * std::min(_positions[0].inv_mass*mu_k*lam/dp_tan.norm(), 1.0);
            const Eigen::Vector3d corr_v2 = -_v * dp_tan * std::min(_positions[1].inv_mass*mu_k*lam/dp_tan.norm(), 1.0);
            const Eigen::Vector3d corr_v3 = -_w * dp_tan * std::min(_positions[2].inv_mass*mu_k*lam/dp_tan.norm(), 1.0);
            _positions[0].position_ptr[0] += corr_v1[0];
            _positions[0].position_ptr[1] += corr_v1[1];
            _positions[0].position_ptr[2] += corr_v1[2];

            _positions[1].position_ptr[0] += corr_v2[0];
            _positions[1].position_ptr[1] += corr_v2[1];
            _positions[1].position_ptr[2] += corr_v2[2];

            _positions[2].position_ptr[0] += corr_v3[0];
            _positions[2].position_ptr[1] += corr_v3[1];
            _positions[2].position_ptr[2] += corr_v3[2];

            force_dtdt = -(corr_v1/_positions[0].inv_mass + corr_v2/_positions[1].inv_mass + corr_v3/_positions[2].inv_mass);
            // _rigid_bodies[0]->applyForceAtPoint(force, rigid_p_cur);
        }

        // update orientation - compute torque caused by applied force
        const Eigen::Vector3d torque = (rigid_p_cur - _rigid_bodies[0]->position()).cross(force_dtdt);
        const Eigen::Vector3d body_torque = GeometryUtils::rotateVectorByQuat(torque, GeometryUtils::inverseQuat(_rigid_bodies[0]->orientation()));
        const Eigen::Vector3d body_omega = _rigid_bodies[0]->invI() * body_torque;
        const Eigen::Vector3d global_omega = GeometryUtils::rotateVectorByQuat(body_omega, _rigid_bodies[0]->orientation());
        const Eigen::Vector4d w4({global_omega[0], global_omega[1], global_omega[2], 0.0});
        const Eigen::Vector4d q_update = 0.5 * (GeometryUtils::quatMult(w4, _rigid_bodies[0]->orientation()));
        _rigid_bodies[0]->setOrientation(_rigid_bodies[0]->orientation() + q_update);

        // update position
        _rigid_bodies[0]->setPosition(_rigid_bodies[0]->position() + force_dtdt / _rigid_bodies[0]->mass() );
    }

    private:

    protected:
    const Geometry::SDF* _sdf;  // the SDF of the rigid body - used to get accurate penetration distance
    Eigen::Vector3d _point_on_rigid_body; // the colliding point on the rigid body, in global coordinates
    double _u;  // barycentric coordinates of point on deformable body
    double _v;  // the vertices themselves are stored in the _positions vector defined in the Constraint base class
    double _w;


};

} // namespace Solver

#endif // __RIGID_DEFORMABLE_COLLISION_CONSTRAINT_HPP