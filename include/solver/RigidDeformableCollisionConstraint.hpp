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
        _sdf(sdf), _point_on_rigid_body(rigid_body_point), _u(u), _v(v), _w(w)
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
        *C = _sdf->evaluate(a) + 1e-4;
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

    inline virtual Eigen::Vector3d p1() const override
    {
        return _u*Eigen::Map<Eigen::Vector3d>(_positions[0].position_ptr) + 
               _v*Eigen::Map<Eigen::Vector3d>(_positions[1].position_ptr) + 
               _w*Eigen::Map<Eigen::Vector3d>(_positions[2].position_ptr);
    }

    inline virtual Eigen::Vector3d p2() const override
    {
        return _point_on_rigid_body;
    }

    inline virtual Eigen::Vector3d prevP1() const override
    {
        return _u*Eigen::Map<Eigen::Vector3d>(_positions[0].prev_position_ptr) + 
               _v*Eigen::Map<Eigen::Vector3d>(_positions[1].prev_position_ptr) + 
               _w*Eigen::Map<Eigen::Vector3d>(_positions[2].prev_position_ptr);
    }

    inline virtual Eigen::Vector3d prevP2() const override
    {
        const Sim::RigidObject* obj = _rigid_bodies[0];
        const Eigen::Vector3d p_body = obj->globalToBody(_point_on_rigid_body);
        return obj->prevPosition() + GeometryUtils::rotateVectorByQuat(p_body, obj->prevOrientation());
    }

    inline virtual double u() const override { return _u; }
    inline virtual double v() const override { return _v; }
    inline virtual double w() const override { return _w; }

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