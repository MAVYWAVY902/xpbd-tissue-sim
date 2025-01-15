#ifndef __RIGID_DEFORMABLE_COLLISION_CONSTRAINT_HPP
#define __RIGID_DEFORMABLE_COLLISION_CONSTRAINT_HPP

#include "solver/RigidBodyConstraint.hpp"
#include "simobject/RigidObject.hpp"
#include "simobject/XPBDMeshObject.hpp"
#include "geometry/SDF.hpp"

namespace Solver
{

class RigidDeformableCollisionConstraint : public RigidBodyConstraint
{
    public:
    /** 
     * @param sdf : the SDF for the rigid object
     * @param rigid_obj : the rigid object itself
     * @param p : the point on the rigid object in BODY coordinates
     * @param n : the collision normal in BODY coordinates
     * @param deformable_obj : the deformable object itself
     * @param v1,v2,v3 : the vertex indices of the colliding face
     * @param u,v,w : the barycentric coordinates of the the colliding point on the colliding face
     */
    RigidDeformableCollisionConstraint(const Geometry::SDF* sdf, Sim::RigidObject* rigid_obj, const Eigen::Vector3d& rigid_body_point, const Eigen::Vector3d& collision_normal,
                                       const Sim::XPBDMeshObject* deformable_obj, const int v1, const int v2, const int v3, const double u, const double v, const double w)
    : RigidBodyConstraint(std::vector<PositionReference>({
        PositionReference(deformable_obj, v1),
        PositionReference(deformable_obj, v2),
        PositionReference(deformable_obj, v3)}), std::vector<Sim::RigidObject*>({rigid_obj})),
        _sdf(sdf), _rigid_obj(rigid_obj), _rigid_body_point(rigid_body_point), _collision_normal(collision_normal), _u(u), _v(v), _w(w)
    {
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
        
        
        // const Eigen::Vector3d p_global = _rigid_obj->bodyToGlobal(_p);
        // const Eigen::Vector3d n_global = GeometryUtils::rotateVectorByQuat(_n, _rigid_obj->orientation());
        // *C = n_global.dot(a - p_global) + 1e-4;
        // _updateCollisionInfo();
        _evaluate(C);
    }

    /** Computes the gradient of this constraint in vector form with pre-allocated memory.
     * i.e. returns delC(x)
     * 
     * @param grad (OUTPUT) - the pointer to the (currently empty) constraint gradient vector. Expects it to be _gradient_vector_size x 1.
     */
    inline void gradient(double* grad) const override
    {
        // _updateCollisionInfo();
        _gradient(grad);
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
        // _updateCollisionInfo();
        _evaluate(C);
        _gradient(grad);
    }

    // Eigen::Vector3d pointOnRigidBody() const { return _rigid_body_point; }
    // Eigen::Vector3d collisionNormal() const { return _collision_normal; }
    // Sim::RigidObject* rigidObj() const { return _rigid_obj; }


    /** Collision constraints should be implemented as inequalities, i.e. as C(x) >= 0. */
    inline virtual bool isInequality() const override { return true; }

    private:
    // void _updateCollisionInfo() const
    // {
    //     // TODO: maybe try doing another local optimization using Franke-Wolfe?
    //     const Eigen::Vector3d a = _u*Eigen::Map<Eigen::Vector3d>(_positions[0].position_ptr) + _v*Eigen::Map<Eigen::Vector3d>(_positions[1].position_ptr) + _w*Eigen::Map<Eigen::Vector3d>(_positions[2].position_ptr);
    //     _penetration_dist = _sdf->evaluate(a);
    //     _rigid_body_helpers[0]->setDirection(_sdf->gradient(a));
    //     _rigid_body_helpers[0]->setPointOnBody(a - _penetration_dist * _collision_normal);
    // }

    void _evaluate(double* C) const
    {
        const Eigen::Vector3d a = _u*Eigen::Map<Eigen::Vector3d>(_positions[0].position_ptr) + _v*Eigen::Map<Eigen::Vector3d>(_positions[1].position_ptr) + _w*Eigen::Map<Eigen::Vector3d>(_positions[2].position_ptr);
        *C = _sdf->evaluate(a) + 1e-4;
    }

    void _gradient(double* grad) const
    {
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

    protected:
    const Geometry::SDF* _sdf;
    Sim::RigidObject* _rigid_obj;
    mutable Eigen::Vector3d _rigid_body_point;       // the point on the surface of the rigid body that is colliding
    Eigen::Vector3d _collision_normal;
    double _u;
    double _v;
    double _w;


};

} // namespace Solver

#endif // __RIGID_DEFORMABLE_COLLISION_CONSTRAINT_HPP