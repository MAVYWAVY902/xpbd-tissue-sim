#ifndef __STATIC_DEFORMABLE_COLLISION_CONSTRAINT_HPP
#define __STATIC_DEFORMABLE_COLLISION_CONSTRAINT_HPP

#include "solver/CollisionConstraint.hpp"
#include "geometry/SDF.hpp"

namespace Solver
{

/** Collision constraint between a point on a static rigid body and a point on a deformable body. */
class StaticDeformableCollisionConstraint : public CollisionConstraint
{
    public:
    StaticDeformableCollisionConstraint(const Geometry::SDF* sdf, const Eigen::Vector3d& p, const Eigen::Vector3d& n,
                                        const Sim::XPBDMeshObject* obj, const int v1, const int v2, const int v3, const double u, const double v, const double w)
        : CollisionConstraint(std::vector<PositionReference>({
        PositionReference(obj, v1),
        PositionReference(obj, v2),
        PositionReference(obj, v3)}), n),
        _sdf(sdf), _p(p), _u(u), _v(v), _w(w)
    {

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
        const Eigen::Vector3d a = _u*Eigen::Map<Eigen::Vector3d>(_positions[0].position_ptr) + _v*Eigen::Map<Eigen::Vector3d>(_positions[1].position_ptr) + _w*Eigen::Map<Eigen::Vector3d>(_positions[2].position_ptr);
        *C = _collision_normal.dot(a - _p) + 1e-4;
    }

    /** Computes the gradient of this constraint in vector form with pre-allocated memory.
     * i.e. returns delC(x)
     * 
     * @param grad (OUTPUT) - the pointer to the (currently empty) constraint gradient vector. Expects it to be _gradient_vector_size x 1.
     */
    inline void gradient(double* grad) const override
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
        return _p;
    }

    inline virtual Eigen::Vector3d prevP1() const override
    {
        return _u*Eigen::Map<Eigen::Vector3d>(_positions[0].prev_position_ptr) + 
               _v*Eigen::Map<Eigen::Vector3d>(_positions[1].prev_position_ptr) + 
               _w*Eigen::Map<Eigen::Vector3d>(_positions[2].prev_position_ptr);
    }

    inline virtual Eigen::Vector3d prevP2() const override
    {
        return _p;
    }

    inline virtual double u() const override { return _u; }
    inline virtual double v() const override { return _v; }
    inline virtual double w() const override { return _w; }

    protected:
    const Geometry::SDF* _sdf;
    Eigen::Vector3d _p;
    double _u;
    double _v;
    double _w;

};

} // namespace Solver

#endif // __STATIC_DEFORMABLE_COLLISION_CONSTRAINT_HPP