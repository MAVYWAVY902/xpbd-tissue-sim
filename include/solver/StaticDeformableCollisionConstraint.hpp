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
        *C = _collision_normal.dot(a - _p);
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
        const Eigen::Vector3d dp_tan = dp - (dp.dot(_collision_normal))*_collision_normal;

        const double m = _u*(1.0/_positions[0].inv_mass) + _v*(1.0/_positions[1].inv_mass) + _w*(1.0/_positions[2].inv_mass);

        if (dp_tan.norm() < mu_s*lam/m)
        {
            if (_u>1e-4 && dp_tan.norm() < mu_s*lam*_positions[0].inv_mass)
            {
                const Eigen::Vector3d dp1 = p1 - p1_prev;
                const Eigen::Vector3d dp1_tan = dp1 - (dp1.dot(_collision_normal))*_collision_normal;
                _positions[0].position_ptr[0] -= dp1_tan[0];
                _positions[0].position_ptr[1] -= dp1_tan[1];
                _positions[0].position_ptr[2] -= dp1_tan[2];
            }
                
            if (_v>1e-4 && dp_tan.norm() < mu_s*lam*_positions[1].inv_mass)
            {
                const Eigen::Vector3d dp2 = p2 - p2_prev;
                const Eigen::Vector3d dp2_tan = dp2 - (dp2.dot(_collision_normal))*_collision_normal;
                _positions[1].position_ptr[0] -= dp2_tan[0];
                _positions[1].position_ptr[1] -= dp2_tan[1];
                _positions[1].position_ptr[2] -= dp2_tan[2];

            }

            if (_w>1e-4 && dp_tan.norm() < mu_s*lam*_positions[2].inv_mass)
            {
                const Eigen::Vector3d dp3 = p3 - p3_prev;
                const Eigen::Vector3d dp3_tan = dp3 - (dp3.dot(_collision_normal))*_collision_normal;
                _positions[2].position_ptr[0] -= dp3_tan[0];
                _positions[2].position_ptr[1] -= dp3_tan[1];
                _positions[2].position_ptr[2] -= dp3_tan[2];
            }
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
        }
    }

    protected:
    const Geometry::SDF* _sdf;
    Eigen::Vector3d _p;
    double _u;
    double _v;
    double _w;

};

} // namespace Solver

#endif // __STATIC_DEFORMABLE_COLLISION_CONSTRAINT_HPP