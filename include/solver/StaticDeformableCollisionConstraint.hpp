#ifndef __STATIC_DEFORMABLE_COLLISION_CONSTRAINT_HPP
#define __STATIC_DEFORMABLE_COLLISION_CONSTRAINT_HPP

#include "solver/Constraint.hpp"
#include "geometry/SDF.hpp"

namespace Solver
{

/** Collision constraint between a point on a static rigid body and a point on a deformable body. */
class StaticDeformableCollisionConstraint : public Constraint
{
    public:
    StaticDeformableCollisionConstraint(const Geometry::SDF* sdf, const Eigen::Vector3d& p, const Eigen::Vector3d& n,
                                        const Sim::XPBDMeshObject* obj, const int v1, const int v2, const int v3, const double u, const double v, const double w)
        : Constraint(std::vector<PositionReference>({
        PositionReference(obj, v1),
        PositionReference(obj, v2),
        PositionReference(obj, v3)})),
        _sdf(sdf), _p(p), _n(n), _u(u), _v(v), _w(w)
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
     * @param additional_memory - a pointer to some pre-allocated memory that will be used to compute intermediate values in the constraint calculation, if necessary
     */
    inline void evaluate(double* C, double* additional_memory) const override
    {
        const Eigen::Vector3d a = _u*Eigen::Map<Eigen::Vector3d>(_positions[0].position_ptr) + _v*Eigen::Map<Eigen::Vector3d>(_positions[1].position_ptr) + _w*Eigen::Map<Eigen::Vector3d>(_positions[2].position_ptr);
        *C = _n.dot(a - _p)/10.0 + 1e-4;
    }

    /** Computes the gradient of this constraint in vector form with pre-allocated memory.
     * i.e. returns delC(x)
     * 
     * @param grad (OUTPUT) - the pointer to the (currently empty) constraint gradient vector. Expects it to be _gradient_vector_size x 1.
     * @param additional_memory - a pointer to some pre-allocated memory that will be used to compute intermediate values in the constraint gradient calculation, if necessary
     */
    inline void gradient(double* grad, double* additional_memory) const override
    {
        grad[_gradient_vector_index[0]] = _u*_n[0];
        grad[_gradient_vector_index[1]] = _u*_n[1];
        grad[_gradient_vector_index[2]] = _u*_n[2];

        grad[_gradient_vector_index[3]] = _v*_n[0];
        grad[_gradient_vector_index[4]] = _v*_n[1];
        grad[_gradient_vector_index[5]] = _v*_n[2];
        
        grad[_gradient_vector_index[6]] = _w*_n[0];
        grad[_gradient_vector_index[7]] = _w*_n[1];
        grad[_gradient_vector_index[8]] = _w*_n[2];
    }


    /** Computes the value and gradient of this constraint with pre-allocated memory.
     * i.e. returns C(x) and delC(x) together.
     * 
     * This may be desirable when there would be duplicate work involved to evaluate constraint and its gradient separately.
     * 
     * @param C (OUTPUT) - the pointer to the (currently empty) value of the constraint
     * @param grad (OUTPUT) - the pointer to the (currently empty) constraint gradient vector. Expects it to be _gradient_vector_size x 1.
     * @param additional_memory - a pointer to some pre-allocated memory that will be used to compute intermediate values, if necessary
     */
    void evaluateWithGradient(double* C, double* grad, double* additional_memory) const override
    {
        evaluate(C, additional_memory);
        gradient(grad, additional_memory);
    }

    /** Returns the number of bytes of pre-allocated dynamic memory needed to do its computation. */
    inline size_t memoryNeeded() const override
    {
        return 0;
    }

    /** Collision constraints should be implemented as inequalities, i.e. as C(x) >= 0. */
    inline virtual bool isInequality() const override { return true; }

    private:
    inline double _dot3(const double* v1, const double* v2) const
    {
        return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
    }

    inline void _sub3 (const double* v1, const double* v2, double* v3) const
    {
        v3[0] = v1[0] - v2[0];
        v3[1] = v1[1] - v2[1];
        v3[2] = v1[2] - v2[2];
    }

    protected:
    const Geometry::SDF* _sdf;
    Eigen::Vector3d _p;
    Eigen::Vector3d _n;
    double _u;
    double _v;
    double _w;

};

} // namespace Solver

#endif // __STATIC_DEFORMABLE_COLLISION_CONSTRAINT_HPP