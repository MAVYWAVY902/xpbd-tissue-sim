#include "solver/constraint/DeviatoricConstraint.hpp"

namespace Solver
{

DeviatoricConstraint::DeviatoricConstraint(int v1, Real* p1, Real m1,
                        int v2, Real* p2, Real m2,
                        int v3, Real* p3, Real m3,
                        int v4, Real* p4, Real m4,
                        const ElasticMaterial& material)
    : ElementConstraint(v1, p1, m1, v2, p2, m2, v3, p3, m3, v4, p4, m4)
{
    _alpha = 1/(material.mu() * _volume); // set alpha after the ElementConstraint constructor because we need the element volume
}


void DeviatoricConstraint::evaluate(Real* C) const
{
    Real F[9];
    Real X[9];
    _computeF(F, X);
    _evaluate(C, F);
}

void DeviatoricConstraint::gradient(Real* grad) const
{
    Real F[9];
    Real X[9];
    _computeF(F, X);
    Real C;
    _evaluate(&C, F);                   // we need C(x) since it is used in the gradient calculation
    _gradient(grad, &C, F);
}

void DeviatoricConstraint::evaluateWithGradient(Real* C, Real* grad) const
{
    Real F[9];
    Real X[9];
    _computeF(F, X);
    _evaluate(C, F);
    _gradient(grad, C, F);
}

#ifdef HAVE_CUDA
DeviatoricConstraint::GPUConstraintType DeviatoricConstraint::createGPUConstraint() const
{
    GPUConstraintType gpu_constraint = GPUConstraintType(_positions[0].index, _positions[0].inv_mass,
                                                            _positions[1].index, _positions[1].inv_mass,
                                                            _positions[2].index, _positions[2].inv_mass,
                                                            _positions[3].index, _positions[3].inv_mass,
                                                            _Q, _alpha);
    return gpu_constraint;
}
#endif

inline void DeviatoricConstraint::_evaluate(Real* C, Real* F) const
{
    // C = frob(F)
    *C = std::sqrt(F[0]*F[0] + F[1]*F[1] + F[2]*F[2] + F[3]*F[3] + F[4]*F[4] + F[5]*F[5] + F[6]*F[6] + F[7]*F[7] + F[8]*F[8]);
}

inline void DeviatoricConstraint::_gradient(Real* delC, Real* C, Real* F) const
{
    // for A = 1/C * F * Q^T,
    // 1st column of A is delC wrt 1st position
    // 2nd column of A is delC wrt 2nd position
    // 3rd column of A is delC wrt 3rd position
    // delC wrt 4th position is (-1st column - 2nd column - 3rd column)
    // see supplementary material of Macklin paper for more details

    Real inv_C = 1.0/(*C);

    // F is column major
    // calculation of delC wrt 1st position
    delC[0] = inv_C * (F[0]*_Q(0,0) + F[3]*_Q(0,1) + F[6]*_Q(0,2));
    delC[1] = inv_C * (F[1]*_Q(0,0) + F[4]*_Q(0,1) + F[7]*_Q(0,2));
    delC[2] = inv_C * (F[2]*_Q(0,0) + F[5]*_Q(0,1) + F[8]*_Q(0,2));

    // calculation of delC wrt 2nd position
    delC[3] = inv_C * (F[0]*_Q(1,0) + F[3]*_Q(1,1) + F[6]*_Q(1,2));
    delC[4] = inv_C * (F[1]*_Q(1,0) + F[4]*_Q(1,1) + F[7]*_Q(1,2));
    delC[5] = inv_C * (F[2]*_Q(1,0) + F[5]*_Q(1,1) + F[8]*_Q(1,2));

    // calculation of delC wrt 3rd position
    delC[6] = inv_C * (F[0]*_Q(2,0) + F[3]*_Q(2,1) + F[6]*_Q(2,2));
    delC[7] = inv_C * (F[1]*_Q(2,0) + F[4]*_Q(2,1) + F[7]*_Q(2,2));
    delC[8] = inv_C * (F[2]*_Q(2,0) + F[5]*_Q(2,1) + F[8]*_Q(2,2));

    // calculation of delC wrt 4th position
    delC[9]  = -delC[0] - delC[3] - delC[6];
    delC[10] = -delC[1] - delC[4] - delC[7];
    delC[11] = -delC[2] - delC[5] - delC[8];
}

} // namespace Solver