#include "solver/constraint/HydrostaticConstraint.hpp"


namespace Solver
{

HydrostaticConstraint::HydrostaticConstraint(int v1, Real* p1, Real m1,
                          int v2, Real* p2, Real m2,
                          int v3, Real* p3, Real m3,
                          int v4, Real* p4, Real m4,
                          const ElasticMaterial& material)
    : ElementConstraint(v1, p1, m1, v2, p2, m2, v3, p3, m3, v4, p4, m4)
{
    _alpha = 1/(material.lambda() * _volume);            // set alpha after the ElementConstraint constructor because we need the element volume
    _gamma = material.mu() / material.lambda();  
}

void HydrostaticConstraint::evaluate(Real* C) const
{
    Real F[9];
    Real X[9];

    _computeF(F, X);
    _evaluate(C, F);
}

void HydrostaticConstraint::gradient(Real* grad) const
{
    Real F[9];
    Real X[9];
    _computeF(F, X);
    _gradient(grad, F);     // additional memory needed for the gradient calculation (after F and X) is provided after X
}


void HydrostaticConstraint::evaluateWithGradient(Real* C, Real* grad) const
{
    Real F[9];
    Real X[9];
    _computeF(F, X);
    _evaluate(C, F);
    _gradient(grad, F);
}

#ifdef HAVE_CUDA
HydrostaticConstraint::GPUConstraintType HydrostaticConstraint::createGPUConstraint() const
{
    GPUConstraintType gpu_constraint = GPUConstraintType(_positions[0].index, _positions[0].inv_mass,
                                                            _positions[1].index, _positions[1].inv_mass,
                                                            _positions[2].index, _positions[2].inv_mass,
                                                            _positions[3].index, _positions[3].inv_mass,
                                                            _Q, _alpha, _gamma);
    return gpu_constraint;
}
#endif

inline void HydrostaticConstraint::_evaluate(Real* C, Real* F) const
{
    // compute C(x) = det(F) - (1 + gamma)
    *C = F[0]*F[4]*F[8] - F[0]*F[7]*F[5] - F[3]*F[1]*F[8] + F[3]*F[7]*F[2] + F[6]*F[1]*F[5] - F[6]*F[4]*F[2] - (1+_gamma);
}

inline void HydrostaticConstraint::_gradient(Real* delC, Real* F) const
{   
    // F_cross = [f2 x f3, f3 x f1, f1 x f2] where f_i are the columns of F
    // F_cross and F are both column-major
    // see supplementary material of Macklin paper for more details
    Real F_cross[9];
    _cross3(F+3, F+6, F_cross);             // 2nd column of F crossed with 3rd column
    _cross3(F+6, F, F_cross+3);             // 3rd column of F crossed with 1st column
    _cross3(F, F+3, F_cross+6);             // 1st column of F crossed with 2nd column

    // for A = F_cross * Q^T,
    // 1st column of A is delC wrt 1st position
    // 2nd column of A is delC wrt 2nd position
    // 3rd column of A is delC wrt 3rd position
    // delC wrt 4th position is (-1st column - 2nd column - 3rd column)
    // see supplementary material of Macklin paper for more details

    // calculation of delC wrt 1st position
    delC[0] = (F_cross[0]*_Q(0,0) + F_cross[3]*_Q(0,1) + F_cross[6]*_Q(0,2));
    delC[1] = (F_cross[1]*_Q(0,0) + F_cross[4]*_Q(0,1) + F_cross[7]*_Q(0,2));
    delC[2] = (F_cross[2]*_Q(0,0) + F_cross[5]*_Q(0,1) + F_cross[8]*_Q(0,2));

    // calculation of delC wrt 2nd postion
    delC[3] = (F_cross[0]*_Q(1,0) + F_cross[3]*_Q(1,1) + F_cross[6]*_Q(1,2));
    delC[4] = (F_cross[1]*_Q(1,0) + F_cross[4]*_Q(1,1) + F_cross[7]*_Q(1,2));
    delC[5] = (F_cross[2]*_Q(1,0) + F_cross[5]*_Q(1,1) + F_cross[8]*_Q(1,2));

    // calculation of delC wrt 3rd position
    delC[6] = (F_cross[0]*_Q(2,0) + F_cross[3]*_Q(2,1) + F_cross[6]*_Q(2,2));
    delC[7] = (F_cross[1]*_Q(2,0) + F_cross[4]*_Q(2,1) + F_cross[7]*_Q(2,2));
    delC[8] = (F_cross[2]*_Q(2,0) + F_cross[5]*_Q(2,1) + F_cross[8]*_Q(2,2));

    // calculation of delC wrt 4th position
    delC[9]  = -delC[0] - delC[3] - delC[6];
    delC[10] = -delC[1] - delC[4] - delC[7];
    delC[11] = -delC[2] - delC[5] - delC[8];

    // std::cout << "Ch_grad: " << grad[0] << ", " << grad[1] << ", " << grad[2] << ", " << grad[3] << ", " << grad[4] << ", " << grad[5] << ", " << grad[6] << ", " << grad[7] << ", " << grad[8] << ", " << grad[9] << ", " << grad[10] << ", " << grad[11] << std::endl;
}

inline void HydrostaticConstraint::_cross3(const Real* v1, const Real* v2, Real* v3) const
{
    v3[0] = v1[1]*v2[2] - v1[2]*v2[1];
    v3[1] = v1[2]*v2[0] - v1[0]*v2[2];
    v3[2] = v1[0]*v2[1] - v1[1]*v2[0];
}

} // namespace Solver