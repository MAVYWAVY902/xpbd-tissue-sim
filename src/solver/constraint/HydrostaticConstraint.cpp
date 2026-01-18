#include "solver/constraint/HydrostaticConstraint.hpp"

#include <iostream>

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
    _gradient(grad, F);
}

void HydrostaticConstraint::hessian(Real C, const Real* grad, Mat3r* hess) const
{
    // For Hydrostatic constraint: C = log(det(F)) - gamma
    // Based on Gaia's VBD_NeoHookean.cpp implementation
    // 
    // Key insight: Use correct chain rule transformation from F-space to x-space
    // H_vertex = (dF/dx)^T · H_F · (dF/dx)
    // where dF/dx comes from F = Ds·DmInv, Ds = [x0-x3, x1-x3, x2-x3]
    
    // Recompute F to get detF and derivatives
    Real F_data[9];
    Real X[9];
    _computeF(F_data, X);
    
    Eigen::Map<const Mat3r> F(F_data);
    Real detF = F.determinant();
    
    if (std::abs(detF) < 1e-12) {
        // Determinant too small, fill with zeros
        for (int i = 0; i < 16; i++) {
            hess[i].setZero();
        }
        return;
    }
    
    // Extract F components (column-major)
    const Real F11 = F(0,0), F21 = F(1,0), F31 = F(2,0);
    const Real F12 = F(0,1), F22 = F(1,1), F32 = F(2,1);
    const Real F13 = F(0,2), F23 = F(1,2), F33 = F(2,2);
    
    // Compute ∂(det F)/∂F (cofactor matrix)
    Eigen::Matrix<Real, 9, 1> ddetF_dF;
    ddetF_dF << F22*F33 - F23*F32,
                F13*F32 - F12*F33,
                F12*F23 - F13*F22,
                F23*F31 - F21*F33,
                F11*F33 - F13*F31,
                F13*F21 - F11*F23,
                F21*F32 - F22*F31,
                F12*F31 - F11*F32,
                F11*F22 - F12*F21;
    
    // Compute ∂²(det F)/∂F² (9×9 Hessian of determinant)
    // Following Gaia's formula: d2detF_dF2 = ddetF_dF * ddetF_dF.transpose() + k * [sparse terms]
    Eigen::Matrix<Real, 9, 9> d2detF_dF2 = ddetF_dF * ddetF_dF.transpose();
    
    // Add second-order correction terms
    const Real k = detF - (1.0 + _gamma);  // k = J - (1 + mu/lambda)
    
    d2detF_dF2(0, 4) += k * F33;  d2detF_dF2(4, 0) += k * F33;
    d2detF_dF2(0, 5) += k * -F23; d2detF_dF2(5, 0) += k * -F23;
    d2detF_dF2(0, 7) += k * -F32; d2detF_dF2(7, 0) += k * -F32;
    d2detF_dF2(0, 8) += k * F22;  d2detF_dF2(8, 0) += k * F22;
    
    d2detF_dF2(1, 3) += k * -F33; d2detF_dF2(3, 1) += k * -F33;
    d2detF_dF2(1, 5) += k * F13;  d2detF_dF2(5, 1) += k * F13;
    d2detF_dF2(1, 6) += k * F32;  d2detF_dF2(6, 1) += k * F32;
    d2detF_dF2(1, 8) += k * -F12; d2detF_dF2(8, 1) += k * -F12;
    
    d2detF_dF2(2, 3) += k * F23;  d2detF_dF2(3, 2) += k * F23;
    d2detF_dF2(2, 4) += k * -F13; d2detF_dF2(4, 2) += k * -F13;
    d2detF_dF2(2, 6) += k * -F22; d2detF_dF2(6, 2) += k * -F22;
    d2detF_dF2(2, 7) += k * F12;  d2detF_dF2(7, 2) += k * F12;
    
    d2detF_dF2(3, 7) += k * F31;  d2detF_dF2(7, 3) += k * F31;
    d2detF_dF2(3, 8) += k * -F21; d2detF_dF2(8, 3) += k * -F21;
    
    d2detF_dF2(4, 6) += k * -F31; d2detF_dF2(6, 4) += k * -F31;
    d2detF_dF2(4, 8) += k * F11;  d2detF_dF2(8, 4) += k * F11;
    
    d2detF_dF2(5, 6) += k * F21;  d2detF_dF2(6, 5) += k * F21;
    d2detF_dF2(5, 7) += k * -F11; d2detF_dF2(7, 5) += k * -F11;
    
    // For log(detF), apply chain rule: ∇²log(J) = (1/J)·∇²J - (1/J²)·(∇J ⊗ ∇J)
    const Real inv_detF = 1.0 / detF;
    const Real inv_detF2 = inv_detF * inv_detF;
    
    Eigen::Matrix<Real, 9, 9> d2C_dF2 = inv_detF * d2detF_dF2 - inv_detF2 * (ddetF_dF * ddetF_dF.transpose());
    
    // Transform from F-space (9×9) to x-space (12×12 as 16 3×3 blocks)
    // Using Gaia's assembleVertexVForceAndHessian approach:
    // H_ij = (∂F/∂xi)^T · d2C_dF2 · (∂F/∂xj)
    //
    // For tet element: F = Ds·Q where Ds = [x0-x3, x1-x3, x2-x3]
    // So: ∂F/∂x0 = Q^T·[1,0,0], ∂F/∂x1 = Q^T·[0,1,0], ∂F/∂x2 = Q^T·[0,0,1]
    //     ∂F/∂x3 = -Q^T·[1,1,1]
    
    const Mat3r& DmInv = _Q;  // Q is DmInv in our notation
    
    // Extract column vectors of DmInv (these are the m1,m2,m3 coefficients)
    const Real Q11 = DmInv(0,0), Q21 = DmInv(1,0), Q31 = DmInv(2,0);
    const Real Q12 = DmInv(0,1), Q22 = DmInv(1,1), Q32 = DmInv(2,1);
    const Real Q13 = DmInv(0,2), Q23 = DmInv(1,2), Q33 = DmInv(2,2);
    
    // Compute Hessian blocks for each vertex pair (i,j)
    for (int i = 0; i < 4; i++) {
        // Get coefficients mi1, mi2, mi3 for vertex i
        Real mi1, mi2, mi3;
        if (i == 0) {
            mi1 = Q11; mi2 = Q12; mi3 = Q13;
        } else if (i == 1) {
            mi1 = Q21; mi2 = Q22; mi3 = Q23;
        } else if (i == 2) {
            mi1 = Q31; mi2 = Q32; mi3 = Q33;
        } else {  // vertex 3
            mi1 = -(Q11 + Q21 + Q31);
            mi2 = -(Q12 + Q22 + Q32);
            mi3 = -(Q13 + Q23 + Q33);
        }
        
        for (int j = 0; j < 4; j++) {
            // Get coefficients mj1, mj2, mj3 for vertex j
            Real mj1, mj2, mj3;
            if (j == 0) {
                mj1 = Q11; mj2 = Q12; mj3 = Q13;
            } else if (j == 1) {
                mj1 = Q21; mj2 = Q22; mj3 = Q23;
            } else if (j == 2) {
                mj1 = Q31; mj2 = Q32; mj3 = Q33;
            } else {  // vertex 3
                mj1 = -(Q11 + Q21 + Q31);
                mj2 = -(Q12 + Q22 + Q32);
                mj3 = -(Q13 + Q23 + Q33);
            }
            
            // Apply Gaia's transformation formula
            // H_ij = (dF/dxi)^T · d2C_dF2 · (dF/dxj)
            Eigen::Matrix<Real, 3, 9> HL;  // Intermediate: (dF/dxi)^T · d2C_dF2
            
            // HL.row(k) = Σ_m (∂F_km/∂xi) · d2C_dF2.row(k*3+m)
            HL.row(0) = d2C_dF2.row(0) * mi1 + d2C_dF2.row(3) * mi2 + d2C_dF2.row(6) * mi3;
            HL.row(1) = d2C_dF2.row(1) * mi1 + d2C_dF2.row(4) * mi2 + d2C_dF2.row(7) * mi3;
            HL.row(2) = d2C_dF2.row(2) * mi1 + d2C_dF2.row(5) * mi2 + d2C_dF2.row(8) * mi3;
            
            // H_ij.col(k) = Σ_m HL(k, m*3:m*3+2) · (∂F_km/∂xj)
            Mat3r& H_block = hess[i * 4 + j];
            H_block.col(0) = HL.col(0) * mj1 + HL.col(3) * mj2 + HL.col(6) * mj3;
            H_block.col(1) = HL.col(1) * mj1 + HL.col(4) * mj2 + HL.col(7) * mj3;
            H_block.col(2) = HL.col(2) * mj1 + HL.col(5) * mj2 + HL.col(8) * mj3;
        }
    }
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

} // namespace Solver