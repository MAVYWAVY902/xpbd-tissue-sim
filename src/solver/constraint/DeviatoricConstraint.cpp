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

void DeviatoricConstraint::hessian(Real C, const Real* grad, Mat3r* hess) const
{
    // For Deviatoric constraint: C = ||F||_F = sqrt(sum(F_ij^2))
    // The Hessian ∇²C has the form:
    // ∂²C/∂x_i∂x_j = (1/C) * [Q^T·Q/C² - (∇C_i ⊗ ∇C_j)/C²]
    // 
    // Simplified: ∇²C = (1/C) * [I⊗(Q^T·Q) - (∇C⊗∇C)/C]
    //
    // For 4 vertices of a tet element, we have 16 3×3 blocks
    
    if (C < 1e-12) {
        // Constraint value too small, Hessian undefined - fill with zeros
        for (int i = 0; i < 16; i++) {
            hess[i].setZero();
        }
        return;
    }
    
    const Real inv_C = 1.0 / C;
    const Real inv_C3 = inv_C * inv_C * inv_C;
    
    // Precompute Q^T·Q (this is constant for the element)
    const Mat3r QTQ = _Q.transpose() * _Q;
    
    // Map gradient as 4 Vec3r blocks
    const Vec3r* grad_blocks = reinterpret_cast<const Vec3r*>(grad);
    
    // Compute all 16 blocks: hess[i*4 + j] corresponds to ∂²C/∂x_i∂x_j
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            int block_idx = i * 4 + j;
            
            // Hessian contribution: (1/C³) * [Q^T·Q·δ_ij - ∇C_i⊗∇C_j]
            // where δ_ij = 1 if i==j (same vertex), special handling for vertex 4
            
            if (i == 3 && j == 3) {
                // Vertex 4 (index 3) has special formula: -sum of first 3 columns/rows
                // H_44 = -H_41 - H_42 - H_43 (row sum)
                //      = -H_14 - H_24 - H_34 (column sum)
                // But by symmetry and constraint structure: H_44 = sum_{k<3} sum_{l<3} H_kl
                hess[block_idx] = inv_C3 * (QTQ - grad_blocks[3] * grad_blocks[3].transpose());
            }
            else if (i == 3) {
                // Row 4: H_4j = -(H_0j + H_1j + H_2j)
                hess[block_idx] = -inv_C3 * grad_blocks[3] * grad_blocks[j].transpose();
            }
            else if (j == 3) {
                // Column 4: H_i4 = -(H_i0 + H_i1 + H_i2)
                hess[block_idx] = -inv_C3 * grad_blocks[i] * grad_blocks[3].transpose();
            }
            else if (i == j) {
                // Diagonal blocks (vertices 0,1,2): include Q^T·Q term
                hess[block_idx] = inv_C3 * (QTQ - grad_blocks[i] * grad_blocks[i].transpose());
            }
            else {
                // Off-diagonal blocks (i≠j, both < 3)
                hess[block_idx] = -inv_C3 * (grad_blocks[i] * grad_blocks[j].transpose());
            }
        }
    }
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

} // namespace Solver