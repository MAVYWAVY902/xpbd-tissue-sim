#ifndef __LINEAR_SOLVER_HPP
#define __LINEAR_SOLVER_HPP

#include "common/types.hpp"
#include <Eigen/Dense>
#include <cmath>

namespace Utils
{

// Constants from Gaia's CuMatrix
constexpr Real CMP_EPSILON = 1e-5;
constexpr Real CMP_EPSILON2 = CMP_EPSILON * CMP_EPSILON;

/** 3x3 Matrix Solver using Direct Inversion (Cramer's Rule)
 * 
 * This is the EXACT implementation from Gaia's CuMatrix::solve3x3_psd_stable.
 * Solves the system m * x = b for a 3x3 matrix using matrix inversion.
 * 
 * Matrix storage: column-major
 * m = [m[0] m[3] m[6]]     [a11 a12 a13]
 *     [m[1] m[4] m[7]]  =  [a21 a22 a23]
 *     [m[2] m[5] m[8]]     [a31 a32 a33]
 * 
 * @param m - 3x3 matrix (column-major storage)
 * @param b - right-hand side vector
 * @param out - (OUTPUT) solution vector
 * @return true if solve succeeded, false if matrix is singular (falls back to out=b)
 */
inline bool solve3x3PSD(const Real* m, const Real* b, Real* out)
{
    // Extract matrix elements (column-major)
    const Real a11 = m[0]; const Real a12 = m[3]; const Real a13 = m[6];
    const Real a21 = m[1]; const Real a22 = m[4]; const Real a23 = m[7];
    const Real a31 = m[2]; const Real a32 = m[5]; const Real a33 = m[8];

    // Compute cofactors for matrix inversion
    const Real i11 = a33 * a22 - a32 * a23;
    const Real i12 = -(a33 * a12 - a32 * a13);
    const Real i13 = a23 * a12 - a22 * a13;

    // Compute determinant
    const Real det = (a11 * i11 + a21 * i12 + a31 * i13);

    // Check if matrix is singular (with relative tolerance)
    // This prevents numerical issues for ill-conditioned matrices
    if (std::abs(det) < CMP_EPSILON * (std::abs(a11 * i11) + std::abs(a21 * i12) + std::abs(a31 * i13)))
    {
        // Matrix is singular - fall back to gradient descent (out = b)
        out[0] = b[0];
        out[1] = b[1];
        out[2] = b[2];
        return false;
    }

    const Real deti = 1.0 / det;

    // Compute remaining cofactors
    const Real i21 = -(a33 * a21 - a31 * a23);
    const Real i22 = a33 * a11 - a31 * a13;
    const Real i23 = -(a23 * a11 - a21 * a13);

    const Real i31 = a32 * a21 - a31 * a22;
    const Real i32 = -(a32 * a11 - a31 * a12);
    const Real i33 = a22 * a11 - a21 * a12;

    // Solve: out = inv(m) * b = (1/det) * cofactor^T * b
    out[0] = deti * (i11 * b[0] + i12 * b[1] + i13 * b[2]);
    out[1] = deti * (i21 * b[0] + i22 * b[1] + i23 * b[2]);
    out[2] = deti * (i31 * b[0] + i32 * b[1] + i33 * b[2]);

    return true;
}

/** Alternative using Eigen (more stable, slightly slower)
 * 
 * This version uses Eigen's LDLT decomposition which is more robust
 * for near-singular matrices.
 */
inline bool solve3x3PSD_Eigen(const Real* H_data, const Real* b_data, Real* x_data)
{
    // Map to Eigen matrices
    Eigen::Map<const Mat3r> H(H_data);
    Eigen::Map<const Vec3r> b(b_data);
    Eigen::Map<Vec3r> x(x_data);
    
    // Use LDLT decomposition (robust for symmetric matrices)
    Eigen::LDLT<Mat3r> ldlt(H);
    
    if (ldlt.info() != Eigen::Success) {
        return false;
    }
    
    // Check if matrix is positive definite
    Eigen::Vector3d D = ldlt.vectorD();
    if ((D.array() < 1e-12).any()) {
        return false;
    }
    
    x = ldlt.solve(b);
    return true;
}

/** Solve 3x3 system with automatic fallback (alias for convenience)
 * 
 * Note: solve3x3PSD already includes automatic fallback to gradient descent,
 * so this is just an alias for clarity.
 * 
 * @param H - 3x3 Hessian matrix
 * @param b - force vector
 * @param x - (OUTPUT) solution
 * @param use_eigen - if true, use Eigen's LDLT (more stable), else use Gaia's method
 * @return true if Newton solve succeeded, false if fell back to gradient descent
 */
inline bool solve3x3PDSWithFallback(const Real* H, const Real* b, Real* x, bool use_eigen = false)
{
    if (use_eigen) {
        return solve3x3PSD_Eigen(H, b, x);
    } else {
        return solve3x3PSD(H, b, x);
    }
}

} // namespace Utils

#endif // __LINEAR_SOLVER_HPP
