#ifndef __NERVE_BENDING_CONSTRAINT_HPP
#define __NERVE_BENDING_CONSTRAINT_HPP

#include "solver/constraint/Constraint.hpp"

namespace Solver {

/**
 * @brief Nerve bending constraint for 1D rod-like nerve structures.
 * 
 * This constraint models the bending resistance of a nerve by constraining
 * the discrete curvature between three consecutive vertices along the nerve.
 * Based on the DisMech discrete rod bending formulation.
 * 
 * The constraint is defined as:
 * C(p) = |κ| - κ_rest
 * where κ is the discrete curvature between three consecutive points.
 */
class NerveBendingConstraint : public Constraint
{
public:
    // Three points, each with 3 coordinates
    static constexpr int NUM_POSITIONS   = 3;
    static constexpr int NUM_COORDINATES = 9;

    // Constructor sequence should be the same as the sequence of emplace_back in cpp!
    // v_i, p_i, m_i, v_j, p_j, m_j, v_k, p_k, m_k, rest_curvature, alpha

    /**
     * @brief Constructor for nerve bending constraint.
     * 
     * @param v_i Index of first vertex
     * @param p_i Pointer to first vertex position
     * @param m_i Mass of first vertex
     * @param v_j Index of second vertex (middle vertex)
     * @param p_j Pointer to second vertex position
     * @param m_j Mass of second vertex
     * @param v_k Index of third vertex
     * @param p_k Pointer to third vertex position
     * @param m_k Mass of third vertex
     * @param rest_curvature Rest curvature magnitude
     * @param alpha Compliance parameter (default 0.0 for hard constraint)
     */
    NerveBendingConstraint(int v_i, Real* p_i, Real m_i,
                          int v_j, Real* p_j, Real m_j,
                          int v_k, Real* p_k, Real m_k,
                          Real rest_curvature,
                          Real alpha)
    : Constraint(
        std::vector<PositionReference>{
            PositionReference{v_i, p_i, m_i},
            PositionReference{v_j, p_j, m_j},
            PositionReference{v_k, p_k, m_k}
        },
        alpha
      )
    , _rest_curvature(rest_curvature)
    {}

    // Core XPBD interface methods - these four functions are required by XPBD
    inline void evaluate(Real* C) const override
    {
        Vec3r e1, e2, cross_product;
        Real norm_e1, norm_e2, denominator, curvature_magnitude;
        
        computeEdgesAndCurvature(e1, e2, cross_product, norm_e1, norm_e2, 
                               denominator, curvature_magnitude);
        
        *C = curvature_magnitude - _rest_curvature;  // C(q) = |κ| - κ_rest
    }

    inline void gradient(Real* grad) const override
    {
        Vec3r e1, e2, cross_product;
        Real norm_e1, norm_e2, denominator, curvature_magnitude;
        
        computeEdgesAndCurvature(e1, e2, cross_product, norm_e1, norm_e2, 
                               denominator, curvature_magnitude);
        
        // Clear gradient array first
        for (int i = 0; i < NUM_COORDINATES; ++i) {
            grad[i] = 0.0;
        }
        
        // If curvature is essentially zero, gradient is zero (prevent division by zero)
        if (curvature_magnitude < Real(1e-12)) {
            return;
        }
        
        // Compute gradient components
        computeGradient(e1, e2, cross_product, norm_e1, norm_e2, 
                       curvature_magnitude, grad);
    }

    inline void evaluateWithGradient(Real* C, Real* grad) const override
    {
        evaluate(C);
        gradient(grad);
    }

    inline int numPositions() const override  { return NUM_POSITIONS; }
    inline int numCoordinates() const override { return NUM_COORDINATES; }

    inline bool isInequality() const override { return false; }

    // Note: do not use override, because alpha in base class is not virtual
    inline Real alpha() const { return _alpha; }

private:
    Real _rest_curvature;

    /**
     * @brief Compute edges and discrete curvature for the three vertices.
     */
    inline void computeEdgesAndCurvature(Vec3r& e1, Vec3r& e2, Vec3r& cross_product,
                                       Real& norm_e1, Real& norm_e2, Real& denominator,
                                       Real& curvature_magnitude) const
    {
        const auto& p_i = _positions[0];
        const auto& p_j = _positions[1];
        const auto& p_k = _positions[2];

        // Compute edges
        e1 = Vec3r(p_j.position_ptr[0] - p_i.position_ptr[0],
                   p_j.position_ptr[1] - p_i.position_ptr[1],
                   p_j.position_ptr[2] - p_i.position_ptr[2]);
                   
        e2 = Vec3r(p_k.position_ptr[0] - p_j.position_ptr[0],
                   p_k.position_ptr[1] - p_j.position_ptr[1],
                   p_k.position_ptr[2] - p_j.position_ptr[2]);

        // Compute edge lengths
        norm_e1 = e1.norm();
        norm_e2 = e2.norm();

        // Compute cross product
        cross_product = e1.cross(e2);
        
        // Compute denominator for discrete curvature formula
        denominator = norm_e1 * norm_e2 * (norm_e1 + norm_e2);
        
        // Compute curvature magnitude
        if (denominator < Real(1e-12)) {
            curvature_magnitude = 0.0;
        } else {
            curvature_magnitude = 2.0 * cross_product.norm() / denominator;
        }
    }

    /**
     * @brief Compute the gradient of the constraint.
     * 
     * This is a simplified gradient computation following the same pattern as NerveStretchConstraint.
     * For production use, you may want to implement the full analytical derivative.
     */
    inline void computeGradient(const Vec3r& e1, const Vec3r& e2, const Vec3r& cross_product,
                              Real norm_e1, Real norm_e2, Real curvature_magnitude,
                              Real* grad) const
    {
        // Safety guards to avoid division by zero / producing inf or NaN gradients.
        const Real eps = Real(1e-12);
        Real norm_cross = cross_product.norm();
        if (norm_cross < eps) return;

        // Recompute denominator and check
        Real denominator = norm_e1 * norm_e2 * (norm_e1 + norm_e2);
        if (denominator < eps) return;

        Vec3r cross_normalized = cross_product / norm_cross;

        // Simplified gradient computation (approximation, following NerveStretchConstraint pattern)
        Real factor = Real(2.0) / denominator;

        // ∂C/∂p_i (affects e1)
        Vec3r grad_pi = -factor * cross_normalized;
        grad[0] = grad_pi[0];
        grad[1] = grad_pi[1];
        grad[2] = grad_pi[2];

        // ∂C/∂p_j (affects both e1 and e2, middle vertex)
        Vec3r grad_pj = factor * cross_normalized;
        grad[3] = grad_pj[0];
        grad[4] = grad_pj[1];
        grad[5] = grad_pj[2];

        // ∂C/∂p_k (affects e2)
        Vec3r grad_pk = factor * cross_normalized;
        grad[6] = grad_pk[0];
        grad[7] = grad_pk[1];
        grad[8] = grad_pk[2];
    }
};

} // namespace Solver

#endif