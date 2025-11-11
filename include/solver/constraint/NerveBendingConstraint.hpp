#ifndef __NERVE_BENDING_CONSTRAINT_HPP
#define __NERVE_BENDING_CONSTRAINT_HPP

#include "solver/constraint/Constraint.hpp"

namespace Solver {

/**
 * @brief Nerve bending constraint for 1D rod-like nerve structures.
 *
 * This constraint models the bending resistance of a nerve by constraining
 * the discrete curvature between three consecutive vertices along the nerve.
 * Based on a discrete rod bending formulation:
 *
 *   κ = 2‖e1 × e2‖ / (‖e1‖ ‖e2‖ (‖e1‖ + ‖e2‖))
 *
 * Constraint:
 *   C(p) = κ - κ_rest
 * (curvature magnitude, no signed binormal here)
 */
class NerveBendingConstraint : public Constraint
{
public:
    static constexpr int NUM_POSITIONS   = 3;
    static constexpr int NUM_COORDINATES = 9;

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

    // --- Core XPBD interface ---

    inline void evaluate(Real* C) const override
    {
        Vec3r e1, e2, cross_product;
        Real norm_e1, norm_e2, denominator, curvature_magnitude;

        computeEdgesAndCurvature(
            e1, e2, cross_product,
            norm_e1, norm_e2,
            denominator, curvature_magnitude
        );

        *C = curvature_magnitude - _rest_curvature; // C(q) = κ - κ_rest
    }

    inline void gradient(Real* grad) const override
    {
        // Initialize to zero
        for (int i = 0; i < NUM_COORDINATES; ++i) {
            grad[i] = Real(0);
        }

        Vec3r e1, e2, cross_product;
        Real norm_e1, norm_e2, denominator, curvature_magnitude;

        computeEdgesAndCurvature(
            e1, e2, cross_product,
            norm_e1, norm_e2,
            denominator, curvature_magnitude
        );

        // Degenerate / flat cases: zero gradient is fine & safe.
        const Real eps = Real(1e-12);
        if (curvature_magnitude < eps ||
            norm_e1 < eps || norm_e2 < eps ||
            denominator < eps) {
            return;
        }

        computeGradient(e1, e2, cross_product,
                        norm_e1, norm_e2,
                        curvature_magnitude,
                        grad);
    }

    inline void evaluateWithGradient(Real* C, Real* grad) const override
    {
        evaluate(C);
        gradient(grad);
    }

    inline int numPositions() const override    { return NUM_POSITIONS; }
    inline int numCoordinates() const override  { return NUM_COORDINATES; }

    inline bool isInequality() const override   { return false; }

    // Note: do not use override, because alpha in base class is not virtual
    inline Real alpha() const { return _alpha; }

private:
    Real _rest_curvature;

    /**
     * @brief Compute edges and discrete curvature for the three vertices.
     *
     * e1 = p_j - p_i
     * e2 = p_k - p_j
     * κ = 2‖e1 × e2‖ / (‖e1‖ ‖e2‖ (‖e1‖ + ‖e2‖))
     */
    inline void computeEdgesAndCurvature(
        Vec3r& e1, Vec3r& e2, Vec3r& cross_product,
        Real& norm_e1, Real& norm_e2,
        Real& denominator,
        Real& curvature_magnitude
    ) const
    {
        const auto& p_i = _positions[0];
        const auto& p_j = _positions[1];
        const auto& p_k = _positions[2];

        // Edges
        e1 = Vec3r(p_j.position_ptr[0] - p_i.position_ptr[0],
                   p_j.position_ptr[1] - p_i.position_ptr[1],
                   p_j.position_ptr[2] - p_i.position_ptr[2]);

        e2 = Vec3r(p_k.position_ptr[0] - p_j.position_ptr[0],
                   p_k.position_ptr[1] - p_j.position_ptr[1],
                   p_k.position_ptr[2] - p_j.position_ptr[2]);

        norm_e1 = e1.norm();
        norm_e2 = e2.norm();

        cross_product = e1.cross(e2);

        denominator = norm_e1 * norm_e2 * (norm_e1 + norm_e2);

        const Real eps = Real(1e-12);
        if (denominator < eps) {
            curvature_magnitude = Real(0);
        } else {
            Real A = cross_product.norm(); // = ‖e1 × e2‖
            curvature_magnitude = Real(2) * A / denominator;
        }
    }

    /**
     * @brief Compute analytical gradient of C = κ - κ_rest
     * for κ = 2‖e1 × e2‖ / (‖e1‖ ‖e2‖ (‖e1‖ + ‖e2‖)).
     *
     * This uses chain rule on:
     *   A = ‖e1 × e2‖
     *   D = ‖e1‖ ‖e2‖ (‖e1‖ + ‖e2‖)
     *   κ = 2A / D
     *
     * Verified against finite differences on multiple configurations
     * (including circle-arc and random non-degenerate triples).
     */
    inline void computeGradient(
        const Vec3r& e1,
        const Vec3r& e2,
        const Vec3r& cross_product,
        Real norm_e1,
        Real norm_e2,
        Real curvature_magnitude,
        Real* grad
    ) const
    {
        const Real eps = Real(1e-12);

        Real A = cross_product.norm();
        if (A < eps || norm_e1 < eps || norm_e2 < eps) {
            return;
        }

        Real L1 = norm_e1;
        Real L2 = norm_e2;
        Real D  = L1 * L2 * (L1 + L2);
        if (D < eps) {
            return;
        }

        // Unit vectors
        Vec3r n  = cross_product / A; // binormal
        Vec3r u1 = e1 / L1;
        Vec3r u2 = e2 / L2;

        // D = L1^2 * L2 + L1 * L2^2
        // ∂D/∂L1 = L2(2L1 + L2)
        // ∂D/∂L2 = L1(L1 + 2L2)
        Real coef1 = L2 * (Real(2)*L1 + L2);
        Real coef2 = L1 * (L1 + Real(2)*L2);

        // ---------- vertex i (p_i) ----------
        // e1 = pj - pi, e2 independent of pi
        // A-part: dA/dpi = - (e2 × n)
        Vec3r dA_pi  = - (e2.cross(n));
        // L1-part: dL1/dpi = -u1; L2 independent
        Vec3r dL1_pi = -u1;
        Vec3r dL2_pi = Vec3r(Real(0), Real(0), Real(0));
        Vec3r dD_pi  = coef1 * dL1_pi + coef2 * dL2_pi;

        // ∂κ/∂p = 2/D * dA - 2A/D^2 * dD
        Vec3r g_pi = (Real(2) / D) * dA_pi
                   - (Real(2) * A / (D * D)) * dD_pi;

        grad[0] = g_pi[0];
        grad[1] = g_pi[1];
        grad[2] = g_pi[2];

        // ---------- vertex k (p_k) ----------
        // e2 = pk - pj, e1 independent of pk
        // A-part: dA/dpk = (n × e1)
        Vec3r dA_pk  = n.cross(e1);
        Vec3r dL1_pk = Vec3r(Real(0), Real(0), Real(0));
        Vec3r dL2_pk = u2;
        Vec3r dD_pk  = coef1 * dL1_pk + coef2 * dL2_pk;

        Vec3r g_pk = (Real(2) / D) * dA_pk
                   - (Real(2) * A / (D * D)) * dD_pk;

        grad[6] = g_pk[0];
        grad[7] = g_pk[1];
        grad[8] = g_pk[2];

        // ---------- vertex j (p_j, middle) ----------
        // affects both e1 and e2
        // A-part: dA/dpj = (e2 × n) - (n × e1)
        Vec3r dA_pj  = e2.cross(n) - n.cross(e1);
        // L1-part: dL1/dpj =  u1
        // L2-part: dL2/dpj = -u2
        Vec3r dL1_pj = u1;
        Vec3r dL2_pj = -u2;
        Vec3r dD_pj  = coef1 * dL1_pj + coef2 * dL2_pj;

        Vec3r g_pj = (Real(2) / D) * dA_pj
                   - (Real(2) * A / (D * D)) * dD_pj;

        grad[3] = g_pj[0];
        grad[4] = g_pj[1];
        grad[5] = g_pj[2];
    }
};

} // namespace Solver

#endif
