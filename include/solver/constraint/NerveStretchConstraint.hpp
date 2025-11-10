// #ifndef __NERVE_STRETCH_CONSTRAINT_HPP
// #define __NERVE_STRETCH_CONSTRAINT_HPP

// #include "solver/constraint/Constraint.hpp"

// namespace Solver {

// class NerveStretchConstraint : public Constraint
// {
// public:
//     NerveStretchConstraint(int i_idx, Real* i_ptr, Real mi,
//                            int j_idx, Real* j_ptr, Real mj,
//                            Real rest_len,
//                            Real alpha = 0.0);

//     bool isInequality() const override;
//     void evaluate(Real* C) const override;
//     void gradient(Real* grad) const override;
//     void evaluateWithGradient(Real* C, Real* grad) const override;
//     int numPositions() const override;
//     int numCoordinates() const override;

// private:
//     Real _rest_len;
// };

// } // namespace Solver

// #endif


// include/solver/constraint/NerveStretchConstraint.hpp
#ifndef __NERVE_STRETCH_CONSTRAINT_HPP
#define __NERVE_STRETCH_CONSTRAINT_HPP

#include "solver/constraint/Constraint.hpp"

namespace Solver {

class NerveStretchConstraint : public Constraint
{
public:
    // two points，each points 3x1
    static constexpr int NUM_POSITIONS   = 2;
    static constexpr int NUM_COORDINATES = 6;

    // ctor sequence should be the same as the sequence of emplace_back in cpp!
    // v_i, p_i, m_i, v_j, p_j, m_j, rest_len, alpha
    NerveStretchConstraint(int v_i, Real* p_i, Real m_i,
                           int v_j, Real* p_j, Real m_j,
                           Real rest_length,
                           Real alpha)
    : Constraint(
        // put two points into base class
        std::vector<PositionReference>{
            PositionReference{v_i, p_i, m_i},
            PositionReference{v_j, p_j, m_j}
        },
        alpha   // _alpha in base class
      )
    , _rest_length(rest_length)
    {}

    // XPBD need these four functions:
    inline void evaluate(Real* C) const override
    {
        const auto& p_i = _positions[0];
        const auto& p_j = _positions[1];

        // current length
        Real dx = p_i.position_ptr[0] - p_j.position_ptr[0];
        Real dy = p_i.position_ptr[1] - p_j.position_ptr[1];
        Real dz = p_i.position_ptr[2] - p_j.position_ptr[2];
        Real dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        *C = dist - _rest_length;     // C(q) = |pi - pj| - L0
    }

    inline void gradient(Real* grad) const override
    {
        const auto& p_i = _positions[0];
        const auto& p_j = _positions[1];

        Real dx = p_i.position_ptr[0] - p_j.position_ptr[0];
        Real dy = p_i.position_ptr[1] - p_j.position_ptr[1];
        Real dz = p_i.position_ptr[2] - p_j.position_ptr[2];
        Real dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        // in case ==0
        if (dist < Real(1e-12)) {
            // i gradient
            grad[0] = grad[1] = grad[2] = 0;
            // j gradient
            grad[3] = grad[4] = grad[5] = 0;
            return;
        }

        Real invd = Real(1.0) / dist;

        // ∂C/∂p_i
        grad[0] = dx * invd;
        grad[1] = dy * invd;
        grad[2] = dz * invd;

        // ∂C/∂p_j = - ∂C/∂p_i
        grad[3] = -grad[0];
        grad[4] = -grad[1];
        grad[5] = -grad[2];
    }

    inline void evaluateWithGradient(Real* C, Real* grad) const override
    {
        evaluate(C);
        gradient(grad);
    }

    inline int numPositions() const override  { return NUM_POSITIONS; }
    inline int numCoordinates() const override { return NUM_COORDINATES; }

    inline bool isInequality() const override { return false; }

    // Note：do not use override，because alpha in base class is not virtual
    inline Real alpha() const { return _alpha; }

private:
    Real _rest_length;
};

} // namespace Solver

#endif
