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
#ifndef SOLVER_NERVE_STRETCH_CONSTRAINT_HPP
#define SOLVER_NERVE_STRETCH_CONSTRAINT_HPP

#include "solver/constraint/Constraint.hpp"
#include <Eigen/Dense>

namespace Solver
{

// 一个最简单的“2 点距离 = restLen”约束
class NerveStretchConstraint : public Constraint
{
public:
    // 我们只有两个位置
    static constexpr int NUM_POSITIONS   = 2;
    // 只有一个标量约束：‖p0 - p1‖ - L0 = 0
    static constexpr int NUM_COORDINATES = 1;

    // 构造函数的风格，照你的 AttachmentConstraint
    NerveStretchConstraint(int i_idx, Real* i_ptr, Real i_invMass,
                           int j_idx, Real* j_ptr, Real j_invMass,
                           Real restLen);

    // 下面这三个都是你基类里要求实现的纯虚函数
    int  numPositions() const override;
    int  numCoordinates() const override;
    bool isInequality() const override;

    // 关键：算 C 和 ∂C/∂x
    // C 大小 = 1
    // grad 大小 = numPositions * 3 = 2 * 3 = 6
    void evaluateWithGradient(Real* C, Real* grad) const override;

private:
    Real _restLen;
};

} // namespace Solver

#endif // SOLVER_NERVE_STRETCH_CONSTRAINT_HPP
