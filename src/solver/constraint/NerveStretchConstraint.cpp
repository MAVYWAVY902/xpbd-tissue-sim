// #include "solver/constraint/NerveStretchConstraint.hpp"

// namespace Solver {
// // 全在 hpp 里写完了，这里可以先空着
// }


// #include "solver/constraint/NerveStretchConstraint.hpp"

// namespace Solver {

// NerveStretchConstraint::NerveStretchConstraint(
//     int i_idx, Real* i_ptr, Real mi,
//     int j_idx, Real* j_ptr, Real mj,
//     Real rest_len,
//     Real alpha)
// : Constraint(
//       std::vector<PositionReference>{
//           PositionReference(i_idx, i_ptr, mi),
//           PositionReference(j_idx, j_ptr, mj)
//       },
//       alpha),
//   _rest_len(rest_len)
// {
// }

// bool NerveStretchConstraint::isInequality() const
// {
//     return false;
// }

// void NerveStretchConstraint::evaluate(Real* C) const
// {
//     Eigen::Map<const Vec3r> pi(_positions[0].position_ptr);
//     Eigen::Map<const Vec3r> pj(_positions[1].position_ptr);
//     *C = (pj - pi).norm() - _rest_len;
// }

// void NerveStretchConstraint::gradient(Real* grad) const
// {
//     Eigen::Map<const Vec3r> pi(_positions[0].position_ptr);
//     Eigen::Map<const Vec3r> pj(_positions[1].position_ptr);
//     Vec3r d = pj - pi;
//     Real n = d.norm();
//     if (n < Real(1e-9)) {
//         for (int k = 0; k < 6; ++k) {
//             grad[k] = 0;
//         }
//         return;
//     }

//     Vec3r g = d / n;

//     // wrt point i
//     grad[0] = -g[0];
//     grad[1] = -g[1];
//     grad[2] = -g[2];

//     // wrt point j
//     grad[3] =  g[0];
//     grad[4] =  g[1];
//     grad[5] =  g[2];
// }

// void NerveStretchConstraint::evaluateWithGradient(Real* C, Real* grad) const
// {
//     evaluate(C);
//     gradient(grad);
// }

// int NerveStretchConstraint::numPositions() const
// {
//     return 2;  // i, j
// }

// int NerveStretchConstraint::numCoordinates() const
// {
//     return 6;  // 2 * 3
// }

// } // namespace Solver
#include "solver/constraint/NerveStretchConstraint.hpp"
#include <array> 
namespace Solver {

NerveStretchConstraint::NerveStretchConstraint(
    int   i_idx, Real* i_ptr, Real i_invMass,
    int   j_idx, Real* j_ptr, Real j_invMass,
    Real  restLen)
    // ✅ 注意这里：基类要的是“一个 position 向量 + alpha”
    : Constraint(
        std::vector<PositionReference>{
            PositionReference{i_idx, i_ptr, i_invMass},
            PositionReference{j_idx, j_ptr, j_invMass}
        },
        /*alpha=*/0.0
      )
    , _restLen(restLen)
{
    // ctor 里其实不用再做别的了，因为上面已经把位置都交给基类了
}

int NerveStretchConstraint::numPositions() const
{
    return NUM_POSITIONS;
}

int NerveStretchConstraint::numCoordinates() const
{
    return NUM_COORDINATES;
}

bool NerveStretchConstraint::isInequality() const
{
    return false;
}

void NerveStretchConstraint::evaluateWithGradient(Real* C, Real* grad) const
{
    // 从基类里把两个点拿出来
    // 基类里我们刚才传了 2 个 PositionReference，所以这里就是 0 和 1
    const auto& pr0 = _positions[0];
    const auto& pr1 = _positions[1];

    Eigen::Map<const Vec3r> p0(pr0.position_ptr);
    Eigen::Map<const Vec3r> p1(pr1.position_ptr);

    Vec3r d  = p0 - p1;
    Real len = d.norm();

    *C = len - _restLen;

    if (len < Real(1e-9))
    {
        // 长度太短就给个 0 梯度
        for (int k = 0; k < 6; ++k)
            grad[k] = Real(0);
        return;
    }

    Vec3r n = d / len;

    // 对 p0 的梯度
    grad[0] = n[0];
    grad[1] = n[1];
    grad[2] = n[2];

    // 对 p1 的梯度
    grad[3] = -n[0];
    grad[4] = -n[1];
    grad[5] = -n[2];
}

} // namespace Solver