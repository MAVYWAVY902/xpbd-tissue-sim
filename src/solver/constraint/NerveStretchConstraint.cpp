
// #include "solver/constraint/NerveStretchConstraint.hpp"
// #include <Eigen/Dense> 

// namespace Solver {

// // C(q) = |p_j - p_i| - L0
// inline void NerveStretchConstraint::evaluate(Real* C) const
// {
//     // 读出两个点的坐标
//     Eigen::Map<const Eigen::Vector3d> pi(_p_i);
//     Eigen::Map<const Eigen::Vector3d> pj(_p_j);

//     Eigen::Vector3d diff = pj - pi;
//     Real dist = diff.norm();

//     // 防止两个点完全重合导致 NAN
//     if (dist < 1e-9)
//     {
//         *C = -_rest_length;   // 这时候其实就是“比想要的短了rest_length”
//         return;
//     }

//     *C = dist - _rest_length;
// }

// // grad = dC/dq = [ -n,  +n ]
// inline void NerveStretchConstraint::gradient(Real* grad) const
// {
//     // grad 要写满 6 个数：前 3 个是点 i 的，后 3 个是点 j 的
//     Eigen::Map<const Eigen::Vector3d> pi(_p_i);
//     Eigen::Map<const Eigen::Vector3d> pj(_p_j);

//     Eigen::Vector3d diff = pj - pi;
//     Real dist = diff.norm();

//     // 先清零，防止下面只写了一部分
//     for (int k = 0; k < NUM_COORDINATES; ++k)
//         grad[k] = 0.0;

//     if (dist < 1e-9)
//     {
//         // 太近了，就给个 0 梯度，避免除 0
//         return;
//     }

//     Eigen::Vector3d n = diff / dist;   // 单位方向：i -> j

//     // dC/dp_i = -n
//     grad[0] = -n[0];
//     grad[1] = -n[1];
//     grad[2] = -n[2];

//     // dC/dp_j = +n
//     grad[3] =  n[0];
//     grad[4] =  n[1];
//     grad[5] =  n[2];
// }

// inline std::vector<Constraint::PositionReference>
// NerveStretchConstraint::positions() const
// {
//     // 跟 AttachmentConstraint 的写法一样：
//     // PositionReference(index, pointer, weight)
//     std::vector<Constraint::PositionReference> pos;
//     pos.reserve(2);
//     pos.emplace_back(_v_i, _p_i, _w_i);
//     pos.emplace_back(_v_j, _p_j, _w_j);
//     return pos;
// }

// } // namespace Solver


// NerveStretchConstraint has header-only implementation.
