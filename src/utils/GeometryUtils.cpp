#include "utils/GeometryUtils.hpp"

namespace GeometryUtils
{

Eigen::Matrix3d quatToMat(const Eigen::Vector4d& quat)
{
    Eigen::Matrix3d mat;
    mat(0,0) = 1 - 2*quat[1]*quat[1] - 2*quat[2]*quat[2];
    mat(0,1) = 2*quat[0]*quat[1] - 2*quat[3]*quat[2];
    mat(0,2) = 2*quat[0]*quat[2] + 2*quat[3]*quat[1];
    mat(1,0) = 2*quat[0]*quat[1] + 2*quat[3]*quat[2];
    mat(1,1) = 1 - 2*quat[0]*quat[0] - 2*quat[2]*quat[2];
    mat(1,2) = 2*quat[1]*quat[2] - 2*quat[3]*quat[0];
    mat(2,0) = 2*quat[0]*quat[2] - 2*quat[3]*quat[1];
    mat(2,1) = 2*quat[1]*quat[2] + 2*quat[3]*quat[0];
    mat(2,2) = 1 - 2*quat[0]*quat[0] - 2*quat[1]*quat[1];

    return mat;
}

Eigen::Vector4d eulXYZ2Quat(const double x, const double y, const double z)
{
    const double q1 = std::sin(0.5*x)*std::cos(0.5*y)*std::cos(0.5*z) - std::cos(0.5*x)*std::sin(0.5*y)*std::sin(0.5*z);
    const double q2 = std::cos(0.5*x)*std::sin(0.5*y)*std::cos(0.5*z) + std::sin(0.5*x)*std::cos(0.5*y)*std::sin(0.5*z);
    const double q3 = std::cos(0.5*x)*std::cos(0.5*y)*std::sin(0.5*z) - std::sin(0.5*x)*std::sin(0.5*y)*std::cos(0.5*z);
    const double w = std::cos(0.5*x)*std::cos(0.5*y)*std::cos(0.5*z) + std::sin(0.5*x)*std::sin(0.5*y)*std::sin(0.5*z);

    return Eigen::Vector4d({q1, q2, q3, w});
}

} // namespace GeometryUtils