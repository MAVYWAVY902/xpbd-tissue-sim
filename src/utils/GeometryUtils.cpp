#include "utils/GeometryUtils.hpp"

namespace GeometryUtils
{

std::tuple<double,double,double> barycentricCoords(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c)
{
    // from https://ceng2.ktu.edu.tr/~cakir/files/grafikler/Texture_Mapping.pdf
    const Eigen::Vector3d v0 = b - a;
    const Eigen::Vector3d v1 = c - a;
    const Eigen::Vector3d v2 = p - a;
    const double d00 = v0.dot(v0);
    const double d01 = v0.dot(v1);
    const double d11 = v1.dot(v1);
    const double d20 = v2.dot(v0);
    const double d21 = v2.dot(v1);
    const double denom = d00*d11 - d01*d01;
    const double v = (d11*d20 - d01*d21) / denom;
    const double w = (d00*d21 - d01*d20) / denom;
    const double u = 1 - v - w;

    return std::tuple<double, double, double>(u, v, w);
}

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

Eigen::Vector3d rotateVectorByQuat(const Eigen::Vector3d& v, const Eigen::Vector4d& quat)
{
    // from https://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-quaternion
    const Eigen::Vector3d& u = quat(Eigen::seq(0,2));
    return 2*u.dot(v)*u + (2*quat[3]*quat[3] - 1)*v + 2*quat[3]*u.cross(v);
}

Eigen::Vector4d inverseQuat(const Eigen::Vector4d& quat)
{
    return Eigen::Vector4d({-quat[0], -quat[1], -quat[2], quat[3]}) / (quat.squaredNorm());
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