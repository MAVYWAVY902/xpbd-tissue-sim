#ifndef __GEOMETRY_UTILS_HPP
#define __GEOMETRY_UTILS_HPP

#include <Eigen/Dense>

namespace GeometryUtils
{

std::tuple<double,double,double> barycentricCoords(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c);
Eigen::Matrix3d quatToMat(const Eigen::Vector4d& quat);
Eigen::Vector4d quatMult(const Eigen::Vector4d& a, const Eigen::Vector4d& b);
Eigen::Vector3d rotateVectorByQuat(const Eigen::Vector3d& v, const Eigen::Vector4d& quat);
Eigen::Vector4d inverseQuat(const Eigen::Vector4d& quat);
Eigen::Vector4d eulXYZ2Quat(const double x, const double y, const double z);

Eigen::Matrix3d Rz(double theta);
Eigen::Matrix3d Ry(double theta);
Eigen::Matrix3d Rx(double theta);

Eigen::Vector3d Vee_SO3(const Eigen::Matrix3d& mat);
Eigen::Matrix3d Bracket_so3(const Eigen::Vector3d& vec);
Eigen::Vector<double,6> Vee_SE3(const Eigen::Matrix4d& mat);
Eigen::Matrix4d Bracket_se3(const Eigen::Vector<double,6>& vec);

}

#endif // __GEOMETRY_UTILS_HPP