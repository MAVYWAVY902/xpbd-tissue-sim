#ifndef __GEOMETRY_UTILS_HPP
#define __GEOMETRY_UTILS_HPP

#include <Eigen/Dense>

namespace GeometryUtils
{

std::tuple<double,double,double> barycentricCoords(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c);
Eigen::Matrix3d quatToMat(const Eigen::Vector4d& quat);
Eigen::Vector3d rotateVectorByQuat(const Eigen::Vector3d& v, const Eigen::Vector4d& quat);
Eigen::Vector4d inverseQuat(const Eigen::Vector4d& quat);
Eigen::Vector4d eulXYZ2Quat(const double x, const double y, const double z);

}

#endif // __GEOMETRY_UTILS_HPP