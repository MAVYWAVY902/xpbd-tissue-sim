#ifndef __GEOMETRY_UTILS_HPP
#define __GEOMETRY_UTILS_HPP

#include <Eigen/Dense>

namespace GeometryUtils
{

Eigen::Matrix3d quatToMat(const Eigen::Vector4d& quat);
Eigen::Vector4d eulXYZ2Quat(const double x, const double y, const double z);

}

#endif // __GEOMETRY_UTILS_HPP