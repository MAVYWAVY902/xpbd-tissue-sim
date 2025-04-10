#include "utils/GeometryUtils.hpp"

namespace GeometryUtils
{

std::tuple<Real,Real,Real> barycentricCoords(const Vec3r& p, const Vec3r& a, const Vec3r& b, const Vec3r& c)
{
    // from https://ceng2.ktu.edu.tr/~cakir/files/grafikler/Texture_Mapping.pdf
    const Vec3r v0 = b - a;
    const Vec3r v1 = c - a;
    const Vec3r v2 = p - a;
    const Real d00 = v0.dot(v0);
    const Real d01 = v0.dot(v1);
    const Real d11 = v1.dot(v1);
    const Real d20 = v2.dot(v0);
    const Real d21 = v2.dot(v1);
    const Real denom = d00*d11 - d01*d01;
    const Real v = (d11*d20 - d01*d21) / denom;
    const Real w = (d00*d21 - d01*d20) / denom;
    const Real u = 1 - v - w;

    return std::tuple<Real, Real, Real>(u, v, w);
}

Mat3r quatToMat(const Vec4r& quat)
{
    Mat3r mat;
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

Vec4r quatMult(const Vec4r& a, const Vec4r& b)
{
    Vec4r res;
    res[0] = a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1];
    res[1] = a[3]*b[1] + a[1]*b[3] + a[2]*b[0] - a[0]*b[2];
    res[2] = a[3]*b[2] + a[2]*b[3] + a[0]*b[1] - a[1]*b[0];
    res[3] = a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2];

    return res;
}

Vec3r rotateVectorByQuat(const Vec3r& v, const Vec4r& quat)
{
    // from https://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-quaternion
    const Vec3r& u = quat(Eigen::seq(0,2));
    return 2*u.dot(v)*u + (2*quat[3]*quat[3] - 1)*v + 2*quat[3]*u.cross(v);
}

Vec4r inverseQuat(const Vec4r& quat)
{
    return Vec4r({-quat[0], -quat[1], -quat[2], quat[3]}) / (quat.squaredNorm());
}

Vec4r eulXYZ2Quat(const Real x, const Real y, const Real z)
{
    const Real q1 = std::sin(0.5*x)*std::cos(0.5*y)*std::cos(0.5*z) - std::cos(0.5*x)*std::sin(0.5*y)*std::sin(0.5*z);
    const Real q2 = std::cos(0.5*x)*std::sin(0.5*y)*std::cos(0.5*z) + std::sin(0.5*x)*std::cos(0.5*y)*std::sin(0.5*z);
    const Real q3 = std::cos(0.5*x)*std::cos(0.5*y)*std::sin(0.5*z) - std::sin(0.5*x)*std::sin(0.5*y)*std::cos(0.5*z);
    const Real w = std::cos(0.5*x)*std::cos(0.5*y)*std::cos(0.5*z) + std::sin(0.5*x)*std::sin(0.5*y)*std::sin(0.5*z);

    return Vec4r({q1, q2, q3, w});
}

} // namespace GeometryUtils