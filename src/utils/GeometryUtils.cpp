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

Vec4r matToQuat(const Mat3r& mat)
{
    Vec4r q;

    // code adapted from https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
    const Real trace = mat.trace();
    if( trace > 0 )
    {
        Real s = Real(0.5) / std::sqrt(trace + 1);
        q[3] = Real(0.25) / s;
        q[0] = ( mat(2,1) - mat(1,2) ) * s;
        q[1] = ( mat(0,2) - mat(2,0) ) * s;
        q[2] = ( mat(1,0) - mat(0,1) ) * s;
    } 
    else
    {
        if ( mat(0,0) > mat(1,1) && mat(0,0) > mat(2,2) ) {
            Real s = 2 * std::sqrt( 1 + mat(0,0) - mat(1,1) - mat(2,2));
            q[3] = (mat(2,1) - mat(1,2) ) / s;
            q[0] = Real(0.25) * s;
            q[1] = (mat(0,1) + mat(1,0) ) / s;
            q[2] = (mat(0,2) + mat(2,0) ) / s;
        } else if (mat(1,1) > mat(2,2)) {
            Real s = 2 * std::sqrt( 1 + mat(1,1) - mat(0,0) - mat(2,2));
            q[3] = (mat(0,2) - mat(2,0) ) / s;
            q[0] = (mat(0,1) + mat(1,0) ) / s;
            q[1] = 0.25f * s;
            q[2] = (mat(1,2) + mat(2,1) ) / s;
        } else {
            Real s = 2 * std::sqrt( 1 + mat(2,2) - mat(0,0) - mat(1,1) );
            q[3] = (mat(1,0) - mat(0,1) ) / s;
            q[0] = (mat(0,2) + mat(2,0) ) / s;
            q[1] = (mat(1,2) + mat(2,1) ) / s;
            q[2] = 0.25f * s;
        }
    }
    
    return q;
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

Eigen::Matrix3d Rz(double theta)
{
    Eigen::Matrix3d rot_mat;
    rot_mat << std::cos(theta), -std::sin(theta), 0.0,
                std::sin(theta), std::cos(theta), 0.0,
                0.0, 0.0, 1.0;
    return rot_mat;
}

Eigen::Matrix3d Ry(double theta)
{
    Eigen::Matrix3d rot_mat;
    rot_mat << std::cos(theta), 0.0, std::sin(theta),
                0.0, 1.0, 0.0,
                -std::sin(theta), 0.0, std::cos(theta);
    return rot_mat;
}

Eigen::Matrix3d Rx(double theta)
{
    Eigen::Matrix3d rot_mat;
    rot_mat << 1.0, 0.0, 0.0,
                0.0, std::cos(theta), -std::sin(theta),
                0.0, std::sin(theta), std::cos(theta);
    return rot_mat;
}

Eigen::Vector3d Vee_SO3(const Eigen::Matrix3d& mat)
{
    // we'll just assume mat is skew-symmetric
    return Eigen::Vector3d(mat(2,1), mat(0,2), mat(1,0));
}

Eigen::Matrix3d Bracket_so3(const Eigen::Vector3d& vec)
{
    // make skew-symmetric matrix
    Eigen::Matrix3d mat;
    mat << 0, -vec[2], vec[1],
           vec[2], 0, -vec[0],
           -vec[1], vec[0], 0;
    return mat;
}

Eigen::Vector<double,6> Vee_SE3(const Eigen::Matrix4d& mat)
{
    Eigen::Vector<double,6> vec;
    vec(Eigen::seq(0,2)) = Vee_SO3(mat.block<3,3>(0,0));
    vec(Eigen::seq(3,5)) = mat.block<3,1>(0,3);
    return vec;
}

Eigen::Matrix4d Bracket_se3(const Eigen::Vector<double,6>& vec)
{
    Eigen::Matrix4d mat = Eigen::Matrix4d::Zero();
    mat.block<3,3>(0,0) = Bracket_so3( (vec(Eigen::seq(0,2))) );
    mat.block<3,1>(0,3) = vec(Eigen::seq(3,5));
    return mat;
}

} // namespace GeometryUtils