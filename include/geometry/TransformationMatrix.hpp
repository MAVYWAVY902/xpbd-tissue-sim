#ifndef __TRANSFORMATION_MATRIX_HPP
#define __TRANSFORMATION_MATRIX_HPP

#include <Eigen/Dense>

namespace Geometry
{

class TransformationMatrix
{
    public:
    explicit TransformationMatrix()
        : _matrix(Eigen::Matrix4d::Identity())
    {}

    explicit TransformationMatrix(const Eigen::Matrix3d& rot_mat, const Eigen::Vector3d& origin)
        : _matrix(Eigen::Matrix4d::Identity())
    {
        _matrix.block<3, 3>(0,0) = rot_mat;
        _matrix.block<3, 1>(0,3) = origin;
    }

    explicit TransformationMatrix(const Eigen::Matrix4d& t_mat)
        : _matrix(t_mat)
    {}

    TransformationMatrix operator*(const TransformationMatrix& other) const
    {
        const Eigen::Matrix4d res = _matrix * other._matrix;
        return TransformationMatrix(res);
    }

    void operator*=(const TransformationMatrix& other)
    {
        _matrix *= other._matrix;
    }

    TransformationMatrix operator+(const TransformationMatrix& other) const
    {
        const Eigen::Matrix4d res = _matrix + other._matrix;
        return TransformationMatrix(res);
    }

    TransformationMatrix operator-(const TransformationMatrix& other) const
    {
        const Eigen::Matrix4d res = _matrix - other._matrix;
        return TransformationMatrix(res);
    }

    const Eigen::Matrix4d& asMatrix() const { return _matrix; }

    Eigen::Vector3d origin() const { return _matrix.block<3, 1>(0,3); }
    void setOrigin(const Eigen::Vector3d& new_origin) { _matrix.block<3, 1>(0,3) = new_origin; }

    private:
    Eigen::Matrix4d _matrix;
};

} // namespace Geometry

#endif // __TRANSFORMATION_MATRIX_HPP