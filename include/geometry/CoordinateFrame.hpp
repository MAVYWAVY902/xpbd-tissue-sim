#ifndef __COORDINATE_FRAME_HPP
#define __COORDINATE_FRAME_HPP

// #include <Eigen/Dense>
#include "geometry/TransformationMatrix.hpp"

namespace Geometry
{

class CoordinateFrame
{
    public:
    explicit CoordinateFrame(const TransformationMatrix& transform)
        : _transform(transform) 
    {
    }

    explicit CoordinateFrame(TransformationMatrix&& transform)
        : _transform(std::move(transform))
    {
    }
    
    explicit CoordinateFrame(const Eigen::Matrix4d& transform)
        : _transform(transform)
    {
    }

    explicit CoordinateFrame()
        : _transform(Eigen::Matrix4d::Identity())
    {
    }

    CoordinateFrame operator*(const TransformationMatrix& transform) const
    {
        return CoordinateFrame(_transform * transform);
    }

    const TransformationMatrix& transform() const { return _transform; }
    Eigen::Vector3d origin() const { return _transform.translation(); }

    /** Sets location and orientation of the coordinate frame via a specified SE3 transformation matrix. */
    void setTransform(const Eigen::Matrix4d& new_transform) { _transform = TransformationMatrix(new_transform); }

    /** Sets the origin of the coordinate frame, keeping the rotation of the coordinate axes the same. */
    void setOrigin(const Eigen::Vector3d& new_origin) { _transform.setTranslation(new_origin); }

    /** Transform the coordinate frame by a specified SE3 transformation matrix. */
    void applyTransform(const TransformationMatrix& transform) { _transform *= transform; } 

    protected:
    // Eigen::Matrix4d _transform;     // transform relative to the global origin, SE3 4x4 transformation matrix
    TransformationMatrix _transform;
};

} // namespace Geometry

#endif // __COORDINATE_FRAME_HPP