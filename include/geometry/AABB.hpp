#ifndef __AABB_HPP
#define __AABB_HPP

#include <Eigen/Dense>

namespace Geometry
{

/** An axis-aligned bounding box (AABB). */
struct AABB
{
    Eigen::Vector3d min;    // the min coordinates of the bounding box
    Eigen::Vector3d max;    // the max coordinates of the bounding box

    /** Initialization from vectors */
    AABB(const Eigen::Vector3d& min_, const Eigen::Vector3d& max_)
        : min(min_), max(max_)
    {}

    /** Initialization from individual coordinates */
    AABB(const double min_x, const double min_y, const double min_z, const double max_x, const double max_y, const double max_z)
        : min(min_x, min_y, min_z), max(max_x, max_y, max_z)
    {}

    /** Returns the center of the bounding box. */
    Eigen::Vector3d center() const
    {
        return min + 0.5*(max - min);
    }

    /** Returns the size of the bounding box. */
    Eigen::Vector3d size() const
    {
        return max - min;
    }

    /** Returns true if two AABBs overlap, false otherwise. */
    bool collidesWith(const AABB& other) const
    {
        return (min[0] <= other.max[0] && max[0] >= other.min[0]) &&
               (min[1] <= other.max[1] && max[1] >= other.min[1]) &&
               (min[2] <= other.max[2] && max[2] >= other.min[2]);
    }
};

} // namespace Geometry

#endif // __AABB_HPP