#ifndef __SDF_HPP
#define __SDF_HPP

#include <Eigen/Dense>

namespace Geometry
{

class SDF
{
    public:
    virtual ~SDF() = default;

    virtual double evaluate(const Eigen::Vector3d& x) const = 0;

    virtual Eigen::Vector3d gradient(const Eigen::Vector3d& x) const = 0;

};

} // namespace Geometry

#endif // __SDF_HPP