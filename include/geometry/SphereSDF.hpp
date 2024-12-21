#ifndef __SPHERE_SDF_HPP
#define __SPHERE_SDF_HPP

#include "geometry/SDF.hpp"
#include "simobject/RigidPrimitives.hpp"

namespace Geometry
{

class SphereSDF : public SDF
{
    public:
    SphereSDF(const Sim::RigidSphere* sphere)
        : SDF(), _sphere(sphere)
    {}

    virtual double evaluate(const Eigen::Vector3d& x) const override
    {
        // std::cout << "x: " << x[0] << ", " << x[1] << ", " << x[2] << "\tdistance: " << (x - _sphere->position()).norm() - _sphere->radius() << std::endl;
        return (x - _sphere->position()).norm() - _sphere->radius();
    }

    virtual Eigen::Vector3d gradient(const Eigen::Vector3d& x) const override
    {
        const double dist = (x - _sphere->position()).norm();
        return (x - _sphere->position()) / dist;
    }

    protected:
    const Sim::RigidSphere* _sphere;
};

} // namespace Geometry

#endif // __SPHERE_SDF_HPP