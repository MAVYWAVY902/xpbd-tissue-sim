#ifndef __SPHERE_SDF_HPP
#define __SPHERE_SDF_HPP

#include "geometry/SDF.hpp"
#include "simobject/RigidPrimitives.hpp"

namespace Geometry
{

/** Implements a signed distance function for a rigid sphere. */
class SphereSDF : public SDF
{
    public:
    SphereSDF(const Sim::RigidSphere* sphere)
        : SDF(), _sphere(sphere)
    {}

    /** Evaluates F(x) for a sphere.
     * @param x - the point at which to evaluate the SDF
     * @returns the distance from x to the shape boundary ( F(x) )
    */
    virtual double evaluate(const Eigen::Vector3d& x) const override
    {
        // the distance from any point the surface of the sphere is simply the distance of the point to the sphere center minus the radius
        return (x - _sphere->position()).norm() - _sphere->radius();
    }

    /** Evaluates the gradient of F at x.
     * @param x - the point at which to evaluate the graient of the SDF
     * @returns the gradient of the SDF at x.
     */
    virtual Eigen::Vector3d gradient(const Eigen::Vector3d& x) const override
    {
        // the gradient simply is a normalized vector pointing out from the sphere center in the direction of x
        return (x - _sphere->position()).normalized();
    }

    const Sim::RigidSphere* sphere() const { return _sphere; }

    protected:
    /** Pointer to sphere needed for sphere's current position and radius. */
    const Sim::RigidSphere* _sphere;
};

} // namespace Geometry

#endif // __SPHERE_SDF_HPP