#ifndef __CYLINDER_SDF_HPP
#define __CYLINDER_SDF_HPP

#include "geometry/SDF.hpp"
#include "simobject/RigidPrimitives.hpp"
#include "utils/GeometryUtils.hpp"

namespace Geometry
{

/** Implements a signed distance function for a rigid cylinder.
 * Assumes the cylinder in its un-transformed configuration is a vertical cylinder centered about the origin, with its height in the z-direction.
 */
class CylinderSDF : public SDF
{
    public:
    CylinderSDF(const Sim::RigidCylinder* cyl)
        : _cyl(cyl)
    {
    }

    /** Evaluates F(x) for a cylinder with arbitrary position, orientation, radius and height
     * @param x - the point at which to evaluate the SDF
     * @returns the distance from x to the shape boundary ( F(x) )
    */
    virtual double evaluate(const Eigen::Vector3d& x) const override
    {
        // transform x into body coordinates
        const Eigen::Vector3d x_body = _cyl->globalToBody(x);

        // the distance from x to the cylinder in the XY plane (i.e. the distance from x to the surface of an infinite cylinder with the same radius)
        const double xy_dist = std::sqrt(x_body[0]*x_body[0] + x_body[1]*x_body[1]) - _cyl->radius();
        // the distance from x to the top/bottom of the cylinder solely in the z-direction
        const double z_dist = std::abs(x_body[2]) - 0.5*_cyl->height();

        // if this is positive, x is outside the cylinder in the XY plane
        const double outside_dist_xy = std::max(xy_dist, 0.0);
        // if this is positive, x is outside the cylinder along the Z-axis
        const double outside_dist_z = std::max(z_dist, 0.0);

        // the distance from x to the surface of the cylinder when x is outside the cylinder
        // evaluates to 0 when x is fully inside the cylinder
        const double dist_when_outside = std::sqrt(outside_dist_xy*outside_dist_xy + outside_dist_z*outside_dist_z);

        // the distance from x to the surface of the cylinder when x is inside the cylinder
        // this is the max (i.e. closest to zero) between the XY distance and the Z distance
        // evaluates to 0 when x is outside the cylinder
        const double dist_when_inside = std::min(std::max(xy_dist, z_dist), 0.0);

        return dist_when_outside + dist_when_inside;
    }

    /** Evaluates the gradient of F at x.
     * @param x - the point at which to evaluate the graient of the SDF
     * @returns the gradient of the SDF at x.
     */
    virtual Eigen::Vector3d gradient(const Eigen::Vector3d& x) const override
    {
        // transform x into body coordinates
        const Eigen::Vector3d x_body = _cyl->globalToBody(x);

        // the distance from x to the cylinder in the XY plane (i.e. the distance from x to the surface of an infinite cylinder with the same radius)        
        const double xy_dist = std::sqrt(x_body[0]*x_body[0] + x_body[1]*x_body[1]) - _cyl->radius();

        // the distance from x to the top/bottom of the cylinder solely in the z-direction
        const double z_dist = std::abs(x_body[2]) - 0.5*_cyl->height();

        // if this is positive, x is outside the cylinder in the XY plane
        const double outside_dist_xy = std::max(xy_dist, 0.0);
        // if this is positive, x is outside the cylinder along the Z-axis
        const double outside_dist_z = std::max(z_dist, 0.0);

        // the distance from x to the surface of the cylinder when x is inside the cylinder
        // this is the max (i.e. closest to zero) between the XY distance and the Z distance
        // evaluates to 0 when x is outside the cylinder
        const double inside_dist = std::min(std::max(xy_dist, z_dist), 0.0);

        const Eigen::Vector3d grad_when_outside = Eigen::Vector3d({x_body[0]*(outside_dist_xy>0), x_body[1]*(outside_dist_xy>0), x_body[2]*(outside_dist_z>0)}).normalized();
        const Eigen::Vector3d grad_when_inside = Eigen::Vector3d({x_body[0]*(inside_dist==xy_dist), x_body[1]*(inside_dist==xy_dist), x_body[2]*(inside_dist==z_dist)}).normalized();
        return GeometryUtils::rotateVectorByQuat(grad_when_outside + grad_when_inside, _cyl->orientation());

    }

    const Sim::RigidCylinder* cylinder() const { return _cyl; }

    protected:
    /** Pointer to cylinder for the cylinder's current position, orientation, radius and height. */
    const Sim::RigidCylinder* _cyl;
    

};

} // namespace Geometry

#endif // __CYLINDER_SDF_HPP