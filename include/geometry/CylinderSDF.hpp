#ifndef __CYLINDER_SDF_HPP
#define __CYLINDER_SDF_HPP

#include "geometry/SDF.hpp"
#include "simobject/RigidPrimitives.hpp"
#include "utils/GeometryUtils.hpp"

namespace Geometry
{

class CylinderSDF : public SDF
{
    public:
    CylinderSDF(const Sim::RigidCylinder* cyl)
        : _cyl(cyl)
    {
    }

    virtual double evaluate(const Eigen::Vector3d& x) const override
    {
        const Eigen::Vector3d x_body = GeometryUtils::rotateVectorByQuat(x - _cyl->position(), GeometryUtils::inverseQuat(_cyl->orientation()));
        // std::cout << "x: " << x[0] << ", " << x[1] << ", " << x[2] << std::endl;
        // vec2 d = abs(vec2(length(p.xz),p.y)) - vec2(r,h);
        // return min(max(d.x,d.y),0.0) + length(max(d,0.0));

        const double xy_dist = std::sqrt(x_body[0]*x_body[0] + x_body[1]*x_body[1]) - _cyl->radius();
        const double z_dist = std::abs(x_body[2]) - 0.5*_cyl->height();

        const double outside_dist_xy = std::max(xy_dist, 0.0);
        const double outside_dist_z = std::max(z_dist, 0.0);
        const double outside_dist = std::sqrt(outside_dist_xy*outside_dist_xy + outside_dist_z*outside_dist_z);
        const double inside_dist = std::min(std::max(xy_dist, z_dist), 0.0);
        // std::cout << "CylinderSDF dist: " << outside_dist + inside_dist << std::endl;
        return outside_dist + inside_dist;
    }

    virtual Eigen::Vector3d gradient(const Eigen::Vector3d& x) const override
    {
        const Eigen::Vector3d x_body = GeometryUtils::rotateVectorByQuat(x - _cyl->position(), GeometryUtils::inverseQuat(_cyl->orientation()));
        // std::cout << "x_body: " << x_body[0] << ", " << x_body[1] << ", " << x_body[2] << std::endl;
        const double xy_dist = std::sqrt(x_body[0]*x_body[0] + x_body[1]*x_body[1]) - _cyl->radius();
        const double z_dist = std::abs(x_body[2]) - 0.5*_cyl->height();

        const double outside_dist_xy = std::max(xy_dist, 0.0);
        const double outside_dist_z = std::max(z_dist, 0.0);
        const double inside_dist = std::min(std::max(xy_dist, z_dist), 0.0);

        const Eigen::Vector3d outside_grad = Eigen::Vector3d({x_body[0]*(outside_dist_xy>0), x_body[1]*(outside_dist_xy>0), x_body[2]*(outside_dist_z>0)}).normalized();
        const Eigen::Vector3d inside_grad = Eigen::Vector3d({x_body[0]*(inside_dist==xy_dist), x_body[1]*(inside_dist==xy_dist), x_body[2]*(inside_dist==z_dist)}).normalized();
        // std::cout << "outside grad: " << outside_grad[0] << ", " << outside_grad[1] << ", " << outside_grad[2] << std::endl;
        // std::cout << "inside_grad: " << inside_grad[0] << ", " << inside_grad[1] << ", " << inside_grad[2] << std::endl;
        // const Eigen::Vector3d grad = 1/outside_len * Eigen::Vector3d({x[0]*(outside_dist_xy>0), x[1]*(outside_dist_z>0), x[2]*(outside_dist_xy>0)})
        //                                         + 1/inside_dist * Eigen::Vector3d({x[0]*(inside_dist==xy_dist), x[1]*(inside_dist==z_dist), x[2]*(inside_dist==xy_dist)});
        return GeometryUtils::rotateVectorByQuat(outside_grad + inside_grad, _cyl->orientation());

    }

    protected:
    const Sim::RigidCylinder* _cyl;
    

};

} // namespace Geometry

#endif // __CYLINDER_SDF_HPP