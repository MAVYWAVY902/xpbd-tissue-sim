#ifndef __VIRTUOSO_ARM_SDF_HPP
#define __VIRTUOSO_ARM_SDF_HPP

#include "geometry/SDF.hpp"
#include "simobject/VirtuosoArm.hpp"

namespace Geometry
{

class VirtuosoArmSDF : public SDF
{
    public:
    VirtuosoArmSDF(const Sim::VirtuosoArm* arm)
        : _virtuoso_arm(arm)
    {
    }

    /** Evaluates F(x) for a Virtuoso Arm in its current state.
     * @param x - the point at which to evaluate the SDF
     * @returns the distance from x to the shape boundary ( F(x) )
    */
    virtual double evaluate(const Eigen::Vector3d& x) const override
    {
        const Eigen::Vector3d x_sdf = _globalToBodyInnerTube(x);
        const double l = std::max(_virtuoso_arm->innerTubeTranslation() - _virtuoso_arm->outerTubeTranslation(), 0.0);
        return _cylinderSDFDistance(x_sdf, _virtuoso_arm->innerTubeDiameter()/2.0, l);
    }

    /** Evaluates the gradient of F at x.
     * @param x - the point at which to evaluate the graient of the SDF
     * @returns the gradient of the SDF at x.
     */
    virtual Eigen::Vector3d gradient(const Eigen::Vector3d& x) const override
    {
        const Eigen::Vector3d x_sdf = _globalToBodyInnerTube(x);
        const double l = std::max(_virtuoso_arm->innerTubeTranslation() - _virtuoso_arm->outerTubeTranslation(), 0.0);
        const Eigen::Vector3d body_grad = _cylinderSDFGradient(x_sdf, _virtuoso_arm->innerTubeDiameter()/2.0, l);
        return _bodyToGlobalInnerTubeGradient(body_grad);
    }

    private:
    Eigen::Vector3d _bodyToGlobalInnerTubeGradient(const Eigen::Vector3d& grad) const
    {
        const double ot_curv = _virtuoso_arm->outerTubeCurvature();
        const double ot_rot = _virtuoso_arm->outerTubeRotation();
        const double ot_trans = _virtuoso_arm->outerTubeTranslation();
        const double ot_max_angle = ot_trans/ot_curv;

        Eigen::Matrix3d rot_mat;
        rot_mat <<  std::cos(ot_rot)*std::cos(ot_max_angle), -std::sin(ot_rot), -std::cos(ot_rot)*std::sin(ot_max_angle),
                    std::sin(ot_max_angle), 0, std::cos(ot_max_angle),
                    -std::sin(ot_rot)*std::cos(ot_max_angle), -std::cos(ot_rot), std::sin(ot_rot)*std::sin(ot_max_angle);
        return rot_mat*grad;
    }
    Eigen::Vector3d _globalToBodyInnerTube(const Eigen::Vector3d& x) const
    {
        const double ot_curv = _virtuoso_arm->outerTubeCurvature();
        const double ot_rot = _virtuoso_arm->outerTubeRotation();
        const double ot_trans = _virtuoso_arm->outerTubeTranslation();
        const double ot_max_angle = ot_trans/ot_curv;

        Eigen::Matrix3d rot_mat;
        rot_mat <<  std::cos(ot_rot)*std::cos(ot_max_angle), -std::sin(ot_rot), -std::cos(ot_rot)*std::sin(ot_max_angle),
                    std::sin(ot_max_angle), 0, std::cos(ot_max_angle),
                    -std::sin(ot_rot)*std::cos(ot_max_angle), -std::cos(ot_rot), std::sin(ot_rot)*std::sin(ot_max_angle);
        
        const double p_x = ot_curv * std::cos(ot_max_angle) - ot_curv;
        const double p_y = ot_curv * std::sin(ot_max_angle);
        const double l = std::max(_virtuoso_arm->innerTubeTranslation() - _virtuoso_arm->outerTubeTranslation(), 0.0);
        const Eigen::Vector3d ot_pos = _virtuoso_arm->outerTubePosition();
        Eigen::Vector3d trans_vec(  std::cos(ot_rot) * (p_x - std::sin(ot_max_angle)*l*0.5) + ot_pos[0],
                                    p_y + std::cos(ot_max_angle)*l*0.5 + ot_pos[1],
                                    -std::sin(ot_rot) * (p_x - std::sin(ot_max_angle)*l*0.5) + ot_pos[2]);
        
        return rot_mat.transpose()*x - rot_mat.transpose()*trans_vec;

    }

    double _cylinderSDFDistance(const Eigen::Vector3d& x, double cyl_radius, double cyl_height) const
    {
        // the distance from x to the cylinder in the XY plane (i.e. the distance from x to the surface of an infinite cylinder with the same radius)
        const double xy_dist = std::sqrt(x[0]*x[0] + x[1]*x[1]) - cyl_radius;
        // the distance from x to the top/bottom of the cylinder solely in the z-direction
        const double z_dist = std::abs(x[2]) - 0.5*cyl_height;

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

    Eigen::Vector3d _cylinderSDFGradient(const Eigen::Vector3d& x, double cyl_radius, double cyl_height) const
    {
        // the distance from x to the cylinder in the XY plane (i.e. the distance from x to the surface of an infinite cylinder with the same radius)        
        const double xy_dist = std::sqrt(x[0]*x[0] + x[1]*x[1]) - cyl_radius;

        // the distance from x to the top/bottom of the cylinder solely in the z-direction
        const double z_dist = std::abs(x[2]) - 0.5*cyl_height;

        // if this is positive, x is outside the cylinder in the XY plane
        const double outside_dist_xy = std::max(xy_dist, 0.0);
        // if this is positive, x is outside the cylinder along the Z-axis
        const double outside_dist_z = std::max(z_dist, 0.0);

        // the distance from x to the surface of the cylinder when x is inside the cylinder
        // this is the max (i.e. closest to zero) between the XY distance and the Z distance
        // evaluates to 0 when x is outside the cylinder
        const double inside_dist = std::min(std::max(xy_dist, z_dist), 0.0);

        const Eigen::Vector3d grad_when_outside = Eigen::Vector3d({x[0]*(outside_dist_xy>0), x[1]*(outside_dist_xy>0), x[2]*(outside_dist_z>0)}).normalized();
        const Eigen::Vector3d grad_when_inside = Eigen::Vector3d({x[0]*(inside_dist==xy_dist), x[1]*(inside_dist==xy_dist), x[2]*(inside_dist==z_dist)}).normalized();
        return grad_when_outside + grad_when_inside;
    }

    private:
    const Sim::VirtuosoArm* _virtuoso_arm;
};

} // namespace Geometry


#endif // __VIRTUOSO_ARM_SDF_HPP