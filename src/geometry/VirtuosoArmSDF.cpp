#include "geometry/VirtuosoArmSDF.hpp"

#include "simobject/VirtuosoArm.hpp"

namespace Geometry
{

VirtuosoArmSDF::VirtuosoArmSDF(const Sim::VirtuosoArm* arm)
    : _virtuoso_arm(arm)
{
}

Real VirtuosoArmSDF::evaluate(const Vec3r& x) const
{
    const Vec3r x_sdf = _globalToBodyInnerTube(x);
    const Real l = std::max(_virtuoso_arm->innerTubeTranslation() - _virtuoso_arm->outerTubeTranslation(), Real(0.0));
    return _cylinderSDFDistance(x_sdf, _virtuoso_arm->innerTubeOuterDiameter()/2.0, l);
}

Vec3r VirtuosoArmSDF::gradient(const Vec3r& x) const
{
    const Vec3r x_sdf = _globalToBodyInnerTube(x);
    const Real l = std::max(_virtuoso_arm->innerTubeTranslation() - _virtuoso_arm->outerTubeTranslation(), Real(0.0));
    const Vec3r body_grad = _cylinderSDFGradient(x_sdf, _virtuoso_arm->innerTubeOuterDiameter()/2.0, l);
    return _bodyToGlobalInnerTubeGradient(body_grad);
}

Vec3r VirtuosoArmSDF::_bodyToGlobalInnerTubeGradient(const Vec3r& grad) const
{
    // const Real ot_curv = _virtuoso_arm->outerTubeRadiusOfCurvature();
    // const Real ot_rot = _virtuoso_arm->outerTubeRotation();
    // const Real ot_trans = _virtuoso_arm->outerTubeTranslation();
    // const Real ot_distal_len = _virtuoso_arm->outerTubeDistalStraightLength();
    // const Real ot_max_angle = std::max(ot_trans - ot_distal_len, 0.0) / ot_curv;

    const Real it_trans = _virtuoso_arm->innerTubeTranslation();
    const Real ot_trans = _virtuoso_arm->outerTubeTranslation();
    const Real exposed_length = std::max(Real(0.0), it_trans - ot_trans);

    // Mat3r rot_mat;
    // rot_mat <<  std::cos(ot_rot)*std::cos(ot_max_angle), -std::sin(ot_rot), -std::cos(ot_rot)*std::sin(ot_max_angle),
    //             std::sin(ot_max_angle), 0, std::cos(ot_max_angle),
    //             -std::sin(ot_rot)*std::cos(ot_max_angle), -std::cos(ot_rot), std::sin(ot_rot)*std::sin(ot_max_angle);

    const TransformationMatrix it_middle_transform = _virtuoso_arm->innerTubeStartFrame().transform() * TransformationMatrix(Mat3r::Identity(), Vec3r(0,0,exposed_length/2));
    
    return it_middle_transform.rotMat()*grad;
}

Vec3r VirtuosoArmSDF::_globalToBodyInnerTube(const Vec3r& x) const
{
    // const Real ot_curv = _virtuoso_arm->outerTubeRadiusOfCurvature();
    // const Real ot_rot = _virtuoso_arm->outerTubeRotation();
    const Real it_trans = _virtuoso_arm->innerTubeTranslation();
    const Real ot_trans = _virtuoso_arm->outerTubeTranslation();
    const Real exposed_length = std::max(Real(0.0), it_trans - ot_trans);
    // const Real ot_distal_len = _virtuoso_arm->outerTubeDistalStraightLength();
    // const Real ot_max_angle = std::max(ot_trans - ot_distal_len, 0.0) / ot_curv;

    // Mat3r rot_mat;
    // rot_mat <<  std::cos(ot_rot)*std::cos(ot_max_angle), -std::sin(ot_rot), -std::cos(ot_rot)*std::sin(ot_max_angle),
    //             std::sin(ot_max_angle), 0, std::cos(ot_max_angle),
    //             -std::sin(ot_rot)*std::cos(ot_max_angle), -std::cos(ot_rot), std::sin(ot_rot)*std::sin(ot_max_angle);

    // const Real p_x = ot_curv * std::cos(ot_max_angle) - ot_curv - ot_distal_len*std::sin(ot_max_angle);
    // const Real p_y = ot_curv * std::sin(ot_max_angle) + ot_distal_len*std::cos(ot_max_angle);
    // const Real l = std::max(_virtuoso_arm->innerTubeTranslation() - _virtuoso_arm->outerTubeTranslation(), 0.0);
    // const Vec3r ot_pos = Vec3r::Zero();//_virtuoso_arm->outerTubePosition();
    // Vec3r trans_vec(  std::cos(ot_rot) * (p_x - std::sin(ot_max_angle)*l*0.5) + ot_pos[0],
    //                             p_y + std::cos(ot_max_angle)*l*0.5 + ot_pos[1],
    //                             -std::sin(ot_rot) * (p_x - std::sin(ot_max_angle)*l*0.5) + ot_pos[2]);
    
    
    const TransformationMatrix it_middle_transform = _virtuoso_arm->innerTubeStartFrame().transform() * TransformationMatrix(Mat3r::Identity(), Vec3r(0,0,exposed_length/2));
    const TransformationMatrix inv_transform = it_middle_transform.inverse();

    return inv_transform.rotMat()*x + inv_transform.translation();

}

Real VirtuosoArmSDF::_cylinderSDFDistance(const Vec3r& x, Real cyl_radius, Real cyl_height) const
{
    // the distance from x to the cylinder in the XY plane (i.e. the distance from x to the surface of an infinite cylinder with the same radius)
    const Real xy_dist = std::sqrt(x[0]*x[0] + x[1]*x[1]) - cyl_radius;
    // the distance from x to the top/bottom of the cylinder solely in the z-direction
    const Real z_dist = std::abs(x[2]) - 0.5*cyl_height;

    // if this is positive, x is outside the cylinder in the XY plane
    const Real outside_dist_xy = std::max(xy_dist, Real(0.0));
    // if this is positive, x is outside the cylinder along the Z-axis
    const Real outside_dist_z = std::max(z_dist, Real(0.0));

    // the distance from x to the surface of the cylinder when x is outside the cylinder
    // evaluates to 0 when x is fully inside the cylinder
    const Real dist_when_outside = std::sqrt(outside_dist_xy*outside_dist_xy + outside_dist_z*outside_dist_z);

    // the distance from x to the surface of the cylinder when x is inside the cylinder
    // this is the max (i.e. closest to zero) between the XY distance and the Z distance
    // evaluates to 0 when x is outside the cylinder
    const Real dist_when_inside = std::min(std::max(xy_dist, z_dist), Real(0.0));

    return dist_when_outside + dist_when_inside;
}

Vec3r VirtuosoArmSDF::_cylinderSDFGradient(const Vec3r& x, Real cyl_radius, Real cyl_height) const
{
    // the distance from x to the cylinder in the XY plane (i.e. the distance from x to the surface of an infinite cylinder with the same radius)        
    const Real xy_dist = std::sqrt(x[0]*x[0] + x[1]*x[1]) - cyl_radius;

    // the distance from x to the top/bottom of the cylinder solely in the z-direction
    const Real z_dist = std::abs(x[2]) - 0.5*cyl_height;

    // if this is positive, x is outside the cylinder in the XY plane
    const Real outside_dist_xy = std::max(xy_dist, Real(0.0));
    // if this is positive, x is outside the cylinder along the Z-axis
    const Real outside_dist_z = std::max(z_dist, Real(0.0));

    // the distance from x to the surface of the cylinder when x is inside the cylinder
    // this is the max (i.e. closest to zero) between the XY distance and the Z distance
    // evaluates to 0 when x is outside the cylinder
    const Real inside_dist = std::min(std::max(xy_dist, z_dist), Real(0.0));

    const Vec3r grad_when_outside = Vec3r({x[0]*(outside_dist_xy>0), x[1]*(outside_dist_xy>0), x[2]*(outside_dist_z>0)}).normalized();
    const Vec3r grad_when_inside = Vec3r({x[0]*(inside_dist==xy_dist), x[1]*(inside_dist==xy_dist), x[2]*(inside_dist==z_dist)}).normalized();
    return grad_when_outside + grad_when_inside;
}

} // namespace Geometry