#include "simobject/VirtuosoArm.hpp"

#include "utils/GeometryUtils.hpp"

namespace Sim 
{

VirtuosoArm::VirtuosoArm(const Simulation* sim, const VirtuosoArmConfig* config)
    : Object(sim, config)
{
    _it_dia = config->innerTubeDiameter();
    _it_translation = config->innerTubeInitialTranslation();
    _it_rotation = config->innerTubeInitialRotation() * 3.1415/180.0;   // convert to radians

    _ot_dia = config->outerTubeDiameter();
    _ot_r_curvature = config->outerTubeRadiusOfCurvature();
    _ot_translation = config->outerTubeInitialTranslation();
    _ot_rotation = config->outerTubeInitialRotation() * 3.1415/180.0;   // convert to radians
    _ot_distal_straight_length = config->outerTubeDistalStraightLength();

    _ot_position = config->outerTubeInitialPosition();

}

std::string VirtuosoArm::toString(const int indent) const
{
    // TODO: better toString
    return Object::toString(indent);
}

void VirtuosoArm::setup()
{
    // nothing for now
}

void VirtuosoArm::update()
{
    // nothing for now
}

void VirtuosoArm::velocityUpdate()
{
    // nothing for now
}

/** Returns the axis-aligned bounding-box (AABB) for this Object in global simulation coordinates. */
Geometry::AABB VirtuosoArm::boundingBox() const
{
    return Geometry::AABB(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
}

void VirtuosoArm::_recomputeCoordinateFrames()
{
    // compute endoscope (base) frame based on current position and orientation
    // TODO: add endoscope orientation
    _endoscope_frame = Geometry::CoordinateFrame(Geometry::TransformationMatrix(Eigen::Matrix3d::Identity(), _ot_position));

    // compute outer tube base frmae
    // consists of rotating about the current z-axis accordint to outer tube rotation
    Geometry::TransformationMatrix T_rot_z(GeometryUtils::Rz(_ot_rotation), Eigen::Vector3d::Zero());
    _ot_base_frame = _endoscope_frame * T_rot_z;

    // compute frame for end of curved section of outer tube
    // consists of rotating around the body y-axis by the angle swept by the outer tube curve
    double swept_angle = std::max(_ot_translation - _ot_distal_straight_length, 0.0) / _ot_r_curvature; 
    const Eigen::Vector3d end_of_xz_curve(-_ot_r_curvature*std::cos(swept_angle) + _ot_r_curvature, 0, _ot_r_curvature*std::sin(swept_angle));
    Geometry::TransformationMatrix T_curve_end(GeometryUtils::Ry(swept_angle), end_of_xz_curve);

    _ot_curve_end_frame = _ot_base_frame * T_rot_z * T_curve_end;

    // compute frame for end of outer tube
    // consists of simply moving along the current z-axis by the length of the distal straight section
    Geometry::TransformationMatrix T_curve_end_to_ot_end(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, std::min(_ot_distal_straight_length, _ot_translation)));
    _ot_end_frame = _ot_curve_end_frame * T_curve_end_to_ot_end;

    // compute frame for end of inner tube
    // consists of rotating around the current z-axis by the inner tube rotation and moving along the current z-axis by the exposed length of the inner tube
    Geometry::TransformationMatrix T_ot_end_to_it_end(GeometryUtils::Rz(_it_rotation), Eigen::Vector3d(0, 0, std::max(0.0, _it_translation - _ot_translation)));
    _it_end_frame = _ot_end_frame * T_ot_end_to_it_end;

}

} // namespace Sim