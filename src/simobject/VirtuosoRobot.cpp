#include "simobject/VirtuosoRobot.hpp"

namespace Sim
{

VirtuosoRobot::VirtuosoRobot(const Simulation* sim, const VirtuosoRobotConfig* config)
    : Object(sim, config), _arm1(nullptr), _arm2(nullptr)
{
    for (const auto& arm_config : config->armConfigs())
    {
        if (!_arm1)
        {
            _arm1 = std::make_unique<VirtuosoArm>(sim, arm_config.get());
        }
        else if (!_arm2)
        {
            _arm2 = std::make_unique<VirtuosoArm>(sim, arm_config.get());
        }
        else
        {
            std::cout << "More than two arms specified! Using only the first two..." << std::endl;
            break;
        }
    }

    const Eigen::Vector3d& position = config->initialPosition();

    const Eigen::Vector3d& rot_eul_xyz = config->initialRotation() * M_PI / 180.0;
    const Eigen::Matrix3d& rot_mat = GeometryUtils::quatToMat(GeometryUtils::eulXYZ2Quat(rot_eul_xyz[0], rot_eul_xyz[1], rot_eul_xyz[2]));
    _endoscope_frame = Geometry::CoordinateFrame(Geometry::TransformationMatrix(rot_mat, position));

    _endoscope_dia = config->endoscopeDiameter();
    _endoscope_length = config->endoscopeLength();
    _arm_separation_dist = config->armSeparationDistance();
    _optic_vertical_dist = config->opticVerticalDistance();
    _optic_tilt = config->opticTilt() * M_PI / 180.0;

    // origin of VirtuosoRobot frame is at the center of the end of the endoscope
    // Z points forward away from the endoscope (aligned with initial arm Z axes)
    // Y up, X left
    // arm 1 is the left arm, arm 2 is the right arm
    Geometry::TransformationMatrix arm1_rel_transform(Eigen::Matrix3d::Identity(), Eigen::Vector3d(_arm_separation_dist/2.0, 0, 0));
    Geometry::TransformationMatrix arm2_rel_transform(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-_arm_separation_dist/2.0, 0, 0));
    _VB_frame = _endoscope_frame * arm1_rel_transform;
    _VR_frame = _endoscope_frame * arm2_rel_transform;

    if (_arm1)
    {
        _arm1->setBasePosition(_VB_frame.transform().translation());
        _arm1->setBaseRotation(_VB_frame.transform().rotMat());
    }
    if (_arm2)
    {
        _arm2->setBasePosition(_VR_frame.transform().translation());
        _arm2->setBaseRotation(_VR_frame.transform().rotMat());
    }

    // compute cam frame
    Geometry::TransformationMatrix cam_rel_transform(GeometryUtils::Rx(_optic_tilt), Eigen::Vector3d(0, _optic_vertical_dist, 0));
    _cam_frame = _endoscope_frame * cam_rel_transform;

}

std::string VirtuosoRobot::toString(int indent) const
{
    // TODO: better toString
    return Object::toString(indent);
}

void VirtuosoRobot::setup()
{
    if (_arm1)
        _arm1->setup();
    if (_arm2)
        _arm2->setup();
}

void VirtuosoRobot::update()
{
    if (_arm1)
        _arm1->update();
    if (_arm2)
        _arm2->update();
}

void VirtuosoRobot::velocityUpdate()
{
    // nothing for now
}

} // namespace Sim