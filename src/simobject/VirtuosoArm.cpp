#include "simobject/VirtuosoArm.hpp"

namespace Sim 
{

VirtuosoArm::VirtuosoArm(const Simulation* sim, const VirtuosoArmConfig* config)
    : Object(sim, config)
{
    _it_dia = config->innerTubeDiameter();
    _it_translation = config->innerTubeInitialTranslation();
    _it_rotation = config->innerTubeInitialRotation() * 3.1415/180.0;   // convert to radians

    _ot_dia = config->outerTubeDiameter();
    _ot_curvature = config->outerTubeCurvature();
    _ot_translation = config->outerTubeInitialTranslation();
    _ot_rotation = config->outerTubeInitialRotation() * 3.1415/180.0;   // convert to radians

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

} // namespace Sim