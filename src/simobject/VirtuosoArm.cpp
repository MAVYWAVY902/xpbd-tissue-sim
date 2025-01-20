#include "simobject/VirtuosoArm.hpp"

namespace Sim 
{

VirtuosoArm::VirtuosoArm(const Simulation* sim, const VirtuosoArmConfig* config)
    : Object(sim, config)
{

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