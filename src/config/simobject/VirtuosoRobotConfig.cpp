#include "config/simobject/VirtuosoRobotConfig.hpp"

#include "simobject/VirtuosoRobot.hpp"

namespace Config
{

std::unique_ptr<VirtuosoRobotConfig::ObjectType> VirtuosoRobotConfig::createObject(const Sim::PhysicsContext* sim) const
{
    return std::make_unique<ObjectType>(sim, this);
}

} // namespace Config