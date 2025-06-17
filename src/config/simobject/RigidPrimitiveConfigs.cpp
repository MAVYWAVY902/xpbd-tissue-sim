#include "config/simobject/RigidPrimitiveConfigs.hpp"

#include "simobject/RigidPrimitives.hpp"

namespace Config
{

std::unique_ptr<RigidSphereConfig::ObjectType> RigidSphereConfig::createObject(const Sim::Simulation* sim) const
{
    return std::make_unique<ObjectType>(sim, this);
}

std::unique_ptr<RigidBoxConfig::ObjectType> RigidBoxConfig::createObject(const Sim::Simulation* sim) const
{
    return std::make_unique<ObjectType>(sim, this);
}

std::unique_ptr<RigidCylinderConfig::ObjectType> RigidCylinderConfig::createObject(const Sim::Simulation* sim) const
{
    return std::make_unique<ObjectType>(sim, this);
}

} // namespace Config