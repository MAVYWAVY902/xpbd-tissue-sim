#include "config/simobject/VirtuosoArmConfig.hpp"

namespace Config
{

std::unique_ptr<VirtuosoArmConfig::ObjectType> VirtuosoArmConfig::createObject(const Sim::Simulation* sim) const
{
    return std::make_unique<ObjectType>(sim, this);
}

} // namespace Config