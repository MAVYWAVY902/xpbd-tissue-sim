#include "config/simobject/FirstOrderXPBDMeshObjectConfig.hpp"

#include "simobject/XPBDObjectFactory.hpp"

namespace Config
{

std::unique_ptr<FirstOrderXPBDMeshObjectConfig::ObjectType> FirstOrderXPBDMeshObjectConfig::createObject(const Sim::Simulation* sim) const
{
    return XPBDObjectFactory::createFirstOrderXPBDMeshObject(sim, this);
}

} // namespace Config