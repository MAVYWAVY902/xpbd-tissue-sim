#include "config/simobject/XPBDMeshObjectConfig.hpp"

#include "simobject/XPBDObjectFactory.hpp"

namespace Config
{

std::unique_ptr<Sim::XPBDMeshObject_Base> XPBDMeshObjectConfig::createObject(const Sim::PhysicsContext* sim) const
{
    return XPBDObjectFactory::createXPBDMeshObject(sim, this);
}

} // namespace Config