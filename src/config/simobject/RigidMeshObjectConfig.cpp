#include "config/simobject/RigidMeshObjectConfig.hpp"

#include "simobject/RigidMeshObject.hpp"

namespace Config
{

std::unique_ptr<RigidMeshObjectConfig::ObjectType> RigidMeshObjectConfig::createObject(const Sim::PhysicsContext* sim) const
{
    return std::make_unique<ObjectType>(sim, this);
}

} // namespace Config