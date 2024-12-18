#ifndef __RIGID_MESH_OBJECT_CONFIG_HPP
#define __RIGID_MESH_OBJECT_CONFIG_HPP

#include "config/RigidObjectConfig.hpp"
#include "config/MeshObjectConfig.hpp"

class RigidMeshObjectConfig : public RigidObjectConfig, public MeshObjectConfig
{
    public:

    explicit RigidMeshObjectConfig(const YAML::Node& node)
        : RigidObjectConfig(node), MeshObjectConfig(node)
    {
        
    }

    protected:
};

#endif // __RIGID_MESH_OBJECT_CONFIG