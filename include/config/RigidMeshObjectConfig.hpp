#ifndef __RIGID_MESH_OBJECT_CONFIG_HPP
#define __RIGID_MESH_OBJECT_CONFIG_HPP

#include "config/MeshObjectConfig.hpp"

class RigidMeshObjectConfig : public MeshObjectConfig
{
    public:
    explicit RigidMeshObjectConfig(const YAML::Node& node)
        : MeshObjectConfig(node)
    {

    }

};

#endif // __RIGID_MESH_OBJECT_CONFIG