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
        _extractParameter("sdf-filename", node, _sdf_filename);
    }

    std::optional<std::string> sdfFilename() const { return _sdf_filename.value; }

    protected:
    ConfigParameter<std::string> _sdf_filename;
};

#endif // __RIGID_MESH_OBJECT_CONFIG