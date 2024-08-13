#ifndef __RIGID_MESH_OBJECT_CONFIG_HPP
#define __RIGID_MESH_OBJECT_CONFIG_HPP

#include "config/MeshObjectConfig.hpp"

class RigidMeshObjectConfig : public MeshObjectConfig
{
    public:
    /** Creates a Config from a YAML node, which consists of specialized parameters needed for RigidMeshObject
     * @param node : the YAML node (i.e. dictionary of key-value pairs) that information is pulled from
     */
    explicit RigidMeshObjectConfig(const YAML::Node& node)
        : MeshObjectConfig(node)
    {
        _extractParameter("primitive-type", node, _primitive_type);

    }

    // Getters
    std::optional<std::string> primitiveType() const { return _primitive_type.value; }

    protected:
    ConfigParameter<std::string> _primitive_type;

};

#endif // __RIGID_MESH_OBJECT_CONFIG