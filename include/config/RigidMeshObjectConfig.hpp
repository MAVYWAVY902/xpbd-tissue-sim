#ifndef __RIGID_MESH_OBJECT_CONFIG_HPP
#define __RIGID_MESH_OBJECT_CONFIG_HPP

#include "config/MeshObjectConfig.hpp"

enum RigidMeshPrimitiveType
{
    PLANE
};

class RigidMeshObjectConfig : public MeshObjectConfig
{
    static std::map<std::string, RigidMeshPrimitiveType>& PRIMITIVE_TYPE_OPTIONS() 
    { 
        static std::map<std::string, RigidMeshPrimitiveType> primitive_type_options{{"Plane", RigidMeshPrimitiveType::PLANE}}; 
        return primitive_type_options; 
    }

    public:
    /** Creates a Config from a YAML node, which consists of specialized parameters needed for RigidMeshObject
     * @param node : the YAML node (i.e. dictionary of key-value pairs) that information is pulled from
     */
    explicit RigidMeshObjectConfig(const YAML::Node& node)
        : MeshObjectConfig(node)
    {
        _extractParameterWithOptions("primitive-type", node, _primitive_type, PRIMITIVE_TYPE_OPTIONS());

    }

    // Getters
    std::optional<RigidMeshPrimitiveType> primitiveType() const { return _primitive_type.value; }

    protected:
    ConfigParameter<RigidMeshPrimitiveType> _primitive_type;

};

#endif // __RIGID_MESH_OBJECT_CONFIG