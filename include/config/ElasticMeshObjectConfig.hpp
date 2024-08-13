#ifndef __ELASTIC_MESH_OBJECT_CONFIG_HPP
#define __ELASTIC_MESH_OBJECT_CONFIG_HPP

#include "config/MeshObjectConfig.hpp"
#include "config/ElasticMaterialConfig.hpp"

class ElasticMeshObjectConfig : public MeshObjectConfig
{
    public:
    /** Creates a Config from a YAML node, which only consists specialized parameters needed for ElasticMeshObject.
     * @param node : the YAML node (i.e. dictionary of key-value pairs) that information is pulled from
     */
    explicit ElasticMeshObjectConfig(const YAML::Node& node)
        : MeshObjectConfig(node)
    {
        // create the ElasticMaterialConfig from material node
        _material_config = std::make_unique<ElasticMaterialConfig>(node["material"]);
    }

    // Getters
    ElasticMaterialConfig* materialConfig() const { return _material_config.get(); }

    protected:
    // parameters
    std::unique_ptr<ElasticMaterialConfig> _material_config;
};

#endif // __ELASTIC_MESH_OBJECT_CONFIG_HPP