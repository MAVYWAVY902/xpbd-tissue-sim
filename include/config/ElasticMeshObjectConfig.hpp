#ifndef __ELASTIC_MESH_OBJECT_CONFIG_HPP
#define __ELASTIC_MESH_OBJECT_CONFIG_HPP

#include "config/MeshObjectConfig.hpp"
#include "config/ElasticMaterialConfig.hpp"

class ElasticMeshObjectConfig : public MeshObjectConfig
{
    public:
    explicit ElasticMeshObjectConfig(const YAML::Node& node)
        : MeshObjectConfig(node)
    {
        _material_config = std::make_unique<ElasticMaterialConfig>(node["material"]);
    }

    ElasticMaterialConfig* materialConfig() const { return _material_config.get(); }

    protected:
    std::unique_ptr<ElasticMaterialConfig> _material_config;
};

#endif // __ELASTIC_MESH_OBJECT_CONFIG_HPP