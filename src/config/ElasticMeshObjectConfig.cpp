#include "config/ElasticMeshObjectConfig.hpp"

ElasticMeshObjectConfig::ElasticMeshObjectConfig(const YAML::Node& node)
    : MeshObjectConfig(node)
{
    _material_config = std::make_unique<ElasticMaterialConfig>(node["material"]);
}