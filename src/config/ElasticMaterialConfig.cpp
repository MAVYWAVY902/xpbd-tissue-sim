#include "config/ElasticMaterialConfig.hpp"

ElasticMaterialConfig::ElasticMaterialConfig(const YAML::Node& node)
    : Config(node)
{
    _extractParameter("density", node, _density);
    _extractParameter("E", node, _E);
    _extractParameter("nu", node, _nu);
}