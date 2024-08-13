#ifndef __ELASTIC_MATERIAL_CONFIG_HPP
#define __ELASTIC_MATERIAL_CONFIG_HPP

#include "config/Config.hpp"

class ElasticMaterialConfig : public Config
{
    public:
    /** Creates a Config from a YAML node, which consists of the specialized parameters needed for ElasticMaterial
     * @param node : the YAML node (i.e. dictionary of key-value pairs) that information is pulled from
     */
    explicit ElasticMaterialConfig(const YAML::Node& node)
        : Config(node)
    {
        // extract parameters
        _extractParameter("density", node, _density);
        _extractParameter("E", node, _E);
        _extractParameter("nu", node, _nu);
    }

    // Getters
    std::optional<double> density() const { return _density.value; }
    std::optional<double> E() const { return _E.value; }
    std::optional<double> nu() const { return _nu.value; }

    protected:
    // Parameters
    ConfigParameter<double> _density;
    ConfigParameter<double> _E;
    ConfigParameter<double> _nu;
};

#endif