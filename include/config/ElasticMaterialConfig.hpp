#ifndef __ELASTIC_MATERIAL_CONFIG_HPP
#define __ELASTIC_MATERIAL_CONFIG_HPP

#include "config/Config.hpp"

class ElasticMaterialConfig : public Config
{
    public:
    explicit ElasticMaterialConfig(const YAML::Node& node);

    std::optional<double> density() const { return _density.value; }
    std::optional<double> E() const { return _E.value; }
    std::optional<double> nu() const { return _nu.value; }

    protected:
    ConfigParameter<double> _density;
    ConfigParameter<double> _E;
    ConfigParameter<double> _nu;
};

#endif