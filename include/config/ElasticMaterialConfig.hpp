#ifndef __ELASTIC_MATERIAL_CONFIG_HPP
#define __ELASTIC_MATERIAL_CONFIG_HPP

#include "config/Config.hpp"

class ElasticMaterialConfig : public Config
{
    /** Static predefined default for density */
    static std::optional<double>& DEFAULT_DENSITY() { static std::optional<double> density(1000); return density; }
    /** Static predefined default for Young's Modulus */
    static std::optional<double>& DEFAULT_E() { static std::optional<double> E(3e6); return E; }
    /** Static predefined default for Poisson's ratio */
    static std::optional<double>& DEFAULT_NU() { static std::optional<double> nu(0.4999); return nu; }
    public:
    /** Creates a Config from a YAML node, which consists of the specialized parameters needed for ElasticMaterial
     * @param node : the YAML node (i.e. dictionary of key-value pairs) that information is pulled from
     */
    explicit ElasticMaterialConfig(const YAML::Node& node)
        : Config(node)
    {
        // extract parameters
        _extractParameter("density", node, _density, DEFAULT_DENSITY());
        _extractParameter("E", node, _E, DEFAULT_E());
        _extractParameter("nu", node, _nu, DEFAULT_NU());
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