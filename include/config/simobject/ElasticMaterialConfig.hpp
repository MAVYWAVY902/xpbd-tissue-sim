#ifndef __ELASTIC_MATERIAL_CONFIG_HPP
#define __ELASTIC_MATERIAL_CONFIG_HPP

#include "config/Config.hpp"

namespace Config
{

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
        _extractParameter("mu-s", node, _mu_s);
        _extractParameter("mu-k", node, _mu_k);
    }

    explicit ElasticMaterialConfig(const std::string& name, Real density, Real E, Real nu, Real mu_s, Real mu_k)
        : Config(name)
    {
        _density.value = density;
        _E.value = E;
        _nu.value = nu;
        _mu_s.value = mu_s;
        _mu_k.value = mu_k;
    }

    // Getters
    Real density() const { return _density.value; }
    Real E() const { return _E.value; }
    Real nu() const { return _nu.value; }
    Real muS() const { return _mu_s.value; }
    Real muK() const { return _mu_k.value; }

    protected:
    // Parameters
    ConfigParameter<Real> _density = ConfigParameter<Real>(1000);
    ConfigParameter<Real> _E = ConfigParameter<Real>(3e6);
    ConfigParameter<Real> _nu = ConfigParameter<Real>(0.499);
    ConfigParameter<Real> _mu_s = ConfigParameter<Real>(0.5);
    ConfigParameter<Real> _mu_k = ConfigParameter<Real>(0.2);
};

} // namespace Config

#endif