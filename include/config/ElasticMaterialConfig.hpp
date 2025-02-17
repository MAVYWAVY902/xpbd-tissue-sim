#ifndef __ELASTIC_MATERIAL_CONFIG_HPP
#define __ELASTIC_MATERIAL_CONFIG_HPP

#include "config/Config.hpp"

class ElasticMaterialConfig : public Config
{
    /** Static predefined default for density */
    static std::optional<Real>& DEFAULT_DENSITY() { static std::optional<Real> density(1000); return density; }
    /** Static predefined default for Young's Modulus */
    static std::optional<Real>& DEFAULT_E() { static std::optional<Real> E(3e6); return E; }
    /** Static predefined default for Poisson's ratio */
    static std::optional<Real>& DEFAULT_NU() { static std::optional<Real> nu(0.4999); return nu; }
    /** Static predefined default for coeff of static friction */
    static std::optional<Real>& DEFAULT_MU_S() { static std::optional<Real> mu_s(0.5); return mu_s; }
    /** Static predefined efault for coeff of kinetic friction */
    static std::optional<Real>& DEFAULT_MU_K() { static std::optional<Real> mu_k(0.2); return mu_k; }

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
        _extractParameter("mu-s", node, _mu_s, DEFAULT_MU_S());
        _extractParameter("mu-k", node, _mu_k, DEFAULT_MU_K());
    }

    explicit ElasticMaterialConfig(Real density, Real E, Real nu, Real mu_s, Real mu_k)
    {
        _density.value = density;
        _E.value = E;
        _nu.value = nu;
        _mu_s.value = mu_s;
        _mu_k.value = mu_k;
    }

    // Getters
    Real density() const { return _density.value.value(); }
    Real E() const { return _E.value.value(); }
    Real nu() const { return _nu.value.value(); }
    Real muS() const { return _mu_s.value.value(); }
    Real muK() const { return _mu_k.value.value(); }

    protected:
    // Parameters
    ConfigParameter<Real> _density;
    ConfigParameter<Real> _E;
    ConfigParameter<Real> _nu;
    ConfigParameter<Real> _mu_s;
    ConfigParameter<Real> _mu_k;
};

#endif