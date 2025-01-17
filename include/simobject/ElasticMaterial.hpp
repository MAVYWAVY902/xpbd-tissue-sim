#ifndef __ELASTIC_MATERIAL_HPP
#define __ELASTIC_MATERIAL_HPP

#include <string>

#include "config/ElasticMaterialConfig.hpp"

/** A class for representing an elastic material.
 * Basically just stores material properties
 */
class ElasticMaterial
{
    public:
    /** Static predefined Rubber material */
    static ElasticMaterial& RUBBER() { static ElasticMaterial rubber("Rubber", 100*100*100/1000, 3e6, 0.49, 0.6, 0.3); return rubber; }
    
    public:

    explicit ElasticMaterial(const ElasticMaterialConfig* config)
        : _name(config->name()),
          _density(config->density()),
          _E(config->E()),
          _nu(config->nu()),
          _mu_s(config->muS()),
          _mu_k(config->muK())
    {
        // calculate Lame parameters
        _mu = _E / (2 * (1 + _nu));
        _lambda = (_E*_nu) / ( (1 + _nu) * (1 - 2*_nu) );
    }

    /** Creates a new ElasticMaterial from density, E, and Poisson's ratio.
     * @param name : the name of the new ElasticMaterial
     * @param density : the density of the material
     * @param E : the elastic modulus of the material
     * @param nu : the Poisson's ratio of the material
    */
    explicit ElasticMaterial(const std::string& name, const double density, const double E, const double nu, const double mu_s, const double mu_k)
        : _name(name), _density(density), _E(E), _nu(nu), _mu_s(mu_s), _mu_k(mu_k)
    {
        // calculate Lame parameters
        _mu = _E / (2 * (1 + _nu));
        _lambda = (_E*_nu) / ( (1 + _nu) * (1 - 2*_nu) );
    }

    // define getters for the material properties
    std::string name() const { return _name; }
    std::string toString() const 
    {
        // write material information to file
        return "Elastic Material '" + name() + "':\n\tDensity: " + std::to_string(density()) + "\n\tE: " + std::to_string(E()) + 
        "\n\tnu: " + std::to_string(nu()) + "\n\tmu: " + std::to_string(mu()) + "\n\tlambda: " + std::to_string(lambda());
    }
    double density() const { return _density; }
    double E() const { return _E; }
    double nu() const { return _nu; }
    double mu() const { return _mu; }
    double lambda() const { return _lambda; }
    double muS() const { return _mu_s; }
    double muK() const { return _mu_k; }

    protected:
    /** Name of the material */
    std::string _name;
    /** Density of the material */
    double _density;
    /** Elastic modulus of the material */
    double _E;
    /** Poisson's ratio of the material */
    double _nu;
    /** Lame's first parameter */
    double _lambda;
    /**  Lame's second parameter */
    double _mu;

    /** Coefficient of static friction */
    double _mu_s;
    /** Coefficient of kinetic friction */
    double _mu_k;

};

#endif // __ELASTIC_MATERIAL_HPP