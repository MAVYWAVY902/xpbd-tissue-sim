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
    static ElasticMaterial& RUBBER() { static ElasticMaterial rubber("Rubber", 100*100*100/1000, 3e6, 0.49); return rubber; }
    
    public:

    explicit ElasticMaterial(const ElasticMaterialConfig* config)
        : _name(config->name().value_or("")),
          _density(config->density().value_or(1000)),
          _E(config->E().value_or(3e6)),
          _nu(config->nu().value_or(0.49))
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
    explicit ElasticMaterial(const std::string& name, const double density, const double E, const double nu)
        : _name(name), _density(density), _E(E), _nu(nu)
    {
        // calculate Lame parameters
        _mu = _E / (2 * (1 + _nu));
        _lambda = (_E*_nu) / ( (1 + _nu) * (1 - 2*_nu) );
    }

    // define getters for the material properties
    std::string name() const { return _name; }
    double density() const { return _density; }
    double E() const { return _E; }
    double nu() const { return _nu; }
    double mu() const { return _mu; }
    double lambda() const { return _lambda; }

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
    

};

#endif // __ELASTIC_MATERIAL_HPP