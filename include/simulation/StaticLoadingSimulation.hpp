#ifndef __STATIC_LOADING_SIMULATION_HPP
#define __STATIC_LOADING_SIMULATION_HPP

#include "OutputSimulation.hpp"
#include "StaticLoadingSimulationConfig.hpp"
#include <Eigen/Dense>

class StaticLoadingSimulation : public OutputSimulation
{
    public:

    explicit StaticLoadingSimulation(const std::string& config_filename);

    virtual std::string toString() const override;
    virtual std::string type() const override { return "StaticLoadingSimulation"; }

    virtual void setup() override;

    virtual void printInfo() const override;

    protected:
    double _center_force;
    std::vector<unsigned> _deflection_vertices;
    std::vector<Eigen::Vector3d> _deflection_starts;

};

#endif // __STATIC_LOADING_SIMULATION_HPP