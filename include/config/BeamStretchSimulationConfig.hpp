#ifndef __BEAM_STRETCH_SIMULATION_CONFIG_HPP
#define __BEAM_STRETCH_SIMULATION_CONFIG_HPP

#include "SimulationConfig.hpp"

class BeamStretchSimulationConfig : public SimulationConfig
{
    /** Static predefined default for simulation time step */
    static std::optional<double>& DEFAULT_STRETCH_VELOCITY() { static std::optional<double> velocity(0); return velocity; }
    /** Static predefined efault for stretch time */
    static std::optional<double>& DEFAULT_STRETCH_TIME() { static std::optional<double> stretch_time(1); return stretch_time; }

    public:
    explicit BeamStretchSimulationConfig(const YAML::Node& node)
        : SimulationConfig(node)
    {
        _extractParameter("stretch-velocity", node, _stretch_velocity, DEFAULT_STRETCH_VELOCITY());
        _extractParameter("stretch-time", node, _stretch_time, DEFAULT_STRETCH_TIME());
    }

    std::optional<double> stretchVelocity() const { return _stretch_velocity.value; }
    std::optional<double> stretchTime() const { return _stretch_time.value; }

    protected:
    ConfigParameter<double> _stretch_velocity; // in m/s
    ConfigParameter<double> _stretch_time; // in s
};

#endif // __BEAM_STRETCH_SIMULATION_CONFIG_HPP