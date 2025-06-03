#ifndef __BEAM_STRETCH_SIMULATION_CONFIG_HPP
#define __BEAM_STRETCH_SIMULATION_CONFIG_HPP

#include "config/simulation/OutputSimulationConfig.hpp"

namespace Config
{

class BeamStretchSimulationConfig : public OutputSimulationConfig
{
    /** Static predefined default for simulation time step */
    static std::optional<Real>& DEFAULT_STRETCH_VELOCITY() { static std::optional<Real> velocity(0); return velocity; }
    /** Static predefined efault for stretch time */
    static std::optional<Real>& DEFAULT_STRETCH_TIME() { static std::optional<Real> stretch_time(1); return stretch_time; }

    public:
    explicit BeamStretchSimulationConfig(const YAML::Node& node)
        : OutputSimulationConfig(node)
    {
        // extract the parameters from the config file
        _extractParameter("stretch-velocity", node, _stretch_velocity, DEFAULT_STRETCH_VELOCITY());
        _extractParameter("stretch-time", node, _stretch_time, DEFAULT_STRETCH_TIME());
    }

    // getters
    std::optional<Real> stretchVelocity() const { return _stretch_velocity.value; }
    std::optional<Real> stretchTime() const { return _stretch_time.value; }

    protected:
    /** The velocity of the beam stretching, in m/s. Use a negative value for beam compression. */
    ConfigParameter<Real> _stretch_velocity;
    /** The amount of time to stretch the beam. */
    ConfigParameter<Real> _stretch_time; // in s
};

} // namespace Config

#endif // __BEAM_STRETCH_SIMULATION_CONFIG_HPP