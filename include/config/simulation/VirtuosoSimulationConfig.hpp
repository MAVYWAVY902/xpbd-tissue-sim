#ifndef __VIRTUOSO_SIMULATION_CONFIG_HPP
#define __VIRTUOSO_SIMULATION_CONFIG_HPP

#include "config/simulation/SimulationConfig.hpp"

namespace Config
{

class VirtuosoSimulationConfig : public SimulationConfig
{
    static std::map<std::string, SimulationInput::Device>& INPUT_DEVICE_OPTIONS() 
    {
        static std::map<std::string, SimulationInput::Device> input_device_options{{"Mouse", SimulationInput::Device::MOUSE},
                                                                                 {"Keyboard", SimulationInput::Device::KEYBOARD},
                                                                                 {"Haptic", SimulationInput::Device::HAPTIC}};
        return input_device_options;
    }

    public:
    explicit VirtuosoSimulationConfig(const YAML::Node& node)
        : SimulationConfig(node)
    {
        _extractParameterWithOptions("input-device", node, _input_device, INPUT_DEVICE_OPTIONS());
    }

    SimulationInput::Device inputDevice() const { return _input_device.value; }

    protected:
    ConfigParameter<SimulationInput::Device> _input_device = ConfigParameter<SimulationInput::Device>(SimulationInput::Device::MOUSE);
};

} // namespace Config


#endif // __VIRTUOSO_SIMULATION_CONFIG_HPP