#ifndef __VIRTUOSO_SIMULATION_CONFIG_HPP
#define __VIRTUOSO_SIMULATION_CONFIG_HPP

#include "config/simulation/SimulationConfig.hpp"

namespace Config
{

class VirtuosoSimulationConfig : public SimulationConfig
{
    /** Predefined default for input device */
    static std::optional<SimulationInput::Device>& DEFAULT_INPUT_DEVICE() { static std::optional<SimulationInput::Device> input_device(SimulationInput::Device::KEYBOARD); return input_device; }

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
        _extractParameterWithOptions("input-device", node, _input_device, INPUT_DEVICE_OPTIONS(), DEFAULT_INPUT_DEVICE());
    }

    SimulationInput::Device inputDevice() const { return _input_device.value.value(); }

    protected:
    ConfigParameter<SimulationInput::Device> _input_device;
};

} // namespace Config


#endif // __VIRTUOSO_SIMULATION_CONFIG_HPP