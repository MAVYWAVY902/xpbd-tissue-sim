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
        _extractParameter("show-tip-cursor", node, _show_tip_cursor);
        _extractParameter("haptic-force-scaling", node, _haptic_force_scaling);
    }

    SimulationInput::Device inputDevice() const { return _input_device.value; }
    bool showTipCursor() const { return _show_tip_cursor.value; }
    Real hapticForceScaling() const { return _haptic_force_scaling.value; }

    protected:
    ConfigParameter<SimulationInput::Device> _input_device = ConfigParameter<SimulationInput::Device>(SimulationInput::Device::MOUSE);

    ConfigParameter<bool> _show_tip_cursor = ConfigParameter<bool>(true);

    ConfigParameter<Real> _haptic_force_scaling = ConfigParameter<Real>(1.0);
};

} // namespace Config


#endif // __VIRTUOSO_SIMULATION_CONFIG_HPP