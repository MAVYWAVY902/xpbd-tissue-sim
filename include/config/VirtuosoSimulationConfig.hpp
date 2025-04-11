#ifndef __VIRTUOSO_SIMULATION_CONFIG_HPP
#define __VIRTUOSO_SIMULATION_CONFIG_HPP

#include "config/SimulationConfig.hpp"

enum SimulationInputDevice
{
    MOUSE,
    KEYBOARD,
    HAPTIC
};

class VirtuosoSimulationConfig : public SimulationConfig
{
    /** Predefined default for input device */
    static std::optional<SimulationInputDevice>& DEFAULT_INPUT_DEVICE() { static std::optional<SimulationInputDevice> input_device(SimulationInputDevice::KEYBOARD); return input_device; }

    static std::map<std::string, SimulationInputDevice>& INPUT_DEVICE_OPTIONS() 
    {
        static std::map<std::string, SimulationInputDevice> input_device_options{{"Mouse", SimulationInputDevice::MOUSE},
                                                                                 {"Keyboard", SimulationInputDevice::KEYBOARD},
                                                                                 {"Haptic", SimulationInputDevice::HAPTIC}};
        return input_device_options;
    }

    public:
    explicit VirtuosoSimulationConfig(const YAML::Node& node)
        : SimulationConfig(node)
    {
        _extractParameterWithOptions("input-device", node, _input_device, INPUT_DEVICE_OPTIONS(), DEFAULT_INPUT_DEVICE());
        _extractParameter("fixed-faces-filename", node, _fixed_faces_filename);
        _extractParameter("goal-filename", node, _goal_filename);
    }

    SimulationInputDevice inputDevice() const { return _input_device.value.value(); }
    std::optional<std::string> fixedFacesFilename() const { return _fixed_faces_filename.value; }
    std::optional<std::string> goalFilename() const { return _goal_filename.value; }

    protected:
    ConfigParameter<SimulationInputDevice> _input_device;
    ConfigParameter<std::string> _fixed_faces_filename; 
    ConfigParameter<std::string> _goal_filename;
};


#endif // __VIRTUOSO_SIMULATION_CONFIG_HPP