#ifndef __VIRTUOSO_SIMULATION_CONFIG_HPP
#define __VIRTUOSO_SIMULATION_CONFIG_HPP

#include "config/SimulationConfig.hpp"

enum SimulationInputDevice
{
    MOUSE,
    KEYBOARD,
    HAPTIC,
    DOUBLE_HAPTIC
};

class VirtuosoSimulationConfig : public SimulationConfig
{
    /** Predefined default for input device */
    static std::optional<SimulationInputDevice>& DEFAULT_INPUT_DEVICE() { static std::optional<SimulationInputDevice> input_device(SimulationInputDevice::KEYBOARD); return input_device; }

    static std::map<std::string, SimulationInputDevice>& INPUT_DEVICE_OPTIONS() 
    {
        static std::map<std::string, SimulationInputDevice> input_device_options{{"Mouse", SimulationInputDevice::MOUSE},
                                                                                 {"Keyboard", SimulationInputDevice::KEYBOARD},
                                                                                 {"Haptic", SimulationInputDevice::HAPTIC},
                                                                                 {"Double Haptic", SimulationInputDevice::DOUBLE_HAPTIC}};
        return input_device_options;
    }

    public:
    explicit VirtuosoSimulationConfig(const YAML::Node& node)
        : SimulationConfig(node)
    {
        _extractParameterWithOptions("input-device", node, _input_device, INPUT_DEVICE_OPTIONS(), DEFAULT_INPUT_DEVICE());
        _extractParameter("fixed-faces-filename", node, _fixed_faces_filename);
        _extractParameter("tumor-faces-filename", node, _tumor_faces_filename);
        _extractParameter("goal-filename", node, _goal_filename);
        _extractParameter("goals-folder", node, _goals_folder);

        _extractParameter("device-name1", node, _device_name1);
        _extractParameter("device-name2", node, _device_name2);
    }

    SimulationInputDevice inputDevice() const { return _input_device.value.value(); }
    std::optional<std::string> fixedFacesFilename() const { return _fixed_faces_filename.value; }
    std::optional<std::string> tumorFacesFilename() const { return _tumor_faces_filename.value; }
    std::optional<std::string> goalFilename() const { return _goal_filename.value; }
    std::optional<std::string> goalsFolder() const { return _goals_folder.value; }

    std::optional<std::string> deviceName1() const { return _device_name1.value; }
    std::optional<std::string> deviceName2() const { return _device_name2.value; }

    protected:
    ConfigParameter<SimulationInputDevice> _input_device;
    ConfigParameter<std::string> _fixed_faces_filename;
    ConfigParameter<std::string> _tumor_faces_filename; 
    ConfigParameter<std::string> _goal_filename;
    ConfigParameter<std::string> _goals_folder;

    ConfigParameter<std::string> _device_name1;
    ConfigParameter<std::string> _device_name2;
};


#endif // __VIRTUOSO_SIMULATION_CONFIG_HPP