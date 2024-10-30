#ifndef __TISSUE_GRASPING_SIMULATION_CONFIG_HPP
#define __TISSUE_GRASPING_SIMULATION_CONFIG_HPP

#include "OutputSimulationConfig.hpp"

enum SimulationInputDevice
{
    MOUSE,
    HAPTIC
};

class TissueGraspingSimulationConfig : public OutputSimulationConfig
{
    /** Static predefined default for simulation time step */
    static std::optional<double>& DEFAULT_GRASP_SIZE() { static std::optional<double> grasp_size(0.001); return grasp_size; }
    /** Static predefined efault for stretch time */
    static std::optional<double>& DEFAULT_Z_SCALING() { static std::optional<double> z_scaling(0.001); return z_scaling; }
    /** Predefined default for input device */
    static std::optional<SimulationInputDevice>& DEFAULT_INPUT_DEVICE() { static std::optional<SimulationInputDevice> input_device(SimulationInputDevice::MOUSE); return input_device; }

    static std::map<std::string, SimulationInputDevice>& INPUT_DEVICE_OPTIONS() 
    {
        static std::map<std::string, SimulationInputDevice> input_device_options{{"Mouse", SimulationInputDevice::MOUSE},
                                                                                 {"Haptic", SimulationInputDevice::HAPTIC}};
        return input_device_options;
    }


    public:
    explicit TissueGraspingSimulationConfig(const YAML::Node& node)
        : OutputSimulationConfig(node)
    {
        _extractParameter("grasp-size", node, _grasp_size, DEFAULT_GRASP_SIZE());
        _extractParameter("z-scaling", node, _z_scaling, DEFAULT_Z_SCALING());
        _extractParameterWithOptions("input-device", node, _input_device, INPUT_DEVICE_OPTIONS(), DEFAULT_INPUT_DEVICE());
        _extractParameter("fixed-faces-filename", node, _fixed_faces_filename);
    }

    std::optional<double> graspSize() const { return _grasp_size.value; }
    std::optional<double> zScaling() const { return _z_scaling.value; }
    std::optional<SimulationInputDevice> inputDevice() const { return _input_device.value; }
    std::optional<std::string> fixedFacesFilename() const { return _fixed_faces_filename.value; }

    protected:
    ConfigParameter<double> _grasp_size; // in m
    ConfigParameter<double> _z_scaling;
    ConfigParameter<SimulationInputDevice> _input_device;
    ConfigParameter<std::string> _fixed_faces_filename;
};

#endif // __TISSUE_GRASPING_SIMULATION_CONFIG_HPP