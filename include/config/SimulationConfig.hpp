#ifndef __SIMULATION_CONFIG_HPP
#define __SIMULATION_CONFIG_HPP

#include "config/MeshObjectConfig.hpp"
#include "config/XPBDMeshObjectConfig.hpp"
#include "config/RigidMeshObjectConfig.hpp"
#include "config/FastFEMMeshObjectConfig.hpp"


/** Enum defining the different ways the simulation can be run 
 * VISUALIZATION: if simulation is running faster than real-time, slow down updates so that sim time = wall time
 * AFAP: run the simulation As Fast As Possible - i.e. do not slow updates
 * FRAME_BY_FRAME: allow the user to step the simulation forward, time step by time step, using the keyboard
*/
enum SimulationMode
{
    VISUALIZATION,
    AFAP,
    FRAME_BY_FRAME
};

class SimulationConfig : public Config
{
    /** Static predefined default for simulation time step */
    static std::optional<double>& DEFAULT_TIME_STEP() { static std::optional<double> time_step(1e-3); return time_step; }
    /** Static predefined default for simulation end time */
    static std::optional<double>& DEFAULT_END_TIME() { static std::optional<double> end_time(10); return end_time; }
    /** Static predefined default for simulation mode */
    static std::optional<SimulationMode>& DEFAULT_SIM_MODE() { static std::optional<SimulationMode> sim_mode(SimulationMode::VISUALIZATION); return sim_mode; }
    /** Static predefined default for acceleration due to gravity */
    static std::optional<double>& DEFAULT_G_ACCEL() { static std::optional<double> g_accel(9.81); return g_accel; }
    /** Static predefined default for simulation description */
    static std::optional<std::string>& DEFAULT_DESCRIPTION() { static std::optional<std::string> description(""); return description; }
    /** Static predefined default for simulation FPS */
    static std::optional<double>& DEFAULT_FPS() { static std::optional<double> fps(30.0); return fps; }

    /** Static predifined options for the simulation mode. Maps strings to the Simulation mode enum. */
    static std::map<std::string, SimulationMode> SIM_MODE_OPTIONS()
    {
        static std::map<std::string, SimulationMode> sim_mode_options{{"Visualization", SimulationMode::VISUALIZATION},
                                                                      {"AFAP", SimulationMode::AFAP},
                                                                      {"Frame-by-frame", SimulationMode::FRAME_BY_FRAME}};
        return sim_mode_options;
    }

    public:
    /** Creates a Config from a YAML node, which consists of parameters needed for Simulation.
     * @param node : the YAML node (i.e. dictionary of key-value pairs) that information is pulled from
     */
    explicit SimulationConfig(const YAML::Node& node)
        : Config(node)
    {
        // extract parameters
        _extractParameter("time-step", node, _time_step, DEFAULT_TIME_STEP());
        _extractParameter("end-time", node, _end_time, DEFAULT_END_TIME());
        _extractParameterWithOptions("sim-mode", node, _sim_mode, SIM_MODE_OPTIONS(), DEFAULT_SIM_MODE());
        _extractParameter("g-accel", node, _g_accel, DEFAULT_G_ACCEL());
        _extractParameter("description", node, _description, DEFAULT_DESCRIPTION());
        _extractParameter("fps", node, _fps, DEFAULT_FPS());

        // create a MeshObject for each object specified in the YAML file
        for (const auto& obj_node : node["objects"])
        {
            std::string type;
            try 
            {
                // extract type information
                type = obj_node["type"].as<std::string>();
            }
            catch (const std::exception& e)
            {
                std::cerr << e.what() << std::endl;
                std::cerr << "Type of object is needed!" << std::endl;
                continue;
            }
            

            // create the specified type of object based on type string
            if (type == "XPBDMeshObject")
            {
                std::unique_ptr<XPBDMeshObjectConfig> config = std::make_unique<XPBDMeshObjectConfig>(obj_node);
                config->timeStep(_time_step); 
                _mesh_object_configs.push_back(std::move(config));
            }
            if (type == "FastFEMMeshObject")
            {
                std::unique_ptr<FastFEMMeshObjectConfig> config = std::make_unique<FastFEMMeshObjectConfig>(obj_node);
                config->timeStep(_time_step);
                _mesh_object_configs.push_back(std::move(config));
            }
            if(type == "RigidMeshObject")
            {
                std::unique_ptr<RigidMeshObjectConfig> config = std::make_unique<RigidMeshObjectConfig>(obj_node);
                config->timeStep(_time_step);
                _mesh_object_configs.push_back(std::move(config));
            }
        }
    }

    // Getters
    std::optional<double> timeStep() const { return _time_step.value; }
    std::optional<double> endTime() const { return _end_time.value; }
    std::optional<SimulationMode> simMode() const { return _sim_mode.value; }
    std::optional<double> gAccel() const { return _g_accel.value; }
    std::optional<std::string> description() const { return _description.value; }
    std::optional<double> fps() const { return _fps.value; }

    // get list of MeshObject configs that will be used to create MeshObjects
    const std::vector<std::unique_ptr<MeshObjectConfig> >& meshObjectConfigs() const { return _mesh_object_configs; }

    protected:
    // Parameters
    ConfigParameter<double> _time_step;
    ConfigParameter<double> _end_time;
    ConfigParameter<SimulationMode> _sim_mode; 
    ConfigParameter<double> _g_accel;
    ConfigParameter<std::string> _description;
    ConfigParameter<double> _fps;

    /** List of MeshObject configs for each object in the Simulation */
    std::vector<std::unique_ptr<MeshObjectConfig>> _mesh_object_configs;

};

#endif // __SIMULATION_CONFIG_HPP