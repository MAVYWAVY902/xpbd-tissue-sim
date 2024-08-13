#ifndef __SIMULATION_CONFIG_HPP
#define __SIMULATION_CONFIG_HPP

#include "config/MeshObjectConfig.hpp"
#include "config/XPBDMeshObjectConfig.hpp"
#include "config/RigidMeshObjectConfig.hpp"

class SimulationConfig : public Config
{
    public:
    /** Creates a Config from a YAML node, which consists of parameters needed for Simulation.
     * @param node : the YAML node (i.e. dictionary of key-value pairs) that information is pulled from
     */
    explicit SimulationConfig(const YAML::Node& node)
        : Config(node)
    {
        // extract parameters
        _extractParameter("time-step", node, _time_step);
        _extractParameter("end-time", node, _end_time);

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
                _mesh_object_configs.push_back(std::make_unique<XPBDMeshObjectConfig>(obj_node));
            }
            if(type == "RigidMeshObject")
            {
                _mesh_object_configs.push_back(std::make_unique<RigidMeshObjectConfig>(obj_node));
            }
        }
    }

    // Getters
    std::optional<double> timeStep() const { return _time_step.value; }
    std::optional<double> endTime() const { return _end_time.value; }

    // get list of MeshObject configs that will be used to create MeshObjects
    const std::vector<std::unique_ptr<MeshObjectConfig> >& meshObjectConfigs() const { return _mesh_object_configs; }

    protected:
    // Parameters
    ConfigParameter<double> _time_step;
    ConfigParameter<double> _end_time; 

    /** List of MeshObject configs for each object in the Simulation */
    std::vector<std::unique_ptr<MeshObjectConfig>> _mesh_object_configs;

};

#endif // __SIMULATION_CONFIG_HPP