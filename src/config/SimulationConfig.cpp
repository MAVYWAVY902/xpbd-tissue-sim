#include "config/SimulationConfig.hpp"
#include "config/XPBDMeshObjectConfig.hpp"

#include <iostream>

SimulationConfig::SimulationConfig(const YAML::Node& node)
    : Config(node)
{
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
            continue;
        }
        

        // create the specified type of object
        if (type == "XPBDMeshObject")
        { 
            _mesh_object_configs.push_back(std::make_unique<XPBDMeshObjectConfig>(obj_node));
        }
        if (type == "ElasticMeshObject")
        {
            _mesh_object_configs.push_back(std::make_unique<ElasticMeshObjectConfig>(obj_node));
        }
        if (type == "MeshObject")
        {
            _mesh_object_configs.push_back(std::make_unique<MeshObjectConfig>(obj_node));
        }
        if(type == "RigidMeshObject")
        {
            // RigidMeshObject* mesh_obj = new RigidMeshObject(name, obj);
            // addObject(mesh_obj);
        }
    }
}