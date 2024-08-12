#include "config/Config.hpp"

#include <iostream>

Config::Config()
{

}

Config::Config(const YAML::Node& node)
{
    _extractParameter("name", node, _name);
}

// template<typename T>
// void Config::_extractParameter(const std::string& param_name, const YAML::Node& yaml_node, ConfigParameter<T>& param)
// {
//     param.name = param_name;

//     try 
//     {
//         if (yaml_node[param_name].Type() != YAML::NodeType::Null)
//         {
//             param.value = yaml_node[param_name].as<T>();
//         }
//         else
//         {
//             std::cerr << "Parameter with name " << param_name << " is null (did you forget to set it?)" << std::endl;
//         }
//     }
//     catch (const std::exception& e)
//     {
//         std::cerr << e.what() << '\n';
//         std::cerr << "Parameter name " << param_name << " does not exist or is not of the type " << typeid(T).name() << "." << std::endl;
//     }
// }

// void Config::_extractParameter(const std::string& param_name, const YAML::Node& yaml_node, ConfigParameter<Eigen::Vector3d>& param)
// {
//     std::cout << "Setting Vector3 parameter with name " << param_name << "..." << std::endl;
//     param.name = param_name;
//     try
//     {
//         if (yaml_node[param_name].Type() != YAML::NodeType::Null)
//         {
//             param.value = Eigen::Vector3d({ yaml_node[param_name][0].as<double>(), 
//                                             yaml_node[param_name][1].as<double>(),
//                                             yaml_node[param_name][2].as<double>() });
//         }
//         else
//         {
//             std::cerr << "Parameter with name " << param_name << " is null (did you forget to set it?)" << std::endl;
//         }
//     }
//     catch(const std::exception& e)
//     {
//         std::cerr << e.what() << '\n';
//     }
    
// }

// void Config::_extractParameter(const std::string& param_name, const YAML::Node& yaml_node, ConfigParameter<Eigen::Vector4d>& param)
// {
//     std::cout << "Setting Vector4 parameter with name " << param_name << "..." << std::endl;
//     param.name = param_name;
//     try
//     {
//         if (yaml_node[param_name].Type() != YAML::NodeType::Null)
//         {
//             param.value = Eigen::Vector4d({ yaml_node[param_name][0].as<double>(), 
//                                             yaml_node[param_name][1].as<double>(),
//                                             yaml_node[param_name][2].as<double>(),
//                                             yaml_node[param_name][3].as<double>() });
//         }
//         else
//         {
//             std::cerr << "Parameter with name " << param_name << " is null (did you forget to set it?)" << std::endl;
//         }
//     }
//     catch(const std::exception& e)
//     {
//         std::cerr << e.what() << '\n';
//     }
    
// }