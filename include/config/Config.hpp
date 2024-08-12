#ifndef __CONFIG_HPP
#define __CONFIG_HPP

#include "yaml-cpp/yaml.h"

#include <Eigen/Dense>

#include <optional>
#include <iostream>

template <typename T>
struct ConfigParameter
{
    std::string name;
    std::optional<T> value;
};

class Config
{
    public:
    explicit Config();
    explicit Config(const YAML::Node& node);

    virtual ~Config() = default;

    std::optional<std::string> name() const { return _name.value; }

    protected:

    template<typename T>
    void _extractParameter(const std::string& param_name, const YAML::Node& yaml_node, ConfigParameter<T>& param)
    {
        param.name = param_name;

        try 
        {
            if (yaml_node[param_name].Type() != YAML::NodeType::Null)
            {
                param.value = yaml_node[param_name].as<T>();
            }
            else
            {
                std::cerr << "Parameter with name " << param_name << " is null (did you forget to set it?)" << std::endl;
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            std::cerr << "Parameter name " << param_name << " does not exist or is not of the type " << typeid(T).name() << "." << std::endl;
        }
    }

    void _extractParameter(const std::string& param_name, const YAML::Node& yaml_node, ConfigParameter<Eigen::Vector3d>& param)
    {
        std::cout << "Setting Vector3 parameter with name " << param_name << "..." << std::endl;
        param.name = param_name;
        try
        {
            if (yaml_node[param_name].Type() != YAML::NodeType::Null)
            {
                param.value = Eigen::Vector3d({ yaml_node[param_name][0].as<double>(), 
                                                yaml_node[param_name][1].as<double>(),
                                                yaml_node[param_name][2].as<double>() });
            }
            else
            {
                std::cerr << "Parameter with name " << param_name << " is null (did you forget to set it?)" << std::endl;
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        
    }

    void _extractParameter(const std::string& param_name, const YAML::Node& yaml_node, ConfigParameter<Eigen::Vector4d>& param)
    {
        std::cout << "Setting Vector4 parameter with name " << param_name << "..." << std::endl;
        param.name = param_name;
        try
        {
            if (yaml_node[param_name].Type() != YAML::NodeType::Null)
            {
                param.value = Eigen::Vector4d({ yaml_node[param_name][0].as<double>(), 
                                                yaml_node[param_name][1].as<double>(),
                                                yaml_node[param_name][2].as<double>(),
                                                yaml_node[param_name][3].as<double>() });
            }
            else
            {
                std::cerr << "Parameter with name " << param_name << " is null (did you forget to set it?)" << std::endl;
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        
    }

    protected:
    ConfigParameter<std::string> _name;    
};

#endif // __CONFIG_HPP