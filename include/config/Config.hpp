#ifndef __CONFIG_HPP
#define __CONFIG_HPP

#include "yaml-cpp/yaml.h"

#include <Eigen/Dense>

#include <optional>
#include <iostream>

/** Simple templated struct that stores a parameter from a YAML config.
 * The name field is the name of the YAML parameter.
 * The value is the value of the YAML parameter. Note the use of std::optional to represent YAML parameters that are null or missing.
 */
template <typename T>
struct ConfigParameter
{
    std::string name;
    std::optional<T> value;
};

/** A class that represents a YAML node, which at its simplest is just the name of the object.
 * The Config object is increasingly specialized by Derived classes to incorporate more options/parameters.
 * Provides some helper functions to do proper error checking on a YAML config file which handles missing or null parameters gracefully.
 */
class Config
{
    /** Static predefined default name */
    static std::optional<std::string>& DEFAULT_NAME() { static std::optional<std::string> name(""); return name; }

    public:
    /** Default constructor */
    explicit Config() {}

    /** Creates a Config from a YAML node, which only consists of a name.
     * @param node : the YAML node (i.e. dictionary of key-value pairs) that information is pulled from
     */
    explicit Config(const YAML::Node& node)
    {
        // load the name parameter
        _extractParameter("name", node, _name, DEFAULT_NAME());
    }

    /** Declare virtual destructor for polymorphism */
    virtual ~Config() = default;

    // Getters
    std::optional<std::string> name() const { return _name.value; }

    protected:

    /** Extracts a key-value pair from the YAML node.
     * If the parameter doesn't exist, the ConfigParameter value is a null optional.
     * @param param_name : the name of the parameter to get from the YAML file
     * @param yaml_node : the YAML node to extract information from
     * @param param : (output) the ConfigParameter, which gets set by the function. At the very least, the param name is set and if there is no error, the value gets set too.
     * 
     */
    template<typename T>
    void _extractParameter(const std::string& param_name, const YAML::Node& yaml_node, ConfigParameter<T>& param, const std::optional<T>& default_value = std::nullopt)
    {
        // set the name field of the ConfigParameter
        param.name = param_name;

        try 
        {
            if (yaml_node[param_name].Type() != YAML::NodeType::Null)
            {
                // if we get here, the parameter exists, and it is not null
                // so, set the value of the ConfigParameter and we're done!
                param.value = yaml_node[param_name].as<T>();
                return;
            }
            else
            {
                // parameter in YAML node exists, but is null
                std::cerr << "Parameter with name " << param_name << " is null (did you forget to set it?)" << std::endl;
            }
        }
        catch (const std::exception& e)
        {
            // parameter in YAML does not exist
            std::cerr << e.what() << '\n';
            std::cerr << "Parameter name " << param_name << " does not exist or is not of the type " << typeid(T).name() << "." << std::endl;
        }

        // if we get to here, the parameter was not specified so set the default value
        param.value = default_value;

        if (default_value.has_value())
        {
            std::cout << "Setting parameter " << param_name << " to default value of " << default_value.value() << std::endl;
        }
    }

    template<typename K, typename T>
    void _extractParameterWithOptions(const std::string& param_name, const YAML::Node& yaml_node, ConfigParameter<T>& param, const std::map<K, T>& options, const std::optional<T>& default_value = std::nullopt)
    {
        // set the name field of the ConfigParameter
        param.name = param_name;

        try
        {
            if (yaml_node[param_name].Type() != YAML::NodeType::Null)
            {
                K key = yaml_node[param_name].as<K>();
                if (options.count(key) == 1)
                {
                    param.value = options.at(key);
                    return;
                }
                else
                {
                    std::cerr << key << " is not a valid option for parameter " << param_name << "! Valid options are ";
                    for (const auto& [k, v] : options)
                    {
                        std::cerr << k << "; ";
                    }
                    std::cerr << std::endl;
                }
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }

        // if we get to here, the parameter was not specified so set the default value
        param.value = default_value;

        if (default_value.has_value())
        {
            std::cout << "Setting parameter " << param_name << " to default value of " << default_value.value() << std::endl;
        }
    }

    /** Extracts a 3-vector from YAML node as an Eigen::Vector3d
     * If the parameter doesn't exist, the ConfigParameter value is a null optional.
     * @param param_name : the name of the 3-Vector parameter to get from the YAML file
     * @param yaml_node : the YAML node to extract information from
     * @param param : (output) the ConfigParameter, which gets set by the function
     */
    void _extractParameter(const std::string& param_name, const YAML::Node& yaml_node, ConfigParameter<Eigen::Vector3d>& param, const std::optional<Eigen::Vector3d>& default_value = std::nullopt)
    {
        // set the name field of the ConfigParameter
        param.name = param_name;
        try
        {
            if (yaml_node[param_name].Type() != YAML::NodeType::Null)
            {
                // make sure the node is an array of exactly 3 values
                if (yaml_node[param_name].size() == 3)
                {
                    // if we get here, the parameter exists and it is not null
                    // so, set the value of the ConfigParameter
                    param.value = Eigen::Vector3d({ yaml_node[param_name][0].as<double>(), 
                                                    yaml_node[param_name][1].as<double>(),
                                                    yaml_node[param_name][2].as<double>() });
                    return;
                }
                else
                {
                    std::cerr << "Expected exactly 3 values for the parameter " << param_name << std::endl;
                }
                
            }
            else
            {
                // parameter in YAML node exists, but is null
                std::cerr << "Parameter with name " << param_name << " is null (did you forget to set it?)" << std::endl;
            }
        }
        catch(const std::exception& e)
        {
            // parameter in YAML does not exist
            std::cerr << e.what() << '\n';
        }

        // if we get to here, there was an issue with the parameter
        param.value = default_value;

        if (default_value.has_value())
        {
            std::cout << "Setting parameter " << param_name << " to default value of " << default_value.value() << std::endl;
        }
        
    }

    /** Extracts a 4-vector from YAML node as an Eigen::Vector4d 
     * If the parameter doesn't exist, the ConfigParameter value is a null optional.
     * @param param_name : the name of the 4-vector parameter to get from the YAML file
     * @param yaml_node : the YAML node to extract information from
     * @param param : (output) the ConfigParameter, which gets set by the function
    */
    void _extractParameter(const std::string& param_name, const YAML::Node& yaml_node, ConfigParameter<Eigen::Vector4d>& param, const std::optional<Eigen::Vector4d>& default_value = std::nullopt)
    {
        // set the name field of the ConfigParameter
        param.name = param_name;
        try
        {
            if (yaml_node[param_name].Type() != YAML::NodeType::Null)
            {
                // make sure the node is an array of exactly 4 values
                if (yaml_node[param_name].size() == 4)
                {
                    // if we get here, the parameter exists and it is not null
                    // so, set the value of the ConfigParameter
                    param.value = Eigen::Vector4d({ yaml_node[param_name][0].as<double>(), 
                                                    yaml_node[param_name][1].as<double>(),
                                                    yaml_node[param_name][2].as<double>(),
                                                    yaml_node[param_name][3].as<double>() });
                    return;
                }
                else
                {
                    std::cerr << "Expected exactly 4 values for the parameter " << param_name << std::endl;
                }
                
            }
            else
            {
                // parameter in YAML node eixsts, but is null
                std::cerr << "Parameter with name " << param_name << " is null (did you forget to set it?)" << std::endl;
            }
        }
        catch(const std::exception& e)
        {
            // parameter in YAML does not exist
            std::cerr << e.what() << '\n';
        }

        // if we get to here, there was an issue with the parameter, so set the default
        param.value = default_value;

        if (default_value.has_value())
        {
            std::cout << "Setting parameter " << param_name << " to default value of " << default_value.value() << std::endl;
        }
        
    }

    protected:
    /** Name parameter */
    ConfigParameter<std::string> _name;    
};

#endif // __CONFIG_HPP