#ifndef __CONFIG_HPP
#define __CONFIG_HPP

#include "yaml-cpp/yaml.h"

#include "common/types.hpp"
#include "common/colors.hpp"

#include <iostream>
#include <type_traits>
#include <optional>

// Primary template - defaults to false
template<typename T>
struct is_optional : std::false_type {};

// Specialization for std::optional<U> - true for any optional type
template<typename U>
struct is_optional<std::optional<U>> : std::true_type {};

namespace Config
{

/** Simple templated struct that stores a parameter from a YAML config.
 * The name field is the name of the YAML parameter.
 * The value is the value of the YAML parameter. Defaults must be provided for parameters.
 */
template <typename T>
struct ConfigParameter
{
    std::string name;
    T value;

    ConfigParameter(const T& default_value)
        : value(default_value)
    {}
};

// template <typename T>
// struct ConfigParameter;

template<typename T>
struct ConfigParameter<std::optional<T>>
{
    std::string name;
    std::optional<T> value;

    ConfigParameter(const T& default_value)
        : value(default_value)
    {}

    ConfigParameter()
        : value(std::nullopt)
    {}
};

// template<typename T>
// struct ConfigParameter<T>
// {
//     std::string name;
//     T value;

//     ConfigParameter(const T& default_value)
//         : value(default_value)
//     {}
// };

/** A class that represents a YAML node, which at its simplest is just the name of the object.
 * The Config object is increasingly specialized by Derived classes to incorporate more options/parameters.
 * Provides some helper functions to do proper error checking on a YAML config file which handles missing or null parameters gracefully.
 */
class Config
{
    friend class MeshObjectConfig;

    public:
    /** Default constructor - sets everything to defaults */
    explicit Config() {}

    /** Creates a Config from a YAML node, which only consists of a name.
     * @param node : the YAML node (i.e. dictionary of key-value pairs) that information is pulled from
     */
    explicit Config(const YAML::Node& node)
    {
        // load the name parameter
        _extractParameter("name", node, _name);

        // load nerve configuration parameters (optional)
        _extractParameter("nerve-enable", node, _nerve_enable);
        _extractParameter("nerve-stretch-enable", node, _nerve_stretch_enable);
        _extractParameter("nerve-bending-enable", node, _nerve_bending_enable);
        _extractParameter("nerve-mesh-file", node, _nerve_mesh_file);
        _extractParameter("nerve-physical-group", node, _nerve_physical_group);
        _extractParameter("nerve-stretch-alpha", node, _nerve_stretch_alpha);
        _extractParameter("nerve-bending-alpha", node, _nerve_bending_alpha);
        
        // load constraint-type-specific nerve control parameters (optional)
        _extractParameter("stable-neohookean-nerve-stretch-enable", node, _stable_neohookean_nerve_stretch_enable);
        _extractParameter("stable-neohookean-nerve-bending-enable", node, _stable_neohookean_nerve_bending_enable);
        _extractParameter("stable-neohookean-combined-nerve-stretch-enable", node, _stable_neohookean_combined_nerve_stretch_enable);
        _extractParameter("stable-neohookean-combined-nerve-bending-enable", node, _stable_neohookean_combined_nerve_bending_enable);
        
        // load nerve-tumor adhesion parameters (optional)
        _extractParameter("nerve-tumor-adhesion-enable", node, _nerve_tumor_adhesion_enable);
        _extractParameter("nerve-tumor-adhesion-distance-window", node, _nerve_tumor_adhesion_distance_window);
        _extractParameter("nerve-tumor-adhesion-alpha", node, _nerve_tumor_adhesion_alpha);
        _extractParameter("nerve-tumor-adhesion-break-ratio", node, _nerve_tumor_adhesion_break_ratio);
        
        // Legacy parameters (kept for backward compatibility but not used in new logic)
        _extractParameter("nerve-tumor-adhesion-target-gap", node, _nerve_tumor_adhesion_target_gap);
        _extractParameter("nerve-tumor-adhesion-bond-distance", node, _nerve_tumor_adhesion_bond_distance);
        _extractParameter("nerve-tumor-adhesion-break-distance", node, _nerve_tumor_adhesion_break_distance);
        
        // load inter-deform-deform adhesion parameters (optional)
        _extractParameter("inter-deform-adhesion-enable", node, _inter_deform_adhesion_enable);
        _extractParameter("inter-deform-adhesion-interaction-type", node, _inter_deform_adhesion_interaction_type);
        _extractParameter("inter-deform-adhesion-rest-gap", node, _inter_deform_adhesion_rest_gap);
        _extractParameter("inter-deform-adhesion-break-ratio", node, _inter_deform_adhesion_break_ratio);
        _extractParameter("inter-deform-adhesion-alpha", node, _inter_deform_adhesion_alpha);
        _extractParameter("inter-deform-adhesion-bond-distance", node, _inter_deform_adhesion_bond_distance);
        
        // load inter-deform unified-distance curve parameters (optional)
        _extractParameter("inter-deform-unified-alpha", node, _inter_deform_unified_alpha);
        _extractParameter("inter-deform-unified-break-ratio", node, _inter_deform_unified_break_ratio);
        _extractParameter("inter-deform-unified-bond-distance", node, _inter_deform_unified_bond_distance);
        _extractParameter("inter-deform-unified-d-contact", node, _inter_deform_unified_d_contact);
        _extractParameter("inter-deform-unified-d-rest", node, _inter_deform_unified_d_rest);
        _extractParameter("inter-deform-unified-d-neutral-start", node, _inter_deform_unified_d_neutral_start);
        _extractParameter("inter-deform-unified-d-neutral-end", node, _inter_deform_unified_d_neutral_end);
        _extractParameter("inter-deform-unified-d-bond", node, _inter_deform_unified_d_bond);
        _extractParameter("inter-deform-unified-stretch-abs-min", node, _inter_deform_unified_stretch_abs_min);
        
        // load rigid-deform adhesion parameters (optional)
        _extractParameter("rigid-deform-adhesion-enable", node, _rigid_deform_adhesion_enable);
        _extractParameter("rigid-deform-adhesion-interaction-type", node, _rigid_deform_adhesion_interaction_type);
        _extractParameter("rigid-deform-adhesion-rest-gap", node, _rigid_deform_adhesion_rest_gap);
        _extractParameter("rigid-deform-adhesion-break-ratio", node, _rigid_deform_adhesion_break_ratio);
        _extractParameter("rigid-deform-adhesion-alpha", node, _rigid_deform_adhesion_alpha);
        _extractParameter("rigid-deform-adhesion-bond-distance", node, _rigid_deform_adhesion_bond_distance);
        
        // load unified-distance curve parameters (optional)
        _extractParameter("rigid-deform-adhesion-d-contact", node, _rigid_deform_adhesion_d_contact);
        _extractParameter("rigid-deform-adhesion-d-rest", node, _rigid_deform_adhesion_d_rest);
        _extractParameter("rigid-deform-adhesion-d-neutral-start", node, _rigid_deform_adhesion_d_neutral_start);
        _extractParameter("rigid-deform-adhesion-d-neutral-end", node, _rigid_deform_adhesion_d_neutral_end);
        _extractParameter("rigid-deform-adhesion-d-bond", node, _rigid_deform_adhesion_d_bond);
        _extractParameter("rigid-deform-adhesion-stretch-abs-min", node, _rigid_deform_adhesion_stretch_abs_min);

        std::cout << "\nExtracting parameters for object with name " << BOLD << name() << RST << "..." << std::endl;
    }

    /** "Explicit" constructor that does not use a YAML node to set up the Config */
    explicit Config(const std::string& name)
    {
        _name.value = name;
    }

    /** Declare virtual destructor for polymorphism */
    virtual ~Config() = default;

    // Getters
    std::string name() const { return _name.value; }

    // Nerve configuration getters
    bool nerveEnable() const { return _nerve_enable.value.value_or(true); }  // default true
    bool nerveStretchEnable() const { return _nerve_stretch_enable.value.value_or(true); }  // default true
    bool nerveBendingEnable() const { return _nerve_bending_enable.value.value_or(true); }  // default true
    std::string nerveMeshFile() const { return _nerve_mesh_file.value.value_or(""); }  // default empty
    std::string nervePhysicalGroup() const { return _nerve_physical_group.value.value_or("nerve_edge"); }  // default "nerve_edge"
    Real nerveStretchAlpha() const { return _nerve_stretch_alpha.value.value_or(1e-9); }  // default 1e-9 (stiff)
    Real nerveBendingAlpha() const { return _nerve_bending_alpha.value.value_or(1e-9); }  // default 1e-9 (stiff)
    
    // Constraint-type-specific nerve control getters
    bool stableNeohookeanNerveStretchEnable() const { return _stable_neohookean_nerve_stretch_enable.value.value_or(true); }  // default true
    bool stableNeohookeanNerveBendingEnable() const { return _stable_neohookean_nerve_bending_enable.value.value_or(true); }  // default true  
    bool stableNeohookeanCombinedNerveStretchEnable() const { return _stable_neohookean_combined_nerve_stretch_enable.value.value_or(true); }  // default true
    bool stableNeohookeanCombinedNerveBendingEnable() const { return _stable_neohookean_combined_nerve_bending_enable.value.value_or(true); }  // default true
    
    // Nerve-tumor adhesion configuration getters
    bool nerveTumorAdhesionEnable() const { return _nerve_tumor_adhesion_enable.value.value_or(false); }  // default false
    Real nerveTumorAdhesionDistanceWindow() const { return _nerve_tumor_adhesion_distance_window.value.value_or(0.05); }  // 5 cm detection window
    Real nerveTumorAdhesionAlpha() const { return _nerve_tumor_adhesion_alpha.value.value_or(1e-7); }  // compliance
    Real nerveTumorAdhesionBreakRatio() const { return _nerve_tumor_adhesion_break_ratio.value.value_or(1.5); }  // 50% strain breaks bond
    
    // Legacy getters (kept for backward compatibility but not used in new strain-based logic)
    Real nerveTumorAdhesionTargetGap() const { return _nerve_tumor_adhesion_target_gap.value.value_or(1e-4); }  // 0.1 mm  
    Real nerveTumorAdhesionBondDistance() const { return _nerve_tumor_adhesion_bond_distance.value.value_or(3e-4); }  // bond creation threshold
    Real nerveTumorAdhesionBreakDistance() const { return _nerve_tumor_adhesion_break_distance.value.value_or(7e-4); }  // bond break threshold
    
    // Inter-deform-deform adhesion getters
    bool interDeformAdhesionEnable() const { return _inter_deform_adhesion_enable.value.value_or(false); }  // default false
    std::string interDeformAdhesionInteractionType() const { return _inter_deform_adhesion_interaction_type.value.value_or("adhesion"); }  // default "adhesion"
    Real interDeformAdhesionRestGap() const { return _inter_deform_adhesion_rest_gap.value.value_or(0.005); }  // 5 mm rest gap
    Real interDeformAdhesionBreakRatio() const { return _inter_deform_adhesion_break_ratio.value.value_or(1.5); }  // 50% strain breaks bond
    Real interDeformAdhesionAlpha() const { return _inter_deform_adhesion_alpha.value.value_or(1e-6); }  // compliance
    Real interDeformAdhesionBondDistance() const { return _inter_deform_adhesion_bond_distance.value.value_or(0.01); }  // 1 cm bond creation threshold
    
    // Inter-deform unified-distance curve parameters getters
    Real interDeformUnifiedAlpha() const { return _inter_deform_unified_alpha.value.value_or(1e-6); }  // compliance
    Real interDeformUnifiedBreakRatio() const { return _inter_deform_unified_break_ratio.value.value_or(3.0); }  // 200% strain
    Real interDeformUnifiedBondDistance() const { return _inter_deform_unified_bond_distance.value.value_or(0.010); }  // 10mm bond creation threshold
    Real interDeformUnifiedDContact() const { return _inter_deform_unified_d_contact.value.value_or(0.0003); }  // 0.3mm
    Real interDeformUnifiedDRest() const { return _inter_deform_unified_d_rest.value.value_or(0.0015); }  // 1.5mm
    Real interDeformUnifiedDNeutralStart() const { return _inter_deform_unified_d_neutral_start.value.value_or(0.003); }  // 3mm
    Real interDeformUnifiedDNeutralEnd() const { return _inter_deform_unified_d_neutral_end.value.value_or(0.005); }  // 5mm
    Real interDeformUnifiedDBond() const { return _inter_deform_unified_d_bond.value.value_or(0.015); }  // 15mm
    Real interDeformUnifiedStretchAbsMin() const { return _inter_deform_unified_stretch_abs_min.value.value_or(0.003); }  // 3mm

    // Rigid-deform adhesion getters
    bool rigidDeformAdhesionEnable() const { return _rigid_deform_adhesion_enable.value.value_or(false); }  // default false
    std::string rigidDeformAdhesionInteractionType() const { return _rigid_deform_adhesion_interaction_type.value.value_or("adhesion"); }  // default "adhesion"
    Real rigidDeformAdhesionRestGap() const { return _rigid_deform_adhesion_rest_gap.value.value_or(0.005); }  // 5 mm rest gap
    Real rigidDeformAdhesionBreakRatio() const { return _rigid_deform_adhesion_break_ratio.value.value_or(1.5); }  // 50% strain breaks bond
    Real rigidDeformAdhesionAlpha() const { return _rigid_deform_adhesion_alpha.value.value_or(1e-6); }  // compliance
    Real rigidDeformAdhesionBondDistance() const { return _rigid_deform_adhesion_bond_distance.value.value_or(0.01); }  // 1 cm bond creation threshold
    
    // Unified-distance curve parameters getters
    Real rigidDeformAdhesionDContact() const { return _rigid_deform_adhesion_d_contact.value.value_or(0.018); }  // 18mm equilibrium
    Real rigidDeformAdhesionDRest() const { return _rigid_deform_adhesion_d_rest.value.value_or(0.028); }  // 28mm mid-range
    Real rigidDeformAdhesionDNeutralStart() const { return _rigid_deform_adhesion_d_neutral_start.value.value_or(0.034); }  // 34mm transition start
    Real rigidDeformAdhesionDNeutralEnd() const { return _rigid_deform_adhesion_d_neutral_end.value.value_or(0.038); }  // 38mm transition end
    Real rigidDeformAdhesionDBond() const { return _rigid_deform_adhesion_d_bond.value.value_or(0.058); }  // 58mm saturation
    Real rigidDeformAdhesionStretchAbsMin() const { return _rigid_deform_adhesion_stretch_abs_min.value.value_or(0.005); }  // 5mm intrinsic toughness

    protected:

    /** Extracts a key-value pair from the YAML node.
     * If the parameter doesn't exist, the ConfigParameter value is a null optional.
     * @param param_name : the name of the parameter to get from the YAML file
     * @param yaml_node : the YAML node to extract information from
     * @param param : (output) the ConfigParameter, which gets set by the function. At the very least, the param name is set and if there is no error, the value gets set too.
     * 
     */
    template<typename T>
    static void _extractParameter(const std::string& param_name, const YAML::Node& yaml_node, ConfigParameter<T>& param)
    {
        // set the name field of the ConfigParameter
        param.name = param_name;

        try 
        {
            if (yaml_node[param_name].Type() != YAML::NodeType::Null)
            {
                // if we get here, the parameter exists, and it is not null
                // so, set the value of the ConfigParameter and we're done!
                if constexpr (is_optional<T>::value)
                {
                    param.value = yaml_node[param_name].as<typename T::value_type>();
                    std::cout << "Extracted optional type: " << param.value.value() << std::endl;
                }
                else
                {
                    param.value = yaml_node[param_name].as<T>();
                }
                
                return;
            }
            else
            {
                // parameter in YAML node exists, but is null
                std::cerr << KYEL << "\tParameter with name " << BOLD << param_name << RST << KYEL << " is null (did you forget to set it?)" << RST << std::endl;
            }
        }
        catch (const std::exception& e)
        {
            // parameter in YAML does not exist
            std::cerr << KYEL << "\tParameter " << BOLD << param_name << RST << KYEL << " not found for this object, or is not of the expected type." << RST << std::endl;
        }

        if constexpr (is_optional<T>::value)
        {
            // if we get to here, the parameter was not specified so just use the default value (which should already be set as the value of the ConfigParameter)
            std::cout << "\t Optional parameter " << BOLD << param_name << RST << " not specified. " << std::endl;
        }
        else
        {
            // if we get to here, the parameter was not specified so just use the default value (which should already be set as the value of the ConfigParameter)
            std::cout << "\tSetting parameter " << BOLD << param_name << RST << " to default value of " << BOLD << param.value << RST << std::endl;
        }
        
    }

    template<typename K, typename T>
    static void _extractParameterWithOptions(const std::string& param_name, const YAML::Node& yaml_node, ConfigParameter<T>& param, const std::map<K, T>& options)
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
                    std::cerr << KRED << BOLD << "\t" << key << RST << KRED << " is not a valid option for parameter " << BOLD << param_name << RST << KRED << "! Valid options are ";
                    for (const auto& [k, v] : options)
                    {
                        std::cerr << k << "; ";
                    }
                    std::cerr << RST << std::endl;
                }
            }
        }
        catch(const std::exception& e)
        {
            // parameter in YAML does not exist
            std::cerr << KYEL << "\tParameter " << BOLD << param_name << RST << KYEL << " not found for this object, or is not of the expected type." << RST << std::endl;
        }

        // if we get to here, the parameter was not specified so set the default value
        K key;
        for (auto &i : options) {
            if (i.second == param.value) {
                key = i.first;
                break;
            }
        }
        std::cout << "\tSetting parameter " << BOLD << param_name << RST << " to default value of " << BOLD << key << RST << std::endl;
    }

    /** Extracts a vector of arbitrary length from YAML node.
     * @param param_name : the name of the vector parameter
     * @param yaml_node : the YAML node to extract information from
     * @param param : (output) the ConfigParameter, which gets set by the function
     */
    template <typename T>
    static void _extractParameter(const std::string& param_name, const YAML::Node& yaml_node, ConfigParameter<std::vector<T>>& param)
    {
        // set the name field of the ConfigParameter
        param.name = param_name;

        try 
        {
            if (yaml_node[param_name].Type() != YAML::NodeType::Null)
            {
                // if we get here, the parameter exists, and it is not null
                // so, set the value of the ConfigParameter and we're done!
                for (unsigned i = 0; i < yaml_node[param_name].size(); i++)
                    param.value.push_back(yaml_node[param_name][i].as<T>());
                
                return;
            }
            else
            {
                // parameter in YAML node exists, but is null
                std::cerr << KYEL << "\tParameter with name " << BOLD << param_name << RST << KYEL << " is null (did you forget to set it?)" << RST << std::endl;
            }
        }
        catch (const std::exception& e)
        {
            // parameter in YAML does not exist
            std::cerr << KYEL << "\tParameter " << BOLD << param_name << RST << KYEL << " not found for this object, or is not of the expected type." << RST << std::endl;
        }

        // if we get to here, the parameter was not specified so just use the default value (which should already be set as the value of the ConfigParameter)
        std::cout << "\tSetting parameter " << BOLD << param_name << RST << " to default value of " << BOLD << "{";
        for (const auto& v : param.value)   std::cout << v << ",";
        std::cout << "}" << RST << std::endl;
    }

    /** Extracts a 3-vector from YAML node as an Vec3r
     * If the parameter doesn't exist, the ConfigParameter value is a null optional.
     * @param param_name : the name of the 3-Vector parameter to get from the YAML file
     * @param yaml_node : the YAML node to extract information from
     * @param param : (output) the ConfigParameter, which gets set by the function
     */
    static void _extractParameter(const std::string& param_name, const YAML::Node& yaml_node, ConfigParameter<Vec3r>& param)
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
                    param.value = Vec3r({ yaml_node[param_name][0].as<Real>(), 
                                                    yaml_node[param_name][1].as<Real>(),
                                                    yaml_node[param_name][2].as<Real>() });
                    return;
                }
                else
                {
                    std::cerr << KRED << "\tExpected exactly 3 values for the parameter " << BOLD << param_name << RST << std::endl;
                }
                
            }
            else
            {
                // parameter in YAML node exists, but is null
                std::cerr << "\tParameter with name " << param_name << " is null (did you forget to set it?)" << RST << std::endl;
            }
        }
        catch(const std::exception& e)
        {
            // parameter in YAML does not exist
            std::cerr << KYEL << "\tParameter " << BOLD << param_name << RST << KYEL << " not found for this object, or is not of the expected type." << RST << std::endl;
        }

        // if we get to here, there was an issue with the parameter, so just use the default value
        std::cout << "\tSetting parameter " << BOLD << param_name << RST << " to default value of " << BOLD << 
            "(" << param.value[0] << ", " << param.value[1] << ", " << param.value[2] << ")" << RST << std::endl;
        
    }

    /** Extracts an optional 3-vector from YAML node as an Vec3r
     * If the parameter doesn't exist, the ConfigParameter value is a null optional.
     * @param param_name : the name of the 3-Vector parameter to get from the YAML file
     * @param yaml_node : the YAML node to extract information from
     * @param param : (output) the ConfigParameter, which gets set by the function
     */
    static void _extractParameter(const std::string& param_name, const YAML::Node& yaml_node, ConfigParameter<std::optional<Vec3r>>& param)
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
                    param.value = Vec3r({ yaml_node[param_name][0].as<Real>(), 
                                                    yaml_node[param_name][1].as<Real>(),
                                                    yaml_node[param_name][2].as<Real>() });
                    return;
                }
                else
                {
                    std::cerr << KRED << "\tExpected exactly 3 values for the parameter " << BOLD << param_name << RST << std::endl;
                }
                
            }
            else
            {
                // parameter in YAML node exists, but is null
                std::cerr << "\tParameter with name " << param_name << " is null (did you forget to set it?)" << RST << std::endl;
            }
        }
        catch(const std::exception& e)
        {
            // parameter in YAML does not exist
            std::cerr << KYEL << "\tParameter " << BOLD << param_name << RST << KYEL << " not found for this object, or is not of the expected type." << RST << std::endl;
        }

        // if we get to here, the parameter was not specified so just use the default value (which should already be set as the value of the ConfigParameter)
        std::cout << "\t Optional parameter " << BOLD << param_name << RST << " not specified. " << std::endl;
        
    }

    /** Extracts an optional vector of 3-vectors from YAML node as a vector of Vec3r's
     * If the parameter doesn't exist, the ConfigParameter value is a null optional.
     * @param param_name : the name of the 3-Vector parameter to get from the YAML file
     * @param yaml_node : the YAML node to extract information from
     * @param param : (output) the ConfigParameter, which gets set by the function
     */
    static void _extractParameter(const std::string& param_name, const YAML::Node& yaml_node, ConfigParameter<std::optional<std::vector<Vec3r>>>& param)
    {
        // set the name field of the ConfigParameter
        param.name = param_name;
        param.value = std::vector<Vec3r>();
        try
        {
            if (yaml_node[param_name].Type() != YAML::NodeType::Null)
            {
                // if we get here, the parameter exists, and it is not null
                // so, set the value of the ConfigParameter and we're done!
                for (unsigned i = 0; i < yaml_node[param_name].size(); i++)
                {
                    if (yaml_node[param_name][i].size() == 3)
                    {
                        Vec3r vec(  yaml_node[param_name][i][0].as<Real>(), 
                                    yaml_node[param_name][i][1].as<Real>(),
                                    yaml_node[param_name][i][2].as<Real>() );
                        param.value.value().push_back(vec);
                    }
                    else
                    {
                        std::cerr << KRED << "\tEntry " << i << " in " << BOLD << param_name << " does not have exactly 3 values!" << RST << std::endl;
                    }
                }

                return;
                
            }
            else
            {
                // parameter in YAML node exists, but is null
                std::cerr << "\tParameter with name " << param_name << " is null (did you forget to set it?)" << RST << std::endl;
            }
        }
        catch(const std::exception& e)
        {
            // parameter in YAML does not exist
            std::cerr << KYEL << "\tParameter " << BOLD << param_name << RST << KYEL << " not found for this object, or is not of the expected type." << RST << std::endl;
        }

        // if we get to here, the parameter was not specified so just use the default value (which should already be set as the value of the ConfigParameter)
        std::cout << "\t Optional parameter " << BOLD << param_name << RST << " not specified. " << std::endl;
        
    }

    /** Extracts a 4-vector from YAML node as an Vec4r 
     * If the parameter doesn't exist, the ConfigParameter value is a null optional.
     * @param param_name : the name of the 4-vector parameter to get from the YAML file
     * @param yaml_node : the YAML node to extract information from
     * @param param : (output) the ConfigParameter, which gets set by the function
    */
    static void _extractParameter(const std::string& param_name, const YAML::Node& yaml_node, ConfigParameter<Vec4r>& param)
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
                    param.value = Vec4r({ yaml_node[param_name][0].as<Real>(), 
                                                    yaml_node[param_name][1].as<Real>(),
                                                    yaml_node[param_name][2].as<Real>(),
                                                    yaml_node[param_name][3].as<Real>() });
                    return;
                }
                else
                {
                    std::cerr << KRED << "\tExpected exactly 4 values for the parameter " << BOLD << param_name << RST << std::endl;
                }
                
            }
            else
            {
                // parameter in YAML node eixsts, but is null
                std::cerr << KYEL << "\tParameter with name " << BOLD << param_name << RST << KYEL << " is null (did you forget to set it?)" << RST << std::endl;
            }
        }
        catch(const std::exception& e)
        {
            // parameter in YAML does not exist
            std::cerr << KYEL << "\tParameter " << BOLD << param_name << RST << KYEL << " not found for this object, or is not of the expected type." << RST << std::endl;
        }

        // if we get to here, there was an issue with the parameter, so set the default
        std::cout << "\tSetting parameter " << BOLD << param_name << RST << " to default value of " << BOLD <<
            "(" << param.value[0] << ", " << param.value[1] << ", " << param.value[2] << ", " << param.value[3] << ")" << RST << std::endl;
        
    }

    protected:
    /** Name parameter */
    ConfigParameter<std::string> _name = ConfigParameter<std::string>("");    

    /** Nerve configuration parameters */
    ConfigParameter<std::optional<bool>> _nerve_enable = ConfigParameter<std::optional<bool>>(true);
    ConfigParameter<std::optional<bool>> _nerve_stretch_enable = ConfigParameter<std::optional<bool>>(true);
    ConfigParameter<std::optional<bool>> _nerve_bending_enable = ConfigParameter<std::optional<bool>>(true);
    ConfigParameter<std::optional<std::string>> _nerve_mesh_file = ConfigParameter<std::optional<std::string>>("");
    ConfigParameter<std::optional<std::string>> _nerve_physical_group = ConfigParameter<std::optional<std::string>>("nerve_edge");
    ConfigParameter<std::optional<Real>> _nerve_stretch_alpha = ConfigParameter<std::optional<Real>>(1e-9);  // compliance for stretch
    ConfigParameter<std::optional<Real>> _nerve_bending_alpha = ConfigParameter<std::optional<Real>>(1e-9);  // compliance for bending
    
    /** Constraint-type-specific nerve control parameters */
    ConfigParameter<std::optional<bool>> _stable_neohookean_nerve_stretch_enable = ConfigParameter<std::optional<bool>>(true);
    ConfigParameter<std::optional<bool>> _stable_neohookean_nerve_bending_enable = ConfigParameter<std::optional<bool>>(true);
    ConfigParameter<std::optional<bool>> _stable_neohookean_combined_nerve_stretch_enable = ConfigParameter<std::optional<bool>>(true);
    ConfigParameter<std::optional<bool>> _stable_neohookean_combined_nerve_bending_enable = ConfigParameter<std::optional<bool>>(true);
    
    /** Nerve-tumor adhesion configuration parameters */
    ConfigParameter<std::optional<bool>> _nerve_tumor_adhesion_enable = ConfigParameter<std::optional<bool>>(false);
    ConfigParameter<std::optional<Real>> _nerve_tumor_adhesion_distance_window = ConfigParameter<std::optional<Real>>(0.05);  // 5 cm
    ConfigParameter<std::optional<Real>> _nerve_tumor_adhesion_alpha = ConfigParameter<std::optional<Real>>(1e-7);
    ConfigParameter<std::optional<Real>> _nerve_tumor_adhesion_break_ratio = ConfigParameter<std::optional<Real>>(1.5);  // 50% strain
    
    // Legacy parameters (backward compatibility)
    ConfigParameter<std::optional<Real>> _nerve_tumor_adhesion_target_gap = ConfigParameter<std::optional<Real>>(1e-4);
    ConfigParameter<std::optional<Real>> _nerve_tumor_adhesion_bond_distance = ConfigParameter<std::optional<Real>>(3e-4);
    ConfigParameter<std::optional<Real>> _nerve_tumor_adhesion_break_distance = ConfigParameter<std::optional<Real>>(7e-4);
    
    /** Inter-deform-deform adhesion configuration parameters */
    ConfigParameter<std::optional<bool>> _inter_deform_adhesion_enable = ConfigParameter<std::optional<bool>>(false);
    ConfigParameter<std::optional<std::string>> _inter_deform_adhesion_interaction_type = ConfigParameter<std::optional<std::string>>("adhesion");  // "adhesion" or "unified-distance"
    ConfigParameter<std::optional<Real>> _inter_deform_adhesion_rest_gap = ConfigParameter<std::optional<Real>>(0.005);  // 5 mm
    ConfigParameter<std::optional<Real>> _inter_deform_adhesion_break_ratio = ConfigParameter<std::optional<Real>>(1.5);  // 50% strain
    ConfigParameter<std::optional<Real>> _inter_deform_adhesion_alpha = ConfigParameter<std::optional<Real>>(1e-6);
    ConfigParameter<std::optional<Real>> _inter_deform_adhesion_bond_distance = ConfigParameter<std::optional<Real>>(0.01);  // 1 cm
    
    /** Inter-deform unified-distance curve parameters */
    ConfigParameter<std::optional<Real>> _inter_deform_unified_alpha = ConfigParameter<std::optional<Real>>(1e-6);
    ConfigParameter<std::optional<Real>> _inter_deform_unified_break_ratio = ConfigParameter<std::optional<Real>>(3.0);  // 200% strain
    ConfigParameter<std::optional<Real>> _inter_deform_unified_bond_distance = ConfigParameter<std::optional<Real>>(0.010);  // 10mm
    ConfigParameter<std::optional<Real>> _inter_deform_unified_d_contact = ConfigParameter<std::optional<Real>>(0.0003);  // 0.3mm
    ConfigParameter<std::optional<Real>> _inter_deform_unified_d_rest = ConfigParameter<std::optional<Real>>(0.0015);  // 1.5mm
    ConfigParameter<std::optional<Real>> _inter_deform_unified_d_neutral_start = ConfigParameter<std::optional<Real>>(0.003);  // 3mm
    ConfigParameter<std::optional<Real>> _inter_deform_unified_d_neutral_end = ConfigParameter<std::optional<Real>>(0.005);  // 5mm
    ConfigParameter<std::optional<Real>> _inter_deform_unified_d_bond = ConfigParameter<std::optional<Real>>(0.015);  // 15mm
    ConfigParameter<std::optional<Real>> _inter_deform_unified_stretch_abs_min = ConfigParameter<std::optional<Real>>(0.003);  // 3mm
    
    /** Rigid-deform adhesion configuration parameters */
    ConfigParameter<std::optional<bool>> _rigid_deform_adhesion_enable = ConfigParameter<std::optional<bool>>(false);
    ConfigParameter<std::optional<std::string>> _rigid_deform_adhesion_interaction_type = ConfigParameter<std::optional<std::string>>("adhesion");  // "adhesion" or "unified-distance"
    ConfigParameter<std::optional<Real>> _rigid_deform_adhesion_rest_gap = ConfigParameter<std::optional<Real>>(0.005);  // 5 mm
    ConfigParameter<std::optional<Real>> _rigid_deform_adhesion_break_ratio = ConfigParameter<std::optional<Real>>(1.5);  // 50% strain
    ConfigParameter<std::optional<Real>> _rigid_deform_adhesion_alpha = ConfigParameter<std::optional<Real>>(1e-6);
    ConfigParameter<std::optional<Real>> _rigid_deform_adhesion_bond_distance = ConfigParameter<std::optional<Real>>(0.01);  // 1 cm
    
    /** Unified-distance curve parameters (for "unified-distance" interaction type) */
    ConfigParameter<std::optional<Real>> _rigid_deform_adhesion_d_contact = ConfigParameter<std::optional<Real>>(0.018);        // 18mm equilibrium
    ConfigParameter<std::optional<Real>> _rigid_deform_adhesion_d_rest = ConfigParameter<std::optional<Real>>(0.028);           // 28mm mid-range
    ConfigParameter<std::optional<Real>> _rigid_deform_adhesion_d_neutral_start = ConfigParameter<std::optional<Real>>(0.034);  // 34mm transition start
    ConfigParameter<std::optional<Real>> _rigid_deform_adhesion_d_neutral_end = ConfigParameter<std::optional<Real>>(0.038);    // 38mm transition end
    ConfigParameter<std::optional<Real>> _rigid_deform_adhesion_d_bond = ConfigParameter<std::optional<Real>>(0.058);           // 58mm saturation
    ConfigParameter<std::optional<Real>> _rigid_deform_adhesion_stretch_abs_min = ConfigParameter<std::optional<Real>>(0.005);  // 5mm intrinsic toughness
};


} // namespace Config

#endif // __CONFIG_HPP