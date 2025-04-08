#ifndef __VIRTUOSO_ROBOT_CONFIG_HPP
#define __VIRTUOSO_ROBOT_CONFIG_HPP

#include "config/ObjectConfig.hpp"
#include "config/VirtuosoArmConfig.hpp"

class VirtuosoRobotConfig : public ObjectConfig
{
    public:
    static std::optional<double>& DEFAULT_ENDOSCOPE_DIAMETER() { static std::optional<double> e_dia(8.67e-3); return e_dia; }
    static std::optional<double>& DEFAULT_ENDOSCOPE_LENGTH() { static std::optional<double> e_l(0.05); return e_l; }
    static std::optional<double>& DEFAULT_ARM_SEPARATION_DISTANCE() { static std::optional<double> dist(3.05e-3); return dist; }
    static std::optional<double>& DEFAULT_OPTIC_VERTICAL_DISTANCE() { static std::optional<double> dist(3.24e-3); return dist; }
    static std::optional<double>& DEFAULT_OPTIC_TILT() { static std::optional<double> tilt_deg(30); return tilt_deg; }

    explicit VirtuosoRobotConfig(const YAML::Node& node)
        : ObjectConfig(node)
    {
        _extractParameter("endoscope-diameter", node, _endoscope_diameter, DEFAULT_ENDOSCOPE_DIAMETER());
        _extractParameter("endoscope-length", node, _endoscope_length, DEFAULT_ENDOSCOPE_LENGTH());
        _extractParameter("arm-separation-distance", node, _arm_separation_dist, DEFAULT_ARM_SEPARATION_DISTANCE());
        _extractParameter("optic-vertical-distance", node, _optic_vertical_dist, DEFAULT_OPTIC_VERTICAL_DISTANCE());
        _extractParameter("optic-tilt", node, _optic_tilt, DEFAULT_OPTIC_TILT());

        // create a MeshObject for each object specified in the YAML file
        for (const auto& obj_node : node["arms"])
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
            std::unique_ptr<VirtuosoArmConfig> config;
            if (type == "VirtuosoArm")                   
                config = std::make_unique<VirtuosoArmConfig>(obj_node);
            else
            {
                std::cerr << "Unknown type of object! \"" << type << "\" is not a type of VirtuosoArm." << std::endl;
                assert(0);
            }

            _arm_configs.push_back(std::move(config));
            
        }
    }

    const std::vector<std::unique_ptr<VirtuosoArmConfig>>& armConfigs() const { return _arm_configs; }
    double endoscopeDiameter() const { return _endoscope_diameter.value.value(); }
    double endoscopeLength() const { return _endoscope_length.value.value(); }
    double armSeparationDistance() const { return _arm_separation_dist.value.value(); }
    double opticVerticalDistance() const { return _optic_vertical_dist.value.value(); }
    double opticTilt() const { return _optic_tilt.value.value(); }

    protected:
    ConfigParameter<double> _endoscope_diameter;
    ConfigParameter<double> _endoscope_length;
    ConfigParameter<double> _arm_separation_dist;
    ConfigParameter<double> _optic_vertical_dist;
    ConfigParameter<double> _optic_tilt;

    std::vector<std::unique_ptr<VirtuosoArmConfig>> _arm_configs;
};

#endif // __VIRTUOSO_ROBOT_CONFIG_HPP