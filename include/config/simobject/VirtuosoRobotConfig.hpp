#ifndef __VIRTUOSO_ROBOT_CONFIG_HPP
#define __VIRTUOSO_ROBOT_CONFIG_HPP

#include "config/simobject/ObjectConfig.hpp"
#include "config/simobject/VirtuosoArmConfig.hpp"

namespace Config
{

class VirtuosoRobotConfig : public ObjectConfig
{
    public:
    static std::optional<Real>& DEFAULT_ENDOSCOPE_DIAMETER() { static std::optional<Real> e_dia(8.67e-3); return e_dia; }
    static std::optional<Real>& DEFAULT_ENDOSCOPE_LENGTH() { static std::optional<Real> e_l(0.05); return e_l; }
    static std::optional<Real>& DEFAULT_ARM_SEPARATION_DISTANCE() { static std::optional<Real> dist(3.05e-3); return dist; }
    static std::optional<Real>& DEFAULT_OPTIC_VERTICAL_DISTANCE() { static std::optional<Real> dist(3.24e-3); return dist; }
    static std::optional<Real>& DEFAULT_OPTIC_TILT() { static std::optional<Real> tilt_deg(30); return tilt_deg; }

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
    Real endoscopeDiameter() const { return _endoscope_diameter.value.value(); }
    Real endoscopeLength() const { return _endoscope_length.value.value(); }
    Real armSeparationDistance() const { return _arm_separation_dist.value.value(); }
    Real opticVerticalDistance() const { return _optic_vertical_dist.value.value(); }
    Real opticTilt() const { return _optic_tilt.value.value(); }

    protected:
    ConfigParameter<Real> _endoscope_diameter;
    ConfigParameter<Real> _endoscope_length;
    ConfigParameter<Real> _arm_separation_dist;
    ConfigParameter<Real> _optic_vertical_dist;
    ConfigParameter<Real> _optic_tilt;

    std::vector<std::unique_ptr<VirtuosoArmConfig>> _arm_configs;
};

} // namespace Config

#endif // __VIRTUOSO_ROBOT_CONFIG_HPP