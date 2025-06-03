#ifndef __VIRTUOSO_ARM_CONFIG_HPP
#define __VIRTUOSO_ARM_CONFIG_HPP

#include "config/simobject/ObjectConfig.hpp"

#include "simobject/VirtuosoArm.hpp"

namespace Config
{

class VirtuosoArmConfig : public ObjectConfig
{
    public:
    static std::optional<double>& DEFAULT_OT_DIAMETER() { static std::optional<double> ot_dia(0.15); return ot_dia; }
    static std::optional<double>& DEFUALT_OT_R_CURVATURE() { static std::optional<double> ot_curv(0.15); return ot_curv; }
    static std::optional<double>& DEFAULT_IT_DIAMETER() { static std::optional<double> it_dia(0.1); return it_dia; }
    static std::optional<double>& DEFAULT_IT_INITIAL_TRANSLATION() { static std::optional<double> it_trans(0.05); return it_trans; }
    static std::optional<double>& DEFAULT_IT_INITIAL_ROTATION() { static std::optional<double> it_rot(0); return it_rot; }
    static std::optional<double>& DEFAULT_OT_INITIAL_TRANSLATION() { static std::optional<double> ot_trans(0.1); return ot_trans; }
    static std::optional<double>& DEFAULT_OT_INITIAL_ROTATION() { static std::optional<double> ot_rot(0); return ot_rot; }
    static std::optional<double>& DEFAULT_OT_DISTAL_STRAIGHT_LENGTH() { static std::optional<double> ot_l(0); return ot_l; }
    static std::optional<Sim::VirtuosoArm::ToolType>& DEFAULT_TOOL_TYPE() { static std::optional<Sim::VirtuosoArm::ToolType> tool_type(Sim::VirtuosoArm::ToolType::GRASPER); return tool_type; }
    /** Static predifined options for the tool type. Maps strings to the ToolType enum. */
    static std::map<std::string, Sim::VirtuosoArm::ToolType> TOOL_TYPE_OPTIONS()
    {
        static std::map<std::string, Sim::VirtuosoArm::ToolType> tool_type_options{
            {"spatula", Sim::VirtuosoArm::ToolType::SPATULA},
            {"grasper", Sim::VirtuosoArm::ToolType::GRASPER},
            {"cautery", Sim::VirtuosoArm::ToolType::CAUTERY}
        };
        return tool_type_options;
    }

    static std::optional<Eigen::Vector3d>& DEFAULT_BASE_INITIAL_POSITION() { static std::optional<Eigen::Vector3d> pos({0.0, 0.0, 0.0}); return pos; }
    static std::optional<Eigen::Vector3d>& DEFAULT_BASE_INITIAL_ROTATION() { static std::optional<Eigen::Vector3d> rot({0.0, 0.0, 0.0}); return rot; }


    explicit VirtuosoArmConfig(const YAML::Node& node)
        : ObjectConfig(node)
    {
        _extractParameter("inner-tube-diameter", node, _it_diameter, DEFAULT_IT_DIAMETER());
        _extractParameter("inner-tube-translation", node, _it_initial_translation, DEFAULT_IT_INITIAL_TRANSLATION());
        _extractParameter("inner-tube-rotation", node, _it_initial_rotation, DEFAULT_IT_INITIAL_ROTATION());

        _extractParameter("outer-tube-diameter", node, _ot_diameter, DEFAULT_OT_DIAMETER());
        _extractParameter("outer-tube-radius-of-curvature", node, _ot_r_curvature, DEFUALT_OT_R_CURVATURE());
        _extractParameter("outer-tube-translation", node, _ot_initial_translation, DEFAULT_OT_INITIAL_TRANSLATION());
        _extractParameter("outer-tube-rotation", node, _ot_initial_rotation, DEFAULT_OT_INITIAL_ROTATION());
        _extractParameter("outer-tube-distal-straight-length", node, _ot_distal_straight_length, DEFAULT_OT_DISTAL_STRAIGHT_LENGTH());

        _extractParameterWithOptions("tool-type", node, _tool_type, TOOL_TYPE_OPTIONS(), DEFAULT_TOOL_TYPE());

        _extractParameter("base-position", node, _base_initial_position, DEFAULT_BASE_INITIAL_POSITION());
        _extractParameter("base-rotation", node, _base_initial_rotation, DEFAULT_BASE_INITIAL_ROTATION());
    }

    explicit VirtuosoArmConfig( const std::string& name, 
        const Vec3r& initial_pos, const Vec3r& initial_rot, const Vec3r& initial_velocity, bool collisions, bool graphics_only,
        double ot_dia, double ot_r_curve, double ot_d_s_length, double it_dia,
        double ot_rot, double ot_trans, double it_rot, double it_trans,
        Sim::VirtuosoArm::ToolType tool_type
    )
        : ObjectConfig(name, initial_pos, initial_rot, initial_velocity, collisions, graphics_only)
    {
        _ot_diameter.value = ot_dia;
        _ot_r_curvature.value = ot_r_curve;
        _ot_distal_straight_length.value = ot_d_s_length;
        _it_diameter.value = it_dia;

        _ot_initial_rotation.value = ot_rot;
        _ot_initial_translation.value = ot_trans;
        _it_initial_rotation.value = it_rot;
        _it_initial_translation.value = it_trans;

        _tool_type.value = tool_type; 

        _base_initial_position.value = initial_pos;
        _base_initial_rotation.value = initial_rot;
    }

    // Getters and setters
    double innerTubeDiameter() const { return _it_diameter.value.value(); }
    double innerTubeInitialTranslation() const { return _it_initial_translation.value.value(); }
    double innerTubeInitialRotation() const { return _it_initial_rotation.value.value(); }
    double outerTubeDiameter() const { return _ot_diameter.value.value(); }
    double outerTubeRadiusOfCurvature() const { return _ot_r_curvature.value.value(); }
    double outerTubeInitialTranslation() const { return _ot_initial_translation.value.value(); }
    double outerTubeInitialRotation() const { return _ot_initial_rotation.value.value(); }
    double outerTubeDistalStraightLength() const { return _ot_distal_straight_length.value.value(); }

    Sim::VirtuosoArm::ToolType toolType() const { return _tool_type.value.value(); }

    Eigen::Vector3d baseInitialPosition() const { return _base_initial_position.value.value(); }
    Eigen::Vector3d baseInitialRotation() const { return _base_initial_rotation.value.value(); }

    protected:
    ConfigParameter<Real> _it_diameter;
    ConfigParameter<Real> _it_initial_translation;
    ConfigParameter<Real> _it_initial_rotation;

    ConfigParameter<Real> _ot_diameter;
    ConfigParameter<Real> _ot_r_curvature;
    ConfigParameter<Real> _ot_distal_straight_length;
    ConfigParameter<Real> _ot_initial_translation;
    ConfigParameter<Real> _ot_initial_rotation;

    ConfigParameter<Sim::VirtuosoArm::ToolType> _tool_type;

    ConfigParameter<Eigen::Vector3d> _base_initial_position;
    ConfigParameter<Eigen::Vector3d> _base_initial_rotation;

};

} // namespace Config

#endif // __VIRTUOSO_ARM_CONFIG_HPP