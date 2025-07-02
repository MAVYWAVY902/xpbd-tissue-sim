#ifndef __VIRTUOSO_ARM_CONFIG_HPP
#define __VIRTUOSO_ARM_CONFIG_HPP

#include "config/simobject/ObjectConfig.hpp"

#include "simobject/VirtuosoArm.hpp"

namespace Sim
{
    class VirtuosoArm;
}

namespace Config
{

class VirtuosoArmConfig : public ObjectConfig
{
    public:
    using ObjectType = Sim::VirtuosoArm;

    public:
    static std::optional<Real>& DEFAULT_OT_OUTER_DIAMETER() { static std::optional<Real> ot_outer_dia(1.56e-3); return ot_outer_dia; }
    static std::optional<Real>& DEFAULT_OT_INNER_DIAMETER() { static std::optional<Real> ot_inner_dia(1.14e-3); return ot_inner_dia; }
    static std::optional<Real>& DEFUALT_OT_R_CURVATURE() { static std::optional<Real> ot_curv(90); return ot_curv; }
    static std::optional<Real>& DEFAULT_IT_OUTER_DIAMETER() { static std::optional<Real> it_outer_dia(1.04e-3); return it_outer_dia; }
    static std::optional<Real>& DEFAULT_IT_INNER_DIAMETER() { static std::optional<Real> it_inner_dia(0.82e-3); return it_inner_dia; }
    static std::optional<Real>& DEFAULT_IT_INITIAL_TRANSLATION() { static std::optional<Real> it_trans(15e-3); return it_trans; }
    static std::optional<Real>& DEFAULT_IT_INITIAL_ROTATION() { static std::optional<Real> it_rot(0); return it_rot; }
    static std::optional<Real>& DEFAULT_OT_INITIAL_TRANSLATION() { static std::optional<Real> ot_trans(10e-3); return ot_trans; }
    static std::optional<Real>& DEFAULT_OT_INITIAL_ROTATION() { static std::optional<Real> ot_rot(0); return ot_rot; }
    static std::optional<Real>& DEFAULT_OT_DISTAL_STRAIGHT_LENGTH() { static std::optional<Real> ot_l(5e-3); return ot_l; }
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
        _extractParameter("inner-tube-outer-diameter", node, _it_outer_diameter, DEFAULT_IT_OUTER_DIAMETER());
        _extractParameter("inner-tube-inner-diameter", node, _it_inner_diameter, DEFAULT_IT_INNER_DIAMETER());
        _extractParameter("inner-tube-translation", node, _it_initial_translation, DEFAULT_IT_INITIAL_TRANSLATION());
        _extractParameter("inner-tube-rotation", node, _it_initial_rotation, DEFAULT_IT_INITIAL_ROTATION());

        _extractParameter("outer-tube-outer-diameter", node, _ot_outer_diameter, DEFAULT_OT_OUTER_DIAMETER());
        _extractParameter("outer-tube-inner-diameter", node, _ot_inner_diameter, DEFAULT_OT_INNER_DIAMETER());
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
        Real ot_outer_dia, Real ot_inner_dia, Real ot_r_curve, Real ot_d_s_length, Real it_outer_dia, Real it_inner_dia,
        Real ot_rot, Real ot_trans, Real it_rot, Real it_trans,
        Sim::VirtuosoArm::ToolType tool_type
    )
        : ObjectConfig(name, initial_pos, initial_rot, initial_velocity, collisions, graphics_only)
    {
        _ot_outer_diameter.value = ot_outer_dia;
        _ot_inner_diameter.value = ot_inner_dia;
        _ot_r_curvature.value = ot_r_curve;
        _ot_distal_straight_length.value = ot_d_s_length;
        _it_outer_diameter.value = it_outer_dia;
        _it_inner_diameter.value = it_inner_dia;

        _ot_initial_rotation.value = ot_rot;
        _ot_initial_translation.value = ot_trans;
        _it_initial_rotation.value = it_rot;
        _it_initial_translation.value = it_trans;

        _tool_type.value = tool_type; 

        _base_initial_position.value = initial_pos;
        _base_initial_rotation.value = initial_rot;
    }

    std::unique_ptr<ObjectType> createObject(const Sim::Simulation* sim) const;

    // Getters and setters
    Real innerTubeOuterDiameter() const { return _it_outer_diameter.value.value(); }
    Real innerTubeInnerDiameter() const { return _it_inner_diameter.value.value(); }
    Real innerTubeInitialTranslation() const { return _it_initial_translation.value.value(); }
    Real innerTubeInitialRotation() const { return _it_initial_rotation.value.value(); }
    Real outerTubeOuterDiameter() const { return _ot_outer_diameter.value.value(); }
    Real outerTubeInnerDiameter() const { return _ot_inner_diameter.value.value(); }
    Real outerTubeRadiusOfCurvature() const { return _ot_r_curvature.value.value(); }
    Real outerTubeInitialTranslation() const { return _ot_initial_translation.value.value(); }
    Real outerTubeInitialRotation() const { return _ot_initial_rotation.value.value(); }
    Real outerTubeDistalStraightLength() const { return _ot_distal_straight_length.value.value(); }

    Sim::VirtuosoArm::ToolType toolType() const { return _tool_type.value.value(); }

    Vec3r baseInitialPosition() const { return _base_initial_position.value.value(); }
    Vec3r baseInitialRotation() const { return _base_initial_rotation.value.value(); }

    protected:
    ConfigParameter<Real> _it_outer_diameter;
    ConfigParameter<Real> _it_inner_diameter;
    ConfigParameter<Real> _it_initial_translation;
    ConfigParameter<Real> _it_initial_rotation;

    ConfigParameter<Real> _ot_outer_diameter;
    ConfigParameter<Real> _ot_inner_diameter;
    ConfigParameter<Real> _ot_r_curvature;
    ConfigParameter<Real> _ot_distal_straight_length;
    ConfigParameter<Real> _ot_initial_translation;
    ConfigParameter<Real> _ot_initial_rotation;

    ConfigParameter<Sim::VirtuosoArm::ToolType> _tool_type;

    ConfigParameter<Vec3r> _base_initial_position;
    ConfigParameter<Vec3r> _base_initial_rotation;

};

} // namespace Config

#endif // __VIRTUOSO_ARM_CONFIG_HPP