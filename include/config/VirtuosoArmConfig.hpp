#ifndef __VIRTUOSO_ARM_CONFIG_HPP
#define __VIRTUOSO_ARM_CONFIG_HPP

#include "config/ObjectConfig.hpp"

class VirtuosoArmConfig : public ObjectConfig
{
    public:
    static std::optional<Real>& DEFAULT_OT_DIAMETER() { static std::optional<Real> ot_dia(0.15); return ot_dia; }
    static std::optional<Real>& DEFUALT_OT_R_CURVATURE() { static std::optional<Real> ot_curv(0.15); return ot_curv; }
    static std::optional<Real>& DEFAULT_IT_DIAMETER() { static std::optional<Real> it_dia(0.1); return it_dia; }
    static std::optional<Real>& DEFAULT_IT_INITIAL_TRANSLATION() { static std::optional<Real> it_trans(0.05); return it_trans; }
    static std::optional<Real>& DEFAULT_IT_INITIAL_ROTATION() { static std::optional<Real> it_rot(0); return it_rot; }
    static std::optional<Real>& DEFAULT_OT_INITIAL_TRANSLATION() { static std::optional<Real> ot_trans(0.1); return ot_trans; }
    static std::optional<Real>& DEFAULT_OT_INITIAL_ROTATION() { static std::optional<Real> ot_rot(0); return ot_rot; }
    static std::optional<Real>& DEFAULT_OT_DISTAL_STRAIGHT_LENGTH() { static std::optional<Real> ot_l(0); return ot_l; }
    static std::optional<Vec3r>& DEFAULT_BASE_INITIAL_POSITION() { static std::optional<Vec3r> pos({0.0, 0.0, 0.0}); return pos; }
    static std::optional<Vec3r>& DEFAULT_BASE_INITIAL_ROTATION() { static std::optional<Vec3r> rot({0.0, 0.0, 0.0}); return rot; }


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

        _extractParameter("base-position", node, _base_initial_position, DEFAULT_BASE_INITIAL_POSITION());
        _extractParameter("base-rotation", node, _base_initial_rotation, DEFAULT_BASE_INITIAL_ROTATION());
    }

    explicit VirtuosoArmConfig( const std::string& name, 
        const Vec3r& initial_pos, const Vec3r& initial_rot, const Vec3r& initial_velocity, bool collisions, bool graphics_only,
        Real ot_dia, Real ot_r_curve, Real ot_d_s_length, Real it_dia,
        Real ot_rot, Real ot_trans, Real it_rot, Real it_trans
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

        _base_initial_position.value = initial_pos;
        _base_initial_rotation.value = initial_rot;
    }

    // Getters and setters
    Real innerTubeDiameter() const { return _it_diameter.value.value(); }
    Real innerTubeInitialTranslation() const { return _it_initial_translation.value.value(); }
    Real innerTubeInitialRotation() const { return _it_initial_rotation.value.value(); }
    Real outerTubeDiameter() const { return _ot_diameter.value.value(); }
    Real outerTubeRadiusOfCurvature() const { return _ot_r_curvature.value.value(); }
    Real outerTubeInitialTranslation() const { return _ot_initial_translation.value.value(); }
    Real outerTubeInitialRotation() const { return _ot_initial_rotation.value.value(); }
    Real outerTubeDistalStraightLength() const { return _ot_distal_straight_length.value.value(); }
    Vec3r baseInitialPosition() const { return _base_initial_position.value.value(); }
    Vec3r baseInitialRotation() const { return _base_initial_rotation.value.value(); }

    protected:
    ConfigParameter<Real> _it_diameter;
    ConfigParameter<Real> _it_initial_translation;
    ConfigParameter<Real> _it_initial_rotation;

    ConfigParameter<Real> _ot_diameter;
    ConfigParameter<Real> _ot_r_curvature;
    ConfigParameter<Real> _ot_distal_straight_length;
    ConfigParameter<Real> _ot_initial_translation;
    ConfigParameter<Real> _ot_initial_rotation;

    ConfigParameter<Vec3r> _base_initial_position;
    ConfigParameter<Vec3r> _base_initial_rotation;

};

#endif // __VIRTUOSO_ARM_CONFIG_HPP