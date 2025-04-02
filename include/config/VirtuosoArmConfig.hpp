#ifndef __VIRTUOSO_ARM_CONFIG_HPP
#define __VIRTUOSO_ARM_CONFIG_HPP

#include "config/ObjectConfig.hpp"

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
    static std::optional<Eigen::Vector3d>& DEFAULT_OT_INITIAL_POSITION() { static std::optional<Eigen::Vector3d> ot_pos({0.0, 0.0, 0.0}); return ot_pos; }

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

        _extractParameter("outer-tube-position", node, _ot_initial_position, DEFAULT_OT_INITIAL_POSITION());
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
    Eigen::Vector3d outerTubeInitialPosition() const { return _ot_initial_position.value.value(); }

    protected:
    ConfigParameter<double> _it_diameter;
    ConfigParameter<double> _it_initial_translation;
    ConfigParameter<double> _it_initial_rotation;

    ConfigParameter<double> _ot_diameter;
    ConfigParameter<double> _ot_r_curvature;
    ConfigParameter<double> _ot_distal_straight_length;
    ConfigParameter<double> _ot_initial_translation;
    ConfigParameter<double> _ot_initial_rotation;
    ConfigParameter<Eigen::Vector3d> _ot_initial_position;

};

#endif // __VIRTUOSO_ARM_CONFIG_HPP