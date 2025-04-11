#ifndef __RIGID_OBJECT_CONFIG_HPP
#define __RIGID_OBJECT_CONFIG_HPP

#include "config/ObjectConfig.hpp"

class RigidObjectConfig : public ObjectConfig
{
    public:
    static std::optional<Vec3r>& DEFAULT_ANGULAR_VELOCITY() { static std::optional<Vec3r> ang_vel({0,0,0}); return ang_vel; }
    static std::optional<Real>& DEFAULT_DENSITY() { static std::optional<Real> density(1000); return density; }
    static std::optional<bool>& DEFAULT_FIXED() { static std::optional<bool> fixed(false); return fixed; }

    explicit RigidObjectConfig(const YAML::Node& node)
        : ObjectConfig(node)
    {
        // extract parameters from Config
        _extractParameter("angular-velocity", node, _initial_ang_velocity, DEFAULT_ANGULAR_VELOCITY());
        _extractParameter("density", node, _density, DEFAULT_DENSITY());
        _extractParameter("fixed", node, _fixed, DEFAULT_FIXED());
    }

    explicit RigidObjectConfig(const std::string& name, const Vec3r& initial_position, const Vec3r& initial_rotation,
                               const Vec3r& initial_velocity, const Vec3r& initial_angular_velocity, Real density,
                               bool collisions, bool graphics_only, bool fixed)
        : ObjectConfig(name, initial_position, initial_rotation, initial_velocity, collisions, graphics_only)
    {
        _initial_ang_velocity.value = initial_angular_velocity;
        _density.value = density;
        _fixed.value = fixed;
    }

    Vec3r initialAngularVelocity() const { return _initial_ang_velocity.value.value(); }
    Real density() const { return _density.value.value(); }
    bool fixed() const { return _fixed.value.value(); }

    protected:
    ConfigParameter<Vec3r> _initial_ang_velocity;
    ConfigParameter<Real> _density;
    ConfigParameter<bool> _fixed;

};

#endif // __RIGID_OBJECT_CONFIG_HPP