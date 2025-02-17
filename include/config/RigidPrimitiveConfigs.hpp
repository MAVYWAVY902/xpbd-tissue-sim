#ifndef __RIGID_PRIMITIVE_CONFIGS_HPP
#define __RIGID_PRIMITIVE_CONFIGS_HPP

#include "config/RigidObjectConfig.hpp"

class RigidSphereConfig : public RigidObjectConfig
{

    public:
    static std::optional<Real>& DEFAULT_RADIUS() { static std::optional<Real> r(1); return r; }

    explicit RigidSphereConfig(const YAML::Node& node)
        : RigidObjectConfig(node)
    {
        _extractParameter("radius", node, _radius, DEFAULT_RADIUS());
    }

    explicit RigidSphereConfig(const std::string& name, const Vec3r& initial_position, const Vec3r& initial_rotation,
        const Vec3r& initial_velocity, const Vec3r& initial_angular_velocity, Real density, Real radius,
        bool collisions, bool fixed)
        : RigidObjectConfig(name, initial_position, initial_rotation, initial_velocity, initial_angular_velocity, density, collisions, fixed)
    {
        _radius.value = radius;
    }

    Real radius() const { return _radius.value.value(); }

    protected:
    ConfigParameter<Real> _radius;

};

///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

class RigidBoxConfig : public RigidObjectConfig
{
    public:
    static std::optional<Vec3r>& DEFAULT_SIZE() { static std::optional<Vec3r> size({1,1,1}); return size; }

    explicit RigidBoxConfig(const YAML::Node& node)
        : RigidObjectConfig(node)
    {
        _extractParameter("size", node, _size, DEFAULT_SIZE());
    }

    explicit RigidBoxConfig(const std::string& name, const Vec3r& initial_position, const Vec3r& initial_rotation,
        const Vec3r& initial_velocity, const Vec3r& initial_angular_velocity, Real density, const Vec3r& size,
        bool collisions, bool fixed)
        : RigidObjectConfig(name, initial_position, initial_rotation, initial_velocity, initial_angular_velocity, density, collisions, fixed)
    {
        _size.value = size;
    }

    Vec3r size() const { return _size.value.value(); }

    protected:
    ConfigParameter<Vec3r> _size;
};

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

class RigidCylinderConfig : public RigidObjectConfig
{
    public:
    static std::optional<Real>& DEFAULT_RADIUS() { static std::optional<Real> r(1); return r; }
    static std::optional<Real>& DEFAULT_HEIGHT() { static std::optional<Real> h(1); return h; }
    
    explicit RigidCylinderConfig(const YAML::Node& node)
        : RigidObjectConfig(node)
    {
        _extractParameter("radius", node, _radius, DEFAULT_RADIUS());
        _extractParameter("height", node, _height, DEFAULT_HEIGHT());
    }

    explicit RigidCylinderConfig(const std::string& name, const Vec3r& initial_position, const Vec3r& initial_rotation,
        const Vec3r& initial_velocity, const Vec3r& initial_angular_velocity, Real density, Real radius, Real height,
        bool collisions, bool fixed)
        : RigidObjectConfig(name, initial_position, initial_rotation, initial_velocity, initial_angular_velocity, density, collisions, fixed)
    {
        _radius.value = radius;
        _height.value = height;
    }

    Real radius() const { return _radius.value.value(); }
    Real height() const { return _height.value.value(); }

    protected:
    ConfigParameter<Real> _radius;
    ConfigParameter<Real> _height;
};

#endif // __RIGID_PRIMITIVE_CONFIGS_HPP