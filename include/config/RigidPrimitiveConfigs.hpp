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

    Real radius() const { return _radius.value.value(); }
    Real height() const { return _height.value.value(); }

    protected:
    ConfigParameter<Real> _radius;
    ConfigParameter<Real> _height;
};

#endif // __RIGID_PRIMITIVE_CONFIGS_HPP