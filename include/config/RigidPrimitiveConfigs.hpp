#ifndef __RIGID_PRIMITIVE_CONFIGS_HPP
#define __RIGID_PRIMITIVE_CONFIGS_HPP

#include "config/RigidObjectConfig.hpp"

class RigidSphereConfig : public RigidObjectConfig
{

    public:
    static std::optional<double>& DEFAULT_RADIUS() { static std::optional<double> r(1); return r; }

    explicit RigidSphereConfig(const YAML::Node& node)
        : RigidObjectConfig(node)
    {
        _extractParameter("radius", node, _radius, DEFAULT_RADIUS());
    }

    double radius() const { return _radius.value.value(); }

    protected:
    ConfigParameter<double> _radius;

};

///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

class RigidBoxConfig : public RigidObjectConfig
{
    public:
    static std::optional<Eigen::Vector3d>& DEFAULT_SIZE() { static std::optional<Eigen::Vector3d> size({1,1,1}); return size; }

    explicit RigidBoxConfig(const YAML::Node& node)
        : RigidObjectConfig(node)
    {
        _extractParameter("size", node, _size, DEFAULT_SIZE());
    }

    Eigen::Vector3d size() const { return _size.value.value(); }

    protected:
    ConfigParameter<Eigen::Vector3d> _size;
};

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

class RigidCylinderConfig : public RigidObjectConfig
{
    public:
    static std::optional<double>& DEFAULT_RADIUS() { static std::optional<double> r(1); return r; }
    static std::optional<double>& DEFAULT_HEIGHT() { static std::optional<double> h(1); return h; }
    
    explicit RigidCylinderConfig(const YAML::Node& node)
        : RigidObjectConfig(node)
    {
        _extractParameter("radius", node, _radius, DEFAULT_RADIUS());
        _extractParameter("height", node, _height, DEFAULT_HEIGHT());
    }

    double radius() const { return _radius.value.value(); }
    double height() const { return _height.value.value(); }

    protected:
    ConfigParameter<double> _radius;
    ConfigParameter<double> _height;
};

#endif // __RIGID_PRIMITIVE_CONFIGS_HPP