#ifndef __OBJECT_CONFIG_HPP
#define __OBJECT_CONFIG_HPP

#include "config/Config.hpp"

namespace Sim
{
    class Simulation;
    class Object;
}


namespace Config
{

class ObjectConfig : public Config
{
    public:
    static std::optional<Vec3r>& DEFAULT_POSITION() { static std::optional<Vec3r> pos({0,0,0}); return pos; }
    static std::optional<Vec3r>& DEFAULT_VELOCITY() { static std::optional<Vec3r> vel({0,0,0}); return vel; }
    static std::optional<Vec3r>& DEFAULT_ROTATION() { static std::optional<Vec3r> rot({0,0,0}); return rot; }
    static std::optional<bool>& DEFAULT_COLLISIONS() { static std::optional<bool> collisions(false); return collisions; }
    static std::optional<bool>& DEFAULT_GRAPHICS_ONLY() { static std::optional<bool> graphics_only(false); return graphics_only; }


    explicit ObjectConfig(const YAML::Node& node)
        : Config(node)
    {
        _extractParameter("collisions", node, _collisions, DEFAULT_COLLISIONS());
        _extractParameter("graphics-only", node, _graphics_only, DEFAULT_GRAPHICS_ONLY());

        _extractParameter("position", node, _initial_position, DEFAULT_POSITION());
        _extractParameter("velocity", node, _initial_velocity, DEFAULT_VELOCITY());
        _extractParameter("rotation", node, _initial_rotation, DEFAULT_ROTATION());
    }

    explicit ObjectConfig(const std::string& name, const Vec3r& initial_position, const Vec3r& initial_rotation,
                          const Vec3r& initial_velocity, bool collisions, bool graphics_only)
        : Config(name)
    {
        _initial_position.value = initial_position;
        _initial_rotation.value = initial_rotation;
        _initial_velocity.value = initial_velocity;
        _collisions.value = collisions;
        _graphics_only.value = graphics_only;
    }
    
    bool collisions() const { return _collisions.value.value(); }
    bool graphicsOnly() const { return _graphics_only.value.value(); }
    Vec3r initialPosition() const { return _initial_position.value.value(); }
    Vec3r initialVelocity() const { return _initial_velocity.value.value(); }
    Vec3r initialRotation() const { return _initial_rotation.value.value(); }

    protected:

    ConfigParameter<bool> _collisions;
    ConfigParameter<bool> _graphics_only;

    ConfigParameter<Vec3r> _initial_position;
    ConfigParameter<Vec3r> _initial_velocity;
    ConfigParameter<Vec3r> _initial_rotation;

    const Sim::Simulation* _sim;

};

} // namespace Config

#endif // __OBJECT_CONFIG_HPP