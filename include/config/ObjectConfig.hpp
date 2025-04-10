#ifndef __OBJECT_CONFIG_HPP
#define __OBJECT_CONFIG_HPP

#include "config/Config.hpp"

class Simulation;

class ObjectConfig : public Config
{
    public:

    static std::optional<Vec3r>& DEFAULT_POSITION() { static std::optional<Vec3r> pos({0,0,0}); return pos; }
    static std::optional<Vec3r>& DEFAULT_VELOCITY() { static std::optional<Vec3r> vel({0,0,0}); return vel; }
    static std::optional<Vec3r>& DEFAULT_ROTATION() { static std::optional<Vec3r> rot({0,0,0}); return rot; }
    static std::optional<bool>& DEFAULT_COLLISIONS() { static std::optional<bool> collisions(false); return collisions; }


    explicit ObjectConfig(const YAML::Node& node)
        : Config(node)
    {
        _extractParameter("collisions", node, _collisions, DEFAULT_COLLISIONS());

        _extractParameter("position", node, _initial_position, DEFAULT_POSITION());
        _extractParameter("velocity", node, _initial_velocity, DEFAULT_VELOCITY());
        _extractParameter("rotation", node, _initial_rotation, DEFAULT_ROTATION());
    }

    explicit ObjectConfig(const std::string& name, const Vec3r& initial_position, const Vec3r& initial_rotation,
                          const Vec3r& initial_velocity, bool collisions)
        : Config(name)
    {
        _initial_position.value = initial_position;
        _initial_rotation.value = initial_rotation;
        _initial_velocity.value = initial_velocity;
        _collisions.value = collisions;
    }
    
    bool collisions() const { return _collisions.value.value(); }
    Vec3r initialPosition() const { return _initial_position.value.value(); }
    Vec3r initialVelocity() const { return _initial_velocity.value.value(); }
    Vec3r initialRotation() const { return _initial_rotation.value.value(); }

    protected:

    ConfigParameter<bool> _collisions;

    ConfigParameter<Vec3r> _initial_position;
    ConfigParameter<Vec3r> _initial_velocity;
    ConfigParameter<Vec3r> _initial_rotation;

    const Simulation* _sim;

};

#endif // __OBJECT_CONFIG_HPP