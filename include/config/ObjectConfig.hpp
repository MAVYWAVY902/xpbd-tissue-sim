#ifndef __OBJECT_CONFIG_HPP
#define __OBJECT_CONFIG_HPP

#include "config/Config.hpp"

class Simulation;

class ObjectConfig : public Config
{
    public:

    static std::optional<Eigen::Vector3d>& DEFAULT_POSITION() { static std::optional<Eigen::Vector3d> pos({0,0,0}); return pos; }
    static std::optional<Eigen::Vector3d>& DEFAULT_VELOCITY() { static std::optional<Eigen::Vector3d> vel({0,0,0}); return vel; }
    static std::optional<Eigen::Vector3d>& DEFAULT_ROTATION() { static std::optional<Eigen::Vector3d> rot({0,0,0}); return rot; }
    static std::optional<bool>& DEFAULT_COLLISIONS() { static std::optional<bool> collisions(false); return collisions; }


    explicit ObjectConfig(const YAML::Node& node)
        : Config(node)
    {
        _extractParameter("max-size", node, _max_size);
        _extractParameter("size", node, _size);

        _extractParameter("collisions", node, _collisions, DEFAULT_COLLISIONS());

        _extractParameter("position", node, _initial_position, DEFAULT_POSITION());
        _extractParameter("velocity", node, _initial_velocity, DEFAULT_VELOCITY());
        _extractParameter("rotation", node, _initial_rotation, DEFAULT_ROTATION());
    }

    std::optional<double> maxSize() const { return _max_size.value; }
    std::optional<Eigen::Vector3d> size() const { return _size.value; }

    bool collisions() const { return _collisions.value.value(); }
    Eigen::Vector3d initialPosition() const { return _initial_position.value.value(); }
    Eigen::Vector3d initialVelocity() const { return _initial_velocity.value.value(); }
    Eigen::Vector3d initialRotation() const { return _initial_rotation.value.value(); }

    protected:

    ConfigParameter<double> _max_size;
    ConfigParameter<Eigen::Vector3d> _size;

    ConfigParameter<bool> _collisions;

    ConfigParameter<Eigen::Vector3d> _initial_position;
    ConfigParameter<Eigen::Vector3d> _initial_velocity;
    ConfigParameter<Eigen::Vector3d> _initial_rotation;

    const Simulation* _sim;

};

#endif // __OBJECT_CONFIG_HPP