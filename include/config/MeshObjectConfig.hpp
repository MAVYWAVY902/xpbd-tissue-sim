#ifndef __MESH_OBJECT_CONFIG_HPP
#define __MESH_OBJECT_CONFIG_HPP

#include "config/Config.hpp"

class MeshObjectConfig : public Config
{
    public:
    /** Creates a MeshObjectConfig from a YAML node, which consists of the parameters needed for a MeshObject
     * @param node : the YAML node (i.e. dictionary of key-value pairs) that information is pulled from
     */
    explicit MeshObjectConfig(const YAML::Node& node)
        : Config(node)
    {
        // extract parameters from Config
        _extractParameter("filename", node, _filename);

        _extractParameter("max-size", node, _max_size);
        _extractParameter("position", node, _initial_position);
        _extractParameter("velocity", node, _initial_velocity);
        
        _extractParameter("draw-points", node, _draw_points);
        _extractParameter("color", node, _color);
    }

    // Getters
    std::optional<std::string> filename() const { return _filename.value; }
    std::optional<double> maxSize() const { return _max_size.value; }
    std::optional<Eigen::Vector3d> initialPosition() const { return _initial_position.value; }
    std::optional<Eigen::Vector3d> initialVelocity() const { return _initial_velocity.value; }
    std::optional<bool> drawPoints() const { return _draw_points.value; }
    std::optional<Eigen::Vector4d> color() const { return _color.value; }

    protected:
    // parameters
    ConfigParameter<std::string> _filename;

    ConfigParameter<double> _max_size;
    ConfigParameter<Eigen::Vector3d> _initial_position;
    ConfigParameter<Eigen::Vector3d> _initial_velocity;

    ConfigParameter<bool> _draw_points;
    ConfigParameter<Eigen::Vector4d> _color;
};

#endif // __MESH_OBJECT_CONFIG_HPP