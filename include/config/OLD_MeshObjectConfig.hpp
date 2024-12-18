#ifndef __MESH_OBJECT_CONFIG_HPP
#define __MESH_OBJECT_CONFIG_HPP

#include "config/Config.hpp"

class MeshObjectConfig : public Config
{
    /** Static predefined default for whether or not to draw mesh points */
    static std::optional<bool>& DEFAULT_DRAW_POINTS() { static std::optional<bool> draw_points(false); return draw_points; }
    static std::optional<bool>& DEFAULT_DRAW_EDGES() { static std::optional<bool> draw_edges(false); return draw_edges; }
    static std::optional<bool>& DEFAULT_DRAW_FACES() { static std::optional<bool> draw_faces(true); return draw_faces; }
    static std::optional<Eigen::Vector4d>& DEFAULT_COLOR() { static std::optional<Eigen::Vector4d> color({1.0, 1.0, 1.0, 1.0}); return color; }
    static std::optional<bool>& DEFAULT_COLLISIONS() { static std::optional<bool> collisions(false); return collisions; }

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
        _extractParameter("size", node, _size);
        _extractParameter("position", node, _initial_position);
        _extractParameter("velocity", node, _initial_velocity);
        _extractParameter("rotation", node, _initial_rotation);

        _extractParameter("collisions", node, _collisions, DEFAULT_COLLISIONS());
        
        _extractParameter("draw-points", node, _draw_points, DEFAULT_DRAW_POINTS());
        _extractParameter("draw-edges", node, _draw_edges, DEFAULT_DRAW_EDGES());
        _extractParameter("draw-faces", node, _draw_faces, DEFAULT_DRAW_FACES());
        _extractParameter("color", node, _color, DEFAULT_COLOR());
    }

    // Getters
    std::optional<std::string> filename() const { return _filename.value; }
    std::optional<double> maxSize() const { return _max_size.value; }
    std::optional<Eigen::Vector3d> size() const { return _size.value; }
    std::optional<Eigen::Vector3d> initialPosition() const { return _initial_position.value; }
    std::optional<Eigen::Vector3d> initialVelocity() const { return _initial_velocity.value; }
    std::optional<Eigen::Vector3d> initialRotation() const { return _initial_rotation.value;}
    std::optional<bool> collisions() const { return _collisions.value; }
    std::optional<bool> drawPoints() const { return _draw_points.value; }
    std::optional<bool> drawEdges() const { return _draw_edges.value; }
    std::optional<bool> drawFaces() const { return _draw_faces.value; }
    std::optional<Eigen::Vector4d> color() const { return _color.value; }
    std::optional<double> timeStep() const { return _time_step.value; }

    // Setters
    void timeStep(const ConfigParameter<double>& time_step) { _time_step = time_step; }

    protected:
    // parameters
    ConfigParameter<std::string> _filename;

    ConfigParameter<double> _max_size;
    ConfigParameter<Eigen::Vector3d> _size;
    ConfigParameter<Eigen::Vector3d> _initial_position;
    ConfigParameter<Eigen::Vector3d> _initial_velocity;
    ConfigParameter<Eigen::Vector3d> _initial_rotation;

    ConfigParameter<bool> _collisions;

    ConfigParameter<bool> _draw_points;
    ConfigParameter<bool> _draw_edges;
    ConfigParameter<bool> _draw_faces;
    ConfigParameter<Eigen::Vector4d> _color;

    ConfigParameter<double> _time_step;
};

#endif // __MESH_OBJECT_CONFIG_HPP