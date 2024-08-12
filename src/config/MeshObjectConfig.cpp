#include "config/MeshObjectConfig.hpp"

MeshObjectConfig::MeshObjectConfig(const YAML::Node& node)
    : Config(node)
{
    _extractParameter("filename", node, _filename);

    _extractParameter("max-size", node, _max_size);
    _extractParameter("position", node, _initial_position);
    _extractParameter("velocity", node, _initial_velocity);
    
    _extractParameter("draw-points", node, _draw_points);
    _extractParameter("color", node, _color);
}