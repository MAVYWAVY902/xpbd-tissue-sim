#ifndef __MESH_OBJECT_CONFIG_HPP
#define __MESH_OBJECT_CONFIG_HPP

#include "config/Config.hpp"

class MeshObjectConfig
{
    public:
    static std::optional<std::string>& DEFAULT_FILENAME() { static std::optional<std::string> filename(""); return filename; }
    static std::optional<bool>& DEFAULT_DRAW_POINTS() { static std::optional<bool> draw_points(false); return draw_points; }
    static std::optional<bool>& DEFAULT_DRAW_EDGES() { static std::optional<bool> draw_edges(false); return draw_edges; }
    static std::optional<bool>& DEFAULT_DRAW_FACES() { static std::optional<bool> draw_faces(true); return draw_faces; }
    static std::optional<Eigen::Vector4d>& DEFAULT_COLOR() { static std::optional<Eigen::Vector4d> color({1.0, 1.0, 1.0, 1.0}); return color; }

    MeshObjectConfig(const YAML::Node& node)
    {
        Config::_extractParameter("filename", node, _filename, DEFAULT_FILENAME());
        Config::_extractParameter("draw-points", node, _draw_points, DEFAULT_DRAW_POINTS());
        Config::_extractParameter("draw-edges", node, _draw_edges, DEFAULT_DRAW_EDGES());
        Config::_extractParameter("draw-faces", node, _draw_faces, DEFAULT_DRAW_FACES());
        Config::_extractParameter("color", node, _color, DEFAULT_COLOR());

        Config::_extractParameter("max-size", node, _max_size);
        Config::_extractParameter("size", node, _size);
    }

    std::string filename() const { return _filename.value.value(); }
    bool drawPoints() const { return _draw_points.value.value(); }
    bool drawEdges() const { return _draw_edges.value.value(); }
    bool drawFaces() const { return _draw_faces.value.value(); }
    Eigen::Vector4d color() const { return _color.value.value(); }

    std::optional<double> maxSize() const { return _max_size.value; }
    std::optional<Eigen::Vector3d> size() const { return _size.value; }

    protected:
    ConfigParameter<std::string> _filename;

    ConfigParameter<bool> _draw_points;
    ConfigParameter<bool> _draw_edges;
    ConfigParameter<bool> _draw_faces;
    ConfigParameter<Eigen::Vector4d> _color;

    ConfigParameter<double> _max_size;
    ConfigParameter<Eigen::Vector3d> _size;

};

#endif // __MESH_OBJECT_CONFIG_HPP