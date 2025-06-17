#ifndef __MESH_OBJECT_CONFIG_HPP
#define __MESH_OBJECT_CONFIG_HPP

#include "config/Config.hpp"

namespace Config
{

class MeshObjectConfig
{
    public:
    static std::optional<std::string>& DEFAULT_FILENAME() { static std::optional<std::string> filename(""); return filename; }
    static std::optional<bool>& DEFAULT_DRAW_POINTS() { static std::optional<bool> draw_points(false); return draw_points; }
    static std::optional<bool>& DEFAULT_DRAW_EDGES() { static std::optional<bool> draw_edges(false); return draw_edges; }
    static std::optional<bool>& DEFAULT_DRAW_FACES() { static std::optional<bool> draw_faces(true); return draw_faces; }
    static std::optional<Vec4r>& DEFAULT_COLOR() { static std::optional<Vec4r> color({1.0, 1.0, 1.0, 1.0}); return color; }

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

    explicit MeshObjectConfig(const std::string& filename, const std::optional<Real>& max_size, const std::optional<Vec3r>& size,
                             bool draw_points, bool draw_edges, bool draw_faces, const Vec4r& color)
    {
        _filename.value = filename;
        _max_size.value = max_size;
        _size.value = size;
        _draw_points.value = draw_points;
        _draw_edges.value = draw_edges;
        _draw_faces.value = draw_faces;
        _color.value = color;
    }

    std::string filename() const { return _filename.value.value(); }
    bool drawPoints() const { return _draw_points.value.value(); }
    bool drawEdges() const { return _draw_edges.value.value(); }
    bool drawFaces() const { return _draw_faces.value.value(); }
    Vec4r color() const { return _color.value.value(); }

    std::optional<Real> maxSize() const { return _max_size.value; }
    std::optional<Vec3r> size() const { return _size.value; }

    protected:
    ConfigParameter<std::string> _filename;

    ConfigParameter<bool> _draw_points;
    ConfigParameter<bool> _draw_edges;
    ConfigParameter<bool> _draw_faces;
    ConfigParameter<Vec4r> _color;

    ConfigParameter<Real> _max_size;
    ConfigParameter<Vec3r> _size;

};

} // namespace Config

#endif // __MESH_OBJECT_CONFIG_HPP