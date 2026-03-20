#ifndef __OBJECT_RENDER_CONFIG_HPP
#define __OBJECT_RENDER_CONFIG_HPP

#include "config/Config.hpp"

#include <string>
#include <map>
#include <optional>

namespace Config
{

class ObjectRenderConfig : public Config
{
    public:
    enum class RenderType
    {
        PBR=0,
        PHONG,
        FLAT
    };

    static std::map<std::string, RenderType> RENDER_TYPE_MAP()
    {
        static std::map<std::string, RenderType> render_map{
            {"PBR", RenderType::PBR},
            {"Phong", RenderType::PHONG},
            {"Flat", RenderType::FLAT}
        };

        return render_map;
    }

    public:
    explicit ObjectRenderConfig()
        : Config()
    {

    }

    explicit ObjectRenderConfig(const YAML::Node& node)
        : Config(node)
    {
        _extractParameterWithOptions("render-type", node, _render_type, RENDER_TYPE_MAP());

        _extractParameter("orm-texture-filename", node, _orm_texture_filename);
        _extractParameter("normals-texture-filename", node, _normals_texture_filename);
        _extractParameter("base-color-texture-filename", node, _base_color_texture_filename);
        
        // General texture mapping support
        _extractParameter("texture-file", node, _texture_file);

        _extractParameter("metallic", node, _metallic);
        _extractParameter("roughness", node, _roughness);
        _extractParameter("opacity", node, _opacity);
        _extractParameter("color", node, _color);
        _extractParameter("colors", node, _colors);

        _extractParameter("smooth-normals", node, _smooth_normals);
        _extractParameter("draw-faces", node, _draw_faces);
        _extractParameter("draw-edges", node, _draw_edges);
        _extractParameter("draw-points", node, _draw_points);
        
        // MTL material properties
        _extractParameter("mtl-file", node, _mtl_file);
        _extractParameter("material-name", node, _material_name);
        _extractParameter("ambient-color", node, _ambient_color);
        _extractParameter("diffuse-color", node, _diffuse_color);
        _extractParameter("specular-color", node, _specular_color);
        _extractParameter("specular-exponent", node, _specular_exponent);
    }

    explicit ObjectRenderConfig(
        RenderType render_type,
        std::optional<std::string> orm_texture_filename, std::optional<std::string> normals_texture_filename, std::optional<std::string> base_color_texture_filename,
        Real metallic, Real roughness, Real opacity, const Vec3r& color,
        bool smooth_normals, bool draw_faces, bool draw_edges, bool draw_points
    )
    {
        _render_type.value = render_type;

        _orm_texture_filename.value = orm_texture_filename;
        _normals_texture_filename.value = normals_texture_filename;
        _base_color_texture_filename.value = base_color_texture_filename;

        _metallic.value = metallic;
        _roughness.value = roughness;
        _opacity.value = opacity;
        _color.value = color;

        _smooth_normals.value = smooth_normals;
        _draw_faces.value = draw_faces;
        _draw_edges.value = draw_edges;
        _draw_points.value = draw_points;
    }

    RenderType renderType() const { return _render_type.value; }
    std::optional<std::string> ormTextureFilename() const { return _orm_texture_filename.value; }
    std::optional<std::string> normalsTextureFilename() const { return _normals_texture_filename.value; }
    std::optional<std::string> baseColorTextureFilename() const { return _base_color_texture_filename.value; }
    
    // General texture mapping accessor
    std::optional<std::string> textureFile() const { return _texture_file.value; }

    Real metallic() const { return _metallic.value; }
    Real roughness() const { return _roughness.value; }
    Real opacity() const { return _opacity.value; }
    std::optional<Vec3r> color() const { return _color.value; }
    std::optional<std::vector<Vec3r>> colors() const { return _colors.value; }

    bool smoothNormals() const { return _smooth_normals.value; }
    bool drawFaces() const { return _draw_faces.value; }
    bool drawEdges() const { return _draw_edges.value; }
    bool drawPoints() const { return _draw_points.value; }
    
    // MTL material properties accessors
    std::optional<std::string> mtlFile() const { return _mtl_file.value; }
    std::optional<std::string> materialName() const { return _material_name.value; }
    std::optional<Vec3r> ambientColor() const { return _ambient_color.value; }
    std::optional<Vec3r> diffuseColor() const { return _diffuse_color.value; }
    std::optional<Vec3r> specularColor() const { return _specular_color.value; }
    std::optional<Real> specularExponent() const { return _specular_exponent.value; }
    
    // MTL material properties setters (for auto-loading from MTL files)
    void setMtlFile(const std::string& filename) { _mtl_file.value = filename; }
    void setMaterialName(const std::string& name) { _material_name.value = name; }
    void setAmbientColor(const Vec3r& color) { _ambient_color.value = color; }
    void setDiffuseColor(const Vec3r& color) { _diffuse_color.value = color; }
    void setSpecularColor(const Vec3r& color) { _specular_color.value = color; }
    void setSpecularExponent(Real exponent) { _specular_exponent.value = exponent; }
    void setOpacity(Real opacity) { _opacity.value = opacity; }
    
    // Texture file setter
    void setTextureFile(const std::string& filename) { _texture_file.value = filename; }

    // Material property setters
    void setMetallic(Real val) { _metallic.value = val; }
    void setRoughness(Real val) { _roughness.value = val; }

    protected:
    ConfigParameter<RenderType> _render_type = ConfigParameter<RenderType>(RenderType::PHONG); 

    // PBR texture filenames
    ConfigParameter<std::optional<std::string>> _orm_texture_filename;
    ConfigParameter<std::optional<std::string>> _normals_texture_filename;
    ConfigParameter<std::optional<std::string>> _base_color_texture_filename;
    
    // General texture file for texture mapping
    ConfigParameter<std::optional<std::string>> _texture_file = ConfigParameter<std::optional<std::string>>();

    ConfigParameter<Real> _metallic = ConfigParameter<Real>(0.0);
    ConfigParameter<Real> _roughness = ConfigParameter<Real>(0.5);
    
    // MTL material properties
    ConfigParameter<std::optional<std::string>> _mtl_file = ConfigParameter<std::optional<std::string>>();
    ConfigParameter<std::optional<std::string>> _material_name = ConfigParameter<std::optional<std::string>>();
    ConfigParameter<std::optional<Vec3r>> _ambient_color = ConfigParameter<std::optional<Vec3r>>();
    ConfigParameter<std::optional<Vec3r>> _diffuse_color = ConfigParameter<std::optional<Vec3r>>();
    ConfigParameter<std::optional<Vec3r>> _specular_color = ConfigParameter<std::optional<Vec3r>>();
    ConfigParameter<std::optional<Real>> _specular_exponent = ConfigParameter<std::optional<Real>>();
    ConfigParameter<Real> _opacity = ConfigParameter<Real>(1.0);
    ConfigParameter<std::optional<Vec3r>> _color = ConfigParameter<std::optional<Vec3r>>();
    ConfigParameter<std::optional<std::vector<Vec3r>>> _colors = ConfigParameter<std::optional<std::vector<Vec3r>>>();

    ConfigParameter<bool> _smooth_normals = ConfigParameter<bool>(true);
    ConfigParameter<bool> _draw_faces = ConfigParameter<bool>(true);
    ConfigParameter<bool> _draw_edges = ConfigParameter<bool>(false);
    ConfigParameter<bool> _draw_points = ConfigParameter<bool>(false);
};

} // namespace Config

#endif // __OBJECT_RENDER_CONFIG_HPP