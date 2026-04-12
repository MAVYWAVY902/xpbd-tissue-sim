#ifndef __RIGID_MESH_OBJECT_CONFIG_HPP
#define __RIGID_MESH_OBJECT_CONFIG_HPP

#include "config/simobject/RigidObjectConfig.hpp"
#include "config/simobject/MeshObjectConfig.hpp"
#include "utils/MTLParser.hpp"

#include <filesystem>

namespace Sim
{
    class RigidMeshObject;
}

namespace Config
{
    
class RigidMeshObjectConfig : public RigidObjectConfig, public MeshObjectConfig
{
    public:
    using ObjectType = Sim::RigidMeshObject;

    public:

    explicit RigidMeshObjectConfig(const YAML::Node& node)
        : RigidObjectConfig(node), MeshObjectConfig(node)
    {
        _extractParameter("sdf-filename", node, _sdf_filename);
        
        // Auto-load MTL file if it exists next to the OBJ file
        _loadMTLIfAvailable();
    }

    explicit RigidMeshObjectConfig( const std::string& name, const Vec3r& initial_position, const Vec3r& initial_rotation,
                                    const Vec3r& initial_velocity, const Vec3r& initial_angular_velocity, Real density,
                                    bool collisions, bool graphics_only, bool fixed,
                                    const std::string& filename, const std::optional<Real>& max_size, const std::optional<Vec3r>& size,
                                    bool draw_points, bool draw_edges, bool draw_faces, const Vec4r& color,
                                    const std::optional<std::string>& sdf_filename,
                                    const ObjectRenderConfig& render_config )
        : RigidObjectConfig(name, initial_position, initial_rotation, initial_velocity, initial_angular_velocity, density, collisions, graphics_only, fixed, render_config),
          MeshObjectConfig(filename, max_size, size, draw_points, draw_edges, draw_faces, color)
    {
        _sdf_filename.value = sdf_filename;
    }

    std::unique_ptr<ObjectType> createObject(const Sim::PhysicsContext* sim) const;

    std::optional<std::string> sdfFilename() const { return _sdf_filename.value; }
    
    private:
    /** Auto-load MTL file if present and apply to render config */
    void _loadMTLIfAvailable()
    {
        std::string obj_filename = filename();
        if (obj_filename.empty()) {
            return;
        }
        
        std::filesystem::path obj_path(obj_filename);
        std::cout << KCYN << "[RigidMeshObject] Checking for MTL file for: " 
                  << obj_path.filename().string() << RST << std::endl;
        
        // Only try to load MTL if user hasn't manually specified material properties
        if (_render_config.mtlFile().has_value() || 
            _render_config.diffuseColor().has_value() ||
            _render_config.specularColor().has_value() ||
            _render_config.color().has_value())
        {
            std::cout << KYEL << "[RigidMeshObject] Skipping MTL auto-load: User specified material/color in config" 
                      << RST << std::endl;
            return; // User has manually configured materials
        }
        
        // Check if corresponding MTL file exists
        std::filesystem::path mtl_path = obj_path.parent_path() / (obj_path.stem().string() + ".mtl");
        
        if (!std::filesystem::exists(mtl_path)) {
            std::cout << KYEL << "[RigidMeshObject] No MTL file found at: " 
                      << mtl_path.string() << RST << std::endl;
            return; // No MTL file found
        }
        
        std::cout << KGRN << "[RigidMeshObject] Found MTL file: " 
                  << mtl_path.filename().string() << RST << std::endl;
        
        // Parse MTL file
        Utils::MTLParser parser;
        auto materials = parser.parse(mtl_path.string());
        
        std::cout << KCYN << "[RigidMeshObject] Parsed " << materials.size() << " materials from MTL" << RST << std::endl;
        
        if (materials.empty()) {
            std::cout << KRED << "[RigidMeshObject] ERROR: No materials found in MTL file!" << RST << std::endl;
            return; // No materials in MTL file
        }
        
        // Use the first material (most MTL files have only one material)
        const Utils::Material& mat = materials.begin()->second;
        
        std::cout << KGRN << "========================================" << RST << std::endl;
        std::cout << KGRN << "[RigidMeshObject] Successfully loaded MTL material:" << RST << std::endl;
        std::cout << KGRN << "  Material name: " << mat.name << RST << std::endl;
        std::cout << KGRN << "  Diffuse (Kd):  [" << mat.diffuse_color.transpose() << "]" << RST << std::endl;
        std::cout << KGRN << "  Ambient (Ka):  [" << mat.ambient_color.transpose() << "]" << RST << std::endl;
        std::cout << KGRN << "  Specular (Ks): [" << mat.specular_color.transpose() << "]" << RST << std::endl;
        std::cout << KGRN << "  Shininess (Ns): " << mat.specular_exponent << RST << std::endl;
        std::cout << KGRN << "  Opacity: " << mat.opacity << RST << std::endl;
        std::cout << KGRN << "========================================" << RST << std::endl;
        
        // Apply material properties to render config
        std::cout << KCYN << "[RigidMeshObject] Applying material to render config..." << RST << std::endl;
        _render_config.setDiffuseColor(mat.diffuse_color);
        _render_config.setAmbientColor(mat.ambient_color);
        _render_config.setSpecularColor(mat.specular_color);
        _render_config.setSpecularExponent(mat.specular_exponent);
        _render_config.setOpacity(mat.opacity);
        _render_config.setMtlFile(mtl_path.string());
        _render_config.setMaterialName(mat.name);
        std::cout << KGRN << "[RigidMeshObject] Material applied successfully!" << RST << std::endl;
    }

    protected:
    ConfigParameter<std::optional<std::string>> _sdf_filename;
};

} // namespace Config

#endif // __RIGID_MESH_OBJECT_CONFIG