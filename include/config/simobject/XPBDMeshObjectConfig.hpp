#ifndef __XPBD_MESH_OBJECT_CONFIG_HPP
#define __XPBD_MESH_OBJECT_CONFIG_HPP

#include "config/simobject/ObjectConfig.hpp"
#include "config/simobject/MeshObjectConfig.hpp"
#include "config/simobject/ElasticMaterialConfig.hpp"
#include "utils/MTLParser.hpp"

#include "common/XPBDTypedefs.hpp"

#include <memory>
#include <filesystem>

namespace Sim
{
    template<bool IsFirstOrder>
    class XPBDMeshObject_Base_;
}

namespace Config
{

class XPBDMeshObjectConfig : public ObjectConfig, public MeshObjectConfig
{
    public:
    using ObjectType = Sim::XPBDMeshObject_Base_<false>;

    static std::map<std::string, XPBDObjectSolverTypeEnum>& SOLVER_TYPE_OPTIONS() 
    {
        static std::map<std::string, XPBDObjectSolverTypeEnum> solver_type_options{
            {"Gauss-Seidel", XPBDObjectSolverTypeEnum::GAUSS_SEIDEL},
            {"Colored-Gauss-Seidel", XPBDObjectSolverTypeEnum::COLORED_GAUSS_SEIDEL},
            {"Jacobi", XPBDObjectSolverTypeEnum::JACOBI},
            {"Parallel-Jacobi", XPBDObjectSolverTypeEnum::PARALLEL_JACOBI}
        };
        return solver_type_options;
    }

    static std::map<std::string, XPBDMeshObjectConstraintConfigurationEnum>& CONSTRAINT_TYPE_OPTIONS()
    { 
        static std::map<std::string, XPBDMeshObjectConstraintConfigurationEnum> constraint_type_options{
            {"Stable-Neohookean", XPBDMeshObjectConstraintConfigurationEnum::STABLE_NEOHOOKEAN},
            {"Stable-Neohookean-Combined", XPBDMeshObjectConstraintConfigurationEnum::STABLE_NEOHOOKEAN_COMBINED},
            {"Nerve-Only", XPBDMeshObjectConstraintConfigurationEnum::NERVE_ONLY}
        };
        return constraint_type_options;
    }

    static std::map<std::string, XPBDSolverResidualPolicyEnum>& RESIDUAL_POLICY_OPTIONS()
    {
        static std::map<std::string, XPBDSolverResidualPolicyEnum> residual_policy_options{
            {"Never", XPBDSolverResidualPolicyEnum::NEVER},
            {"Every-Substep", XPBDSolverResidualPolicyEnum::EVERY_SUBSTEP},
            {"Every-Iteration", XPBDSolverResidualPolicyEnum::EVERY_ITERATION}
        };
        return residual_policy_options;
    }

    public:
    /** Creates a Config from a YAML node, which consists of the specialized parameters needed for XPBDMeshObject.
     * @param node : the YAML node (i.e. dictionary of key-value pairs) that information is pulled from
     */
    explicit XPBDMeshObjectConfig(const YAML::Node& node)
        : ObjectConfig(node), MeshObjectConfig(node)
    {
        _extractParameter("materials", node, _materials);
        _extractParameter("element-classes-filename", node, _element_classes_filename);

        // extract parameters
        _extractParameter("self-collisions", node, _self_collisions);
        _extractParameter("inter-object-collisions", node, _inter_object_collisions);
        _extractParameter("num-solver-iters", node, _num_solver_iters);
        _extractParameter("num-local-collision-iters", node, _num_local_collision_iters);
        _extractParameterWithOptions("solver-type", node, _solve_type, SOLVER_TYPE_OPTIONS());
        _extractParameterWithOptions("constraint-type", node, _constraint_type, CONSTRAINT_TYPE_OPTIONS());
        _extractParameterWithOptions("residual-policy", node, _residual_policy, RESIDUAL_POLICY_OPTIONS());
    // optional: list of vertex indices to fix (0-based)
    _extractParameter("fixed-vertices", node, _fixed_vertices);
        
        // Auto-load MTL file if available
        _loadMTLIfAvailable();
    }

    explicit XPBDMeshObjectConfig(  const std::string& name, const Vec3r& initial_position, const Vec3r& initial_rotation,                  // Object params
                                    const Vec3r& initial_velocity, bool collisions, bool graphics_only,

                                    const std::string& filename, const std::optional<Real>& max_size, const std::optional<Vec3r>& size,     // MeshObject params
                                    bool draw_points, bool draw_edges, bool draw_faces, const Vec4r& color,

                                    const std::vector<std::string>& mat_names, const std::optional<std::string>& element_classes_filename,

                                    bool self_collisions, bool inter_object_collisions, int num_solver_iters, int num_local_collision_iters,
                                    XPBDObjectSolverTypeEnum solver_type, XPBDMeshObjectConstraintConfigurationEnum constraint_type,                   // XPBDMeshObject params
                                    XPBDSolverResidualPolicyEnum residual_policy,
                                
                                    const ObjectRenderConfig& render_config)
        : ObjectConfig(name, initial_position, initial_rotation, initial_velocity, collisions, graphics_only, render_config),
          MeshObjectConfig(filename, max_size, size, draw_points, draw_edges, draw_faces, color)
    {
        _materials.value = mat_names;
        _element_classes_filename.value = element_classes_filename;

        _self_collisions.value = self_collisions;
        _inter_object_collisions.value = inter_object_collisions;
        _num_solver_iters.value = num_solver_iters;
        _num_local_collision_iters.value = num_local_collision_iters;
        _solve_type.value = solver_type;
        _constraint_type.value = constraint_type;
        _residual_policy.value = residual_policy;
    }

    std::unique_ptr<ObjectType> createObject(const Sim::PhysicsContext* sim) const;

    // Getters
    bool selfCollisions() const { return _self_collisions.value; }
    bool interObjectCollisions() const { return _inter_object_collisions.value; }
    int numSolverIters() const { return _num_solver_iters.value; }
    int numLocalCollisionIters() const { return _num_local_collision_iters.value; }
    XPBDObjectSolverTypeEnum solverType() const { return _solve_type.value; }
    XPBDMeshObjectConstraintConfigurationEnum constraintType() const { return _constraint_type.value; }
    XPBDSolverResidualPolicyEnum residualPolicy() const { return _residual_policy.value; }

    std::vector<std::string> materials() const { return _materials.value; }
    std::optional<std::string> elementClassesFilename() const { return _element_classes_filename.value; }
    std::vector<int> fixedVertices() const { return _fixed_vertices.value; }

    protected:
    // Parameters
    ConfigParameter<bool> _self_collisions = ConfigParameter<bool>(false);
    ConfigParameter<bool> _inter_object_collisions = ConfigParameter<bool>(false);
    ConfigParameter<int> _num_solver_iters = ConfigParameter<int>(1);
    ConfigParameter<int> _num_local_collision_iters = ConfigParameter<int>(0);
    ConfigParameter<XPBDObjectSolverTypeEnum> _solve_type = ConfigParameter<XPBDObjectSolverTypeEnum>(XPBDObjectSolverTypeEnum::GAUSS_SEIDEL);
    ConfigParameter<XPBDMeshObjectConstraintConfigurationEnum> _constraint_type = ConfigParameter<XPBDMeshObjectConstraintConfigurationEnum>(XPBDMeshObjectConstraintConfigurationEnum::STABLE_NEOHOOKEAN_COMBINED);
    ConfigParameter<XPBDSolverResidualPolicyEnum> _residual_policy = ConfigParameter<XPBDSolverResidualPolicyEnum>(XPBDSolverResidualPolicyEnum::NEVER);

    ConfigParameter<std::vector<std::string>> _materials = ConfigParameter<std::vector<std::string>>({});
    ConfigParameter<std::optional<std::string>> _element_classes_filename;
    ConfigParameter<std::vector<int>> _fixed_vertices = ConfigParameter<std::vector<int>>({});
    
    private:
    /** Auto-load MTL file if present and apply to render config */
    void _loadMTLIfAvailable()
    {
        std::string obj_filename = filename();
        if (obj_filename.empty()) {
            return;
        }
        
        std::filesystem::path obj_path(obj_filename);
        std::cout << KCYN << "[XPBDMeshObject] Checking for MTL file for: " 
                  << obj_path.filename().string() << RST << std::endl;
        
        // Only try to load MTL if user hasn't manually specified material properties
        if (_render_config.mtlFile().has_value() || 
            _render_config.diffuseColor().has_value() ||
            _render_config.specularColor().has_value() ||
            _render_config.color().has_value())
        {
            std::cout << KYEL << "[XPBDMeshObject] Skipping MTL auto-load: User specified material/color in config" 
                      << RST << std::endl;
            return; // User has manually configured materials
        }
        
        // Check if corresponding MTL file exists
        std::filesystem::path mtl_path = obj_path.parent_path() / (obj_path.stem().string() + ".mtl");
        
        if (!std::filesystem::exists(mtl_path)) {
            std::cout << KYEL << "[XPBDMeshObject] No MTL file found at: " 
                      << mtl_path.string() << RST << std::endl;
            return; // No MTL file found
        }
        
        std::cout << KGRN << "[XPBDMeshObject] Found MTL file: " 
                  << mtl_path.filename().string() << RST << std::endl;
        
        // Parse MTL file
        Utils::MTLParser parser;
        auto materials = parser.parse(mtl_path.string());
        
        std::cout << KCYN << "[XPBDMeshObject] Parsed " << materials.size() << " materials from MTL" << RST << std::endl;
        
        if (materials.empty()) {
            std::cout << KRED << "[XPBDMeshObject] ERROR: No materials found in MTL file!" << RST << std::endl;
            return; // No materials in MTL file
        }
        
        // Use the first material (most MTL files have only one material)
        const Utils::Material& mat = materials.begin()->second;
        
        std::cout << KGRN << "========================================" << RST << std::endl;
        std::cout << KGRN << "[XPBDMeshObject] Successfully loaded MTL material:" << RST << std::endl;
        std::cout << KGRN << "  Material name: " << mat.name << RST << std::endl;
        std::cout << KGRN << "  Diffuse (Kd):  [" << mat.diffuse_color.transpose() << "]" << RST << std::endl;
        std::cout << KGRN << "  Ambient (Ka):  [" << mat.ambient_color.transpose() << "]" << RST << std::endl;
        std::cout << KGRN << "  Specular (Ks): [" << mat.specular_color.transpose() << "]" << RST << std::endl;
        std::cout << KGRN << "  Shininess (Ns): " << mat.specular_exponent << RST << std::endl;
        std::cout << KGRN << "  Opacity: " << mat.opacity << RST << std::endl;
        std::cout << KGRN << "========================================" << RST << std::endl;
        
        // Apply material properties to render config
        std::cout << KCYN << "[XPBDMeshObject] Applying material to render config..." << RST << std::endl;
        _render_config.setDiffuseColor(mat.diffuse_color);
        _render_config.setAmbientColor(mat.ambient_color);
        _render_config.setSpecularColor(mat.specular_color);
        _render_config.setSpecularExponent(mat.specular_exponent);
        _render_config.setOpacity(mat.opacity);
        _render_config.setMtlFile(mtl_path.string());
        _render_config.setMaterialName(mat.name);
        std::cout << KGRN << "[XPBDMeshObject] Material applied successfully!" << RST << std::endl;
    }
};

} // namespace Config

#endif // __XPBD_MESH_OBJECT_CONFIG_HPP