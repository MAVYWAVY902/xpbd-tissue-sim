#ifndef __XPBD_MESH_OBJECT_CONFIG_HPP
#define __XPBD_MESH_OBJECT_CONFIG_HPP

#include "config/simobject/ObjectConfig.hpp"
#include "config/simobject/MeshObjectConfig.hpp"
#include "config/simobject/ElasticMaterialConfig.hpp"

#include "common/XPBDTypedefs.hpp"

#include <memory>

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
            {"Jacobi", XPBDObjectSolverTypeEnum::JACOBI},
            {"Parallel-Jacobi", XPBDObjectSolverTypeEnum::PARALLEL_JACOBI}
        };
        return solver_type_options;
    }

    static std::map<std::string, XPBDMeshObjectConstraintConfigurationEnum>& CONSTRAINT_TYPE_OPTIONS()
    { 
        static std::map<std::string, XPBDMeshObjectConstraintConfigurationEnum> constraint_type_options{
            {"Stable-Neohookean", XPBDMeshObjectConstraintConfigurationEnum::STABLE_NEOHOOKEAN},
            {"Stable-Neohookean-Combined", XPBDMeshObjectConstraintConfigurationEnum::STABLE_NEOHOOKEAN_COMBINED}
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
        // create the ElasticMaterialConfig from the material yaml node
        // _material_config = std::make_unique<ElasticMaterialConfig>(node["material"]);
        _extractParameter("material", node, _material);

        // extract parameters
        _extractParameter("self-collisions", node, _self_collisions);
        _extractParameter("num-solver-iters", node, _num_solver_iters);
        _extractParameter("num-local-collision-iters", node, _num_local_collision_iters);
        _extractParameterWithOptions("solver-type", node, _solve_type, SOLVER_TYPE_OPTIONS());
        _extractParameterWithOptions("constraint-type", node, _constraint_type, CONSTRAINT_TYPE_OPTIONS());
        _extractParameterWithOptions("residual-policy", node, _residual_policy, RESIDUAL_POLICY_OPTIONS());
        
    }

    explicit XPBDMeshObjectConfig(  const std::string& name, const Vec3r& initial_position, const Vec3r& initial_rotation,                  // Object params
                                    const Vec3r& initial_velocity, bool collisions, bool graphics_only,

                                    const std::string& filename, const std::optional<Real>& max_size, const std::optional<Vec3r>& size,     // MeshObject params
                                    bool draw_points, bool draw_edges, bool draw_faces, const Vec4r& color,

                                    const std::string& mat_name,

                                    bool self_collisions, int num_solver_iters, int num_local_collision_iters,
                                    XPBDObjectSolverTypeEnum solver_type, XPBDMeshObjectConstraintConfigurationEnum constraint_type,                   // XPBDMeshObject params
                                    XPBDSolverResidualPolicyEnum residual_policy,
                                
                                    const ObjectRenderConfig& render_config)
        : ObjectConfig(name, initial_position, initial_rotation, initial_velocity, collisions, graphics_only, render_config),
          MeshObjectConfig(filename, max_size, size, draw_points, draw_edges, draw_faces, color)
    {
        // _material_config = std::make_unique<ElasticMaterialConfig>(name + "_material", density, E, nu, mu_s, mu_k);
        _material.value = mat_name;

        _self_collisions.value = self_collisions;
        _num_solver_iters.value = num_solver_iters;
        _num_local_collision_iters.value = num_local_collision_iters;
        _solve_type.value = solver_type;
        _constraint_type.value = constraint_type;
        _residual_policy.value = residual_policy;
    }

    std::unique_ptr<ObjectType> createObject(const Sim::Simulation* sim) const;

    // Getters
    bool selfCollisions() const { return _self_collisions.value; }
    int numSolverIters() const { return _num_solver_iters.value; }
    int numLocalCollisionIters() const { return _num_local_collision_iters.value; }
    XPBDObjectSolverTypeEnum solverType() const { return _solve_type.value; }
    XPBDMeshObjectConstraintConfigurationEnum constraintType() const { return _constraint_type.value; }
    XPBDSolverResidualPolicyEnum residualPolicy() const { return _residual_policy.value; }

    // ElasticMaterialConfig* materialConfig() const { return _material_config.get(); }
    std::string material() const { return _material.value; }

    protected:
    // Parameters
    ConfigParameter<bool> _self_collisions = ConfigParameter<bool>(false);
    ConfigParameter<int> _num_solver_iters = ConfigParameter<int>(1);
    ConfigParameter<int> _num_local_collision_iters = ConfigParameter<int>(0);
    ConfigParameter<XPBDObjectSolverTypeEnum> _solve_type = ConfigParameter<XPBDObjectSolverTypeEnum>(XPBDObjectSolverTypeEnum::GAUSS_SEIDEL);
    ConfigParameter<XPBDMeshObjectConstraintConfigurationEnum> _constraint_type = ConfigParameter<XPBDMeshObjectConstraintConfigurationEnum>(XPBDMeshObjectConstraintConfigurationEnum::STABLE_NEOHOOKEAN_COMBINED);
    ConfigParameter<XPBDSolverResidualPolicyEnum> _residual_policy = ConfigParameter<XPBDSolverResidualPolicyEnum>(XPBDSolverResidualPolicyEnum::NEVER);

    ConfigParameter<std::string> _material = ConfigParameter<std::string>("");
    // std::unique_ptr<ElasticMaterialConfig> _material_config;
};

} // namespace Config

#endif // __XPBD_MESH_OBJECT_CONFIG_HPP