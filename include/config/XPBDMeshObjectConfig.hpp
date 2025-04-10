#ifndef __XPBD_MESH_OBJECT_CONFIG_HPP
#define __XPBD_MESH_OBJECT_CONFIG_HPP

#include "config/ObjectConfig.hpp"
#include "config/MeshObjectConfig.hpp"
#include "config/ElasticMaterialConfig.hpp"

#include "common/XPBDTypedefs.hpp"

#include <memory>

class XPBDMeshObjectConfig : public ObjectConfig, public MeshObjectConfig
{
    /** Static predefined default for the number of solver iterations */
    static std::optional<int>& DEFAULT_NUM_SOLVER_ITERS() { static std::optional<int> num_solver_iters(1); return num_solver_iters; }
    /** Static predefined default for solve mode */
    static std::optional<XPBDObjectSolverTypeEnum>& DEFAULT_SOLVER_TYPE() { static std::optional<XPBDObjectSolverTypeEnum> solver_type(XPBDObjectSolverTypeEnum::GAUSS_SEIDEL); return solver_type; }
    /** Static predefined default for constraint type */
    static std::optional<XPBDMeshObjectConstraintConfigurationEnum>& DEFAULT_CONSTRAINT_TYPE()
    { 
        static std::optional<XPBDMeshObjectConstraintConfigurationEnum> constraint_type(XPBDMeshObjectConstraintConfigurationEnum::STABLE_NEOHOOKEAN_COMBINED); return constraint_type; 
    }
    
    static std::optional<bool>& DEFAULT_WITH_RESIDUAL() { static std::optional<bool> with_residual(false); return with_residual; }
    static std::optional<bool>& DEFAULT_WITH_DAMPING() { static std::optional<bool> with_damping(false); return with_damping; }

    /** Static predefined default for damping stiffness */
    static std::optional<Real>& DEFAULT_DAMPING_GAMMA() { static std::optional<Real> damping_stiffness(0); return damping_stiffness; }
    /** Static predefined default for residual policy */
    static std::optional<XPBDSolverResidualPolicyEnum>& DEFAULT_RESIDUAL_POLICY() { static std::optional<XPBDSolverResidualPolicyEnum> residual_policy(XPBDSolverResidualPolicyEnum::EVERY_SUBSTEP); return residual_policy; }

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
        _material_config = std::make_unique<ElasticMaterialConfig>(node["material"]);

        // extract parameters

        _extractParameter("num-solver-iters", node, _num_solver_iters, DEFAULT_NUM_SOLVER_ITERS());
        _extractParameterWithOptions("solver-type", node, _solve_type, SOLVER_TYPE_OPTIONS(), DEFAULT_SOLVER_TYPE());
        _extractParameterWithOptions("constraint-type", node, _constraint_type, CONSTRAINT_TYPE_OPTIONS(), DEFAULT_CONSTRAINT_TYPE());
        _extractParameter("with-residual", node, _with_residual, DEFAULT_WITH_RESIDUAL());
        _extractParameter("with-damping", node, _with_damping, DEFAULT_WITH_DAMPING());
        _extractParameter("damping-gamma", node, _damping_gamma, DEFAULT_DAMPING_GAMMA());
        _extractParameterWithOptions("residual-policy", node, _residual_policy, RESIDUAL_POLICY_OPTIONS(), DEFAULT_RESIDUAL_POLICY());
        
    }

    explicit XPBDMeshObjectConfig(  const std::string& name, const Vec3r& initial_position, const Vec3r& initial_rotation,                  // Object params
                                    const Vec3r& initial_velocity, bool collisions,

                                    const std::string& filename, const std::optional<Real>& max_size, const std::optional<Vec3r>& size,     // MeshObject params
                                    bool draw_points, bool draw_edges, bool draw_faces, const Vec4r& color,

                                    Real density, Real E, Real nu, Real mu_s, Real mu_k,                                                    // ElasticMaterial params

                                    int num_solver_iters, XPBDObjectSolverTypeEnum solver_type, XPBDMeshObjectConstraintConfigurationEnum constraint_type,                   // XPBDMeshObject params
                                    bool with_residual, bool with_damping, Real damping_gamma, XPBDSolverResidualPolicyEnum residual_policy )
        : ObjectConfig(name, initial_position, initial_rotation, initial_velocity, collisions),
          MeshObjectConfig(filename, max_size, size, draw_points, draw_edges, draw_faces, color)
    {
        _material_config = std::make_unique<ElasticMaterialConfig>(name + "_material", density, E, nu, mu_s, mu_k);

        _num_solver_iters.value = num_solver_iters;
        _solve_type.value = solver_type;
        _constraint_type.value = constraint_type;
        _with_residual.value = with_residual;
        _with_damping.value = with_damping;
        _damping_gamma.value = damping_gamma;
        _residual_policy.value = residual_policy;
    }

    // Getters
    std::optional<int> numSolverIters() const { return _num_solver_iters.value; }
    std::optional<XPBDObjectSolverTypeEnum> solverType() const { return _solve_type.value; }
    XPBDMeshObjectConstraintConfigurationEnum constraintType() const { return _constraint_type.value.value(); }
    std::optional<bool> withResidual() const { return _with_residual.value; }
    std::optional<bool> withDamping() const { return _with_damping.value; }
    std::optional<Real> dampingGamma() const { return _damping_gamma.value; }
    std::optional<XPBDSolverResidualPolicyEnum> residualPolicy() const { return _residual_policy.value; }

    ElasticMaterialConfig* materialConfig() const { return _material_config.get(); }

    protected:
    // Parameters
    ConfigParameter<int> _num_solver_iters;
    ConfigParameter<XPBDObjectSolverTypeEnum> _solve_type;
    ConfigParameter<XPBDMeshObjectConstraintConfigurationEnum> _constraint_type;
    ConfigParameter<bool> _with_residual;
    ConfigParameter<bool> _with_damping;
    ConfigParameter<Real> _damping_gamma;
    ConfigParameter<XPBDSolverResidualPolicyEnum> _residual_policy;

    std::unique_ptr<ElasticMaterialConfig> _material_config;
};

#endif // __XPBD_MESH_OBJECT_CONFIG_HPP