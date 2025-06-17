#ifndef __FIRST_ORDER_XPBD_MESH_OBJECT_CONFIG_HPP
#define __FIRST_ORDER_XPBD_MESH_OBJECT_CONFIG_HPP

#include "config/simobject/XPBDMeshObjectConfig.hpp"

namespace Config
{

class FirstOrderXPBDMeshObjectConfig : public XPBDMeshObjectConfig
{
    public:
    using ObjectType = Sim::XPBDMeshObject_Base;

    static std::optional<Real>& DEFAULT_DAMPING_MULTIPLIER() { static std::optional<Real> damping(1); return damping; }

    public:
    /** Creates a Config from a YAML node, which consists of the specialized parameters needed for XPBDMeshObject.
     * @param node : the YAML node (i.e. dictionary of key-value pairs) that information is pulled from
     */
    explicit FirstOrderXPBDMeshObjectConfig(const YAML::Node& node)
        : XPBDMeshObjectConfig(node)
    {
        _extractParameter("damping-multiplier", node, _damping_multiplier, DEFAULT_DAMPING_MULTIPLIER());        
    }

    explicit FirstOrderXPBDMeshObjectConfig(  
                                    const std::string& name, const Vec3r& initial_position, const Vec3r& initial_rotation,                  // Object params
                                    const Vec3r& initial_velocity, bool collisions, bool graphics_only,

                                    const std::string& filename, const std::optional<Real>& max_size, const std::optional<Vec3r>& size,     // MeshObject params
                                    bool draw_points, bool draw_edges, bool draw_faces, const Vec4r& color,

                                    Real density, Real E, Real nu, Real mu_s, Real mu_k,                                                    // ElasticMaterial params

                                    int num_solver_iters, XPBDObjectSolverTypeEnum solver_type, XPBDMeshObjectConstraintConfigurationEnum constraint_type,                   // XPBDMeshObject params
                                    bool with_residual, bool with_damping, Real damping_gamma, XPBDSolverResidualPolicyEnum residual_policy,
                                
                                    Real damping_multiplier )  
                                                                                                                                            // FirstOrderXPBDMeshObject params
        : XPBDMeshObjectConfig(name, initial_position, initial_rotation, initial_velocity, collisions, graphics_only,
                                filename, max_size, size, draw_points, draw_edges, draw_faces, color,
                                density, E, nu, mu_s, mu_k,
                                num_solver_iters, solver_type, constraint_type, with_residual, with_damping, damping_gamma, residual_policy)
    {
        _damping_multiplier.value = damping_multiplier;
    }

    std::unique_ptr<ObjectType> createObject(const Sim::Simulation* sim) const;

    std::optional<Real> dampingMultiplier() const { return _damping_multiplier.value; }

    protected:
    ConfigParameter<Real> _damping_multiplier;
};

} // namespace Config

#endif // __FIRST_ORDER_XPBD_MESH_OBJECT_CONFIG_HPP