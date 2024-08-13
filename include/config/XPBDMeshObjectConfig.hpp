#ifndef __XPBD_MESH_OBJECT_CONFIG_HPP
#define __XPBD_MESH_OBJECT_CONFIG_HPP

#include "config/ElasticMeshObjectConfig.hpp"

class XPBDMeshObjectConfig : public ElasticMeshObjectConfig
{
    public:
    /** Creates a Config from a YAML node, which consists of the specialized parameters needed for XPBDMeshObject.
     * @param node : the YAML node (i.e. dictionary of key-value pairs) that information is pulled from
     */
    explicit XPBDMeshObjectConfig(const YAML::Node& node)
        : ElasticMeshObjectConfig(node)
    {
        // extract parameters
        _extractParameter("num-solver-iters", node, _num_solver_iters);
        _extractParameter("solve-mode", node, _solve_mode);
    }

    // Getters
    std::optional<unsigned> numSolverIters() const { return _num_solver_iters.value; }
    std::optional<std::string> solveMode() const { return _solve_mode.value; }

    protected:
    // Parameters
    ConfigParameter<unsigned> _num_solver_iters;
    ConfigParameter<std::string> _solve_mode;
};

#endif // __XPBD_MESH_OBJECT_CONFIG_HPP