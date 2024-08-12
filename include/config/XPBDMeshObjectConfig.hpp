#ifndef __XPBD_MESH_OBJECT_CONFIG_HPP
#define __XPBD_MESH_OBJECT_CONFIG_HPP

#include "config/ElasticMeshObjectConfig.hpp"

class XPBDMeshObjectConfig : public ElasticMeshObjectConfig
{
    public:
    explicit XPBDMeshObjectConfig(const YAML::Node& node)
        : ElasticMeshObjectConfig(node)
    {
        _extractParameter("num-solver-iters", node, _num_solver_iters);
    }

    std::optional<unsigned> numSolverIters() const { return _num_solver_iters.value; }

    protected:
    ConfigParameter<unsigned> _num_solver_iters;
};

#endif // __XPBD_MESH_OBJECT_CONFIG_HPP