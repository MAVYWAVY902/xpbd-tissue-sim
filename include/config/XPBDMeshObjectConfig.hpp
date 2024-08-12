#ifndef __XPBD_MESH_OBJECT_CONFIG_HPP
#define __XPBD_MESH_OBJECT_CONFIG_HPP

#include "config/ElasticMeshObjectConfig.hpp"

class XPBDMeshObjectConfig : public ElasticMeshObjectConfig
{
    public:
    explicit XPBDMeshObjectConfig(const YAML::Node& node);

    std::optional<unsigned> numSolverIters() const { return _num_solver_iters.value; }

    protected:
    ConfigParameter<unsigned> _num_solver_iters;
};

#endif // __XPBD_MESH_OBJECT_CONFIG_HPP