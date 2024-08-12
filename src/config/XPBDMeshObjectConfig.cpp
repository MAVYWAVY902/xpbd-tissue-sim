#include "config/XPBDMeshObjectConfig.hpp"

XPBDMeshObjectConfig::XPBDMeshObjectConfig(const YAML::Node& node)
    : ElasticMeshObjectConfig(node)
{
    _extractParameter("num-solver-iters", node, _num_solver_iters);
}