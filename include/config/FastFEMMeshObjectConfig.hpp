#ifndef __FAST_FEM_MESH_OBJECT_CONFIG_HPP
#define __FAST_FEM_MESH_OBJECT_CONFIG_HPP

#include "config/ElasticMeshObjectConfig.hpp"

class FastFEMMeshObjectConfig : public ElasticMeshObjectConfig
{

    public:

    explicit FastFEMMeshObjectConfig(const YAML::Node& node)
        : ElasticMeshObjectConfig(node)
    {
        // extract parameters
        
    }


};


#endif // __FAST_FEM_MESH_OBJECT_CONFIG_HPP