#ifndef __FAST_FEM_MESH_OBJECT_HPP
#define __FAST_FEM_MESH_OBJECT_HPP

#include "ElasticMeshObject.hpp"
#include "FastFEMMeshObjectConfig.hpp"


class FastFEMMeshObject : public ElasticMeshObject
{
    public:
    
    explicit FastFEMMeshObject(const FastFEMMeshObjectConfig* config);

    virtual std::string toString() const override;
    virtual std::string type() const override { return "FastFEMMeshObject"; }

    void update(const double dt, const double g_accel) override;

    private:
    
    void _init();

    void _precomputeQuantities();

};

#endif // __FAST_FEM_MESH_OBJECT_HPP