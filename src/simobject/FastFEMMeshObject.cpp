#include "FastFEMMeshObject.hpp"

FastFEMMeshObject::FastFEMMeshObject(const FastFEMMeshObjectConfig* config)
    : ElasticMeshObject(config)
{
    _init();
    _precomputeQuantities();
}

void FastFEMMeshObject::_init()
{

}

void FastFEMMeshObject::_precomputeQuantities()
{

}

std::string FastFEMMeshObject::toString() const
{
    return ElasticMeshObject::toString();
}

void FastFEMMeshObject::update(const double dt, const double g_accel)
{

}