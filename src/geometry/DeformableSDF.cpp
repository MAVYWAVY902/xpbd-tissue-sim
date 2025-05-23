#include "geometry/DeformableSDF.hpp"

namespace Geometry
{

DeformableSDF::DeformableSDF(const Sim::TetMeshObject* obj, const EmbreeScene* embree_scene)
    : _obj(obj), _embree_scene(embree_scene)
{}

Real DeformableSDF::evaluate(const Vec3r& x) const
{
    // find enclosing tetrahedron for query point
    
    // map query point back to undeformed configuration (Xm = F^-1 (x - v4) + v4)

    // query MeshSDF with Xm

    // find closest point on surface

    // map closest point on surface back to deformed configuration

    return 0;
}

Vec3r DeformableSDF::gradient(const Vec3r& x) const
{
    // TODO
    return Vec3r(0,0,0);
}

} // namespace Geometry