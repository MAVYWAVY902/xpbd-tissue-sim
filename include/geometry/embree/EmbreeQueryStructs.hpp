#ifndef __EMBREE_QUERY_STRUCTS_HPP
#define __EMBREE_QUERY_STRUCTS_HPP

#include "common/types.hpp"

#include <set>

namespace Sim
{
    class MeshObject;
    class TetMeshObject;
}

namespace Geometry
{

class EmbreeMeshGeometry;
class EmbreeTetMeshGeometry;

/** "Hit" result (for either rays or for point queries) */
struct EmbreeHit
{
    const Sim::MeshObject* obj;     // pointer to object in sim being hit
    int prim_index;             // index of primitive hit (could be triangle or tetrahedron depending on context)
    Vec3r hit_point;            // where the primitive was hit (really only makes sense for ray intersections)

    bool operator <(const EmbreeHit& other) const
    {
        return std::tie(obj, prim_index) < std::tie(other.obj, other.prim_index);
    }

    bool operator ==(const EmbreeHit& other) const
    {
        return std::tie(obj, prim_index, hit_point) == std::tie(other.obj, other.prim_index, other.hit_point);
    }
};

/** User-defined Embree point query data to be used during Embree point-in-tetrahedra queries. */
struct EmbreePointQueryUserData
{
    const Sim::TetMeshObject* obj_ptr;  // pointer to object who owns geometry being queried
    const EmbreeTetMeshGeometry* geom;  // the geometry being queried
    std::set<EmbreeHit> result;          // the "result" of the point query - i.e. unique list of elements the point is inside
    const float* point;                 // the query point
    int vertex_ind;                     // (for self-collision queries) the vertex index of the queried point - used to exclude tetrahedra that contain the vertex
    float radius=0;                       // the radius of the point query - set to 0 for a strict point-in-tetrahedra query
};

/** User-defined Embree point query data for closest point queries. */
struct EmbreeClosestPointQueryUserData
{
    const Sim::MeshObject* obj_ptr;
    const EmbreeMeshGeometry* geom;
    EmbreeHit result;
    const float* point;
};

} // namespace Geometry

#endif // __EMBREE_QUERY_STRUCTS_HPP