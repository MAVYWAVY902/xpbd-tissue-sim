#ifndef __EMBREE_MESH_GEOMETRY_HPP
#define __EMBREE_MESH_GEOMETRY_HPP

#include <embree4/rtcore.h>

#include "geometry/Mesh.hpp"


namespace Geometry
{


/** User-defined Embree gometry to create BVH for surface meshes. */
class EmbreeMeshGeometry
{
    public:

    explicit EmbreeMeshGeometry(const Geometry::Mesh* mesh);
    
    const Mesh* mesh() const { return _mesh; }
    unsigned meshGeomID() const { return _mesh_geom_id; }
    void setMeshGeomID(unsigned id) { _mesh_geom_id = id; }

    /** Returns a pointer to face indices (3 consecutive indices make up a face). */
    const int* faceIndices() const { return _mesh->faces().data(); }

    /** Returns a pointer to the vertices. */
    const float* vertices() const;

    /** Copies mesh vertices to float vertex buffer. */
    void copyVertices();

    static void boundsFuncTriangle(const struct RTCBoundsFunctionArguments *args);
    static void intersectFuncTriangle(const RTCIntersectFunctionNArguments *args);
    static bool pointQueryFuncTriangle(RTCPointQueryFunctionArguments *args);

    private:

    static void _closestPointTriangle(const float p_in[3], const float v0[3], const float v1[3], const float v2[3], float p_out[3]);

    const Geometry::Mesh* _mesh;
    std::vector<float> _vertex_buffer;
    unsigned _mesh_geom_id;
};

} // namespace Geometry

#endif // __EMBREE_MESH_GEOMETRY_HPP