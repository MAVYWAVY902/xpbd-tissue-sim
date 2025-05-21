#ifndef __EMBREE_MANAGER_HPP
#define __EMBREE_MANAGER_HPP

#include <embree4/rtcore.h>

#include <geometry/TetMesh.hpp>

#include <map>
#include <set>

namespace Sim
{
    class Object;
    class TetMeshObject;
}

namespace Geometry
{

/** A class for interfacing with the Embree API */
class EmbreeManager
{
    public:
    /** "Hit" result (for either rays or for point queries) */
    struct EmbreeHit
    {
        const Sim::TetMeshObject* obj;     // pointer to object in sim being hit
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


    private:

    /** User-defined Embree geometry that is used to create the BVH */
    struct EmbreeTetMeshGeometry
    {
        const Geometry::TetMesh* mesh;      // the volumetric (tetrahedral) mesh to create the BVH for
        std::vector<float> vertex_buffer;   // Embree only works with floats, so we need to maintain a float buffer of vertices
        unsigned geom_id;                        // Embree geometry ID in the scene

        explicit EmbreeTetMeshGeometry(const Geometry::TetMesh* mesh_)
            : mesh(mesh_)
        {
            // if double precision is being used in the sim, we must allocate space for the float vertex buffer
            if constexpr (std::is_same_v<Real, double>)
                vertex_buffer.resize(mesh->numVertices()*3);
        }

        /** Returns a pointer to element indices (i.e. 4 consecutive indices make up an element) */
        const int* indices() const
        {
            return mesh->elements().data();
        }

        /** Returns a pointer to the vertices */
        const float* vertices() const
        {
            // if double precision is being used in the sim, we need to return the float vertex buffer
            if constexpr (std::is_same_v<Real, double>)
            {
                return vertex_buffer.data();
            }
            // otherwise we can just use the mesh vertices themselves
            else
            {
                return reinterpret_cast<const float*>(mesh->vertices().data()); // the reinterpret_cast is used to avoid compile error (this branch is not taken if Real = double)
            }
        }

        /** Copies mesh vertices to float vertex buffer.
         * This function does nothing if the sim is using single precision
         */
        void copyVertices()
        {
            if constexpr (std::is_same_v<Real, double>)
            {
                const Real* mesh_vertices = mesh->vertices().data();
                for (int i = 0; i < mesh->numVertices()*3; i++)
                {
                    vertex_buffer[i] = static_cast<float>(mesh_vertices[i]);
                }
            }
        }
    };

    /** User-defined Embree point query data to be used during Embree point queries. */
    struct EmbreePointQueryUserData
    {
        const Sim::TetMeshObject* obj_ptr;  // pointer to object who owns geometry being queried
        const EmbreeTetMeshGeometry* geom;  // the geometry being queried
        std::set<EmbreeHit> result;          // the "result" of the point query - i.e. unique list of elements the point is inside
        const float* point;                 // the query point
    };
    

    public:
    EmbreeManager();

    void addObject(const Sim::TetMeshObject* obj_ptr);

    /** Updates the Embree scene. */
    void update();

    std::set<EmbreeHit> pointQuerySingleObject(const Vec3r& point, const Sim::TetMeshObject* obj_ptr);



    private:
    static bool _isPointInTetrahedron(const float p[3], const float *v0, const float *v1, const float *v2, const float *v3);
    static void _boundsFuncTetrahedra(const struct RTCBoundsFunctionArguments *args);
    static void _intersectFuncTetrahedra(const RTCIntersectFunctionNArguments *args);
    static bool _pointQueryFuncTetrahedra(RTCPointQueryFunctionArguments *args);

    private:
    /** Embree device and scene */
    RTCDevice _device;
    RTCScene _scene;
    
    /** maps object pointers to their Embree user geometries */ 
    std::map<const Sim::TetMeshObject*, EmbreeTetMeshGeometry> _obj_embree_geom;

};

} // namespace Geometry

#endif // __EMBREE_MANAGER_HPP