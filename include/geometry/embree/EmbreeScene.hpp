#ifndef __EMBREE_MANAGER_HPP
#define __EMBREE_MANAGER_HPP

#include <embree4/rtcore.h>

#include "geometry/Mesh.hpp"
#include "geometry/TetMesh.hpp"

#include "geometry/embree/EmbreeMeshGeometry.hpp"
#include "geometry/embree/EmbreeTetMeshGeometry.hpp"
#include "geometry/embree/EmbreeQueryStructs.hpp"

#include "simobject/MeshObject.hpp"

#include <map>
#include <set>

namespace Sim
{
    class Object;
    class TetMeshObject;
}

namespace Geometry
{

/** A simple struct for a point cloud that has class information associated with it.
 */
struct PointsWithClass
{
    std::vector<Vec3r> points;
    int classification;
};

/** A class for interfacing with the Embree API.
 * TODO: add high-level collision scene for broad-phase collision detection
 * TODO: replace custom triangle-ray intersection function with native Embree version. Will require using triangle geometry instead of user geometry.
 * 
 * Supports: ray-tracing queries, closest-point queries, point-in-tetrahedron queries (for tetrahedral volume meshes)
 * 
 * Multiple Embree "scenes" are maintained:
 *   - _collision_scene is for broad-phase collision detection. It maintains a BVH of AABBs for objects added to the EmbreeScene. (TODO)
 *   - _ray_scene is for finer-grain ray-tracing. It maintains a BVH of triangle primitives for meshes added to the EmbreeScene. This is useful for tracing rays to generate a partial-view point cloud, for example.
 *   - each TetMeshObject (i.e. volumetric mesh) added to the EmbreeScene maintains its own scene used for point-in-tetrahedron queries.
 * 
 * When a surface mesh (MeshObject) is added to EmbreeScene, it is added to the ray-tracing scene (_ray_scene) and the collision scene (_collision_scene).
 * 
 * When a volumetric mesh (TetMeshObject) is added to EmbreeScene, it is added to the ray-tracing scene (_ray_scene), the collision scene (_collision_scene), and its own private scene for point-in-tetrahedron queries.
 * A TetMeshObject derives from MeshObject such that the surface mesh can easily be separated from the volumetric part, which will be faster for ray-tracing queries. 
 * 
*/
class EmbreeScene
{   
    public:
    EmbreeScene();

    ~EmbreeScene();

    /** Add a generic object to the EmbreeScene */
    template<typename ObjectType>
    void addObject(const ObjectType* /*obj_ptr*/)
    {
        /** TODO: add AABB's to _collision_scene */
    }

    /** Add a tetrahedral mesh object to the EmbreeScene */
    void addObject(const Sim::TetMeshObject* obj);

    /** Add a surface mesh object to the EmbreeScene */
    void addObject(const Sim::MeshObject* obj);

    /** Updates the Embree scene. */
    void update();

    /** Samples a point cloud from the surfaces of objects in the Embree scene from a given viewpoint.
     * Rays are cast from the view origin, with horizontal angles in [-hfov_deg/2, hfov_deg/2] and vertical angles in [-vfov_deg/2, vfov_deg/2].
     * @param origin : the view origin
     * @param view_dir : the "forward" direction of the view transform
     * @param up_dir : the "up" direction of the view transform - defines the vertical direction of the sampling (i.e. vertical FOV is defined in this direction)
     * @param hfov_deg : the horizontal FOV to sample, in degrees
     * @param vfov_deg : the vertical FOV to sample, in degrees
     * @param sample_density : the number of samples per degree
     */
    std::vector<Vec3r> partialViewPointCloud(const Vec3r& origin, const Vec3r& view_dir, const Vec3r& up_dir, Real hfov_deg, Real vfov_deg, Real sample_density) const;

    std::vector<PointsWithClass> partialViewPointCloudsWithClass(const Vec3r& origin, const Vec3r& view_dir, const Vec3r& up_dir, Real hfov_deg, Real vfov_deg, Real sample_density) const;

    /** Casts a ray and reports its intersection.
     * @param ray_origin : origin of the ray
     * @param ray_dir : direction of the ray (a unit vector)
     * @returns a struct with the intersection info
     */
    EmbreeHit castRay(const Vec3r& ray_origin, const Vec3r& ray_dir) const;
    
    /** Finds the closest point on a surface mesh to the specified point.
     * @param point : the query point
     * @param obj_ptr : a pointer to the MeshObject that we should find the closest point on
     * @returns a struct with the closest point info
     */
    EmbreeHit closestPointSurfaceMesh(const Vec3r& point, const Sim::MeshObject* obj_ptr) const;

    /** Finds the closest point on the surface of a tetrahedral mesh to the specified point. 
     * @param point : the query point
     * @param obj_ptr : a poitner to the TetMeshObject that we should find the closest point on
     * @returns a struct with the closest point info
    */
    EmbreeHit closestPointTetMesh(const Vec3r& point, const Sim::TetMeshObject* obj_ptr) const;

    /** Finds the closest point on the surface of the undeformed tetrahedral mesh to the specified point. */
    EmbreeHit closestPointUndeformedTetMesh(const Vec3r& point, const Sim::TetMeshObject* obj_ptr) const;

    /** Returns all the tetrahedra in a tetrahedral mesh that contain the specified point.
     * @param point : the query point
     * @param obj_ptr : a pointer to the TetMeshObject to query
     * @returns the set of tetrahedra in the mesh that contain the specified point (can be empty)
     */
    std::set<EmbreeHit> pointInTetrahedraQuery(const Vec3r& point, const Sim::TetMeshObject* obj_ptr) const;

    /** Returns all the tetrahedra in a tetrahedral mesh that contain the specified vertex, ignoring all tetrahedra that share the vertex.
     * Used for checking for self-collisions in deformable tetrahedral meshes.
     * @param vertex_index : the index of the vertex in the tetrahedral mesh to test
     * @param obj_ptr : a pointer to the TetMeshObject that we are testing
     * @returns the set of tetrahedra in the mesh (excluding those that have the vertex in question as one of its vertices) that contain the specified vertex (can be empty)
     */
    std::set<EmbreeHit> tetMeshSelfCollisionQuery(int vertex_index, const Sim::TetMeshObject* obj_ptr) const;

    private:
    EmbreeHit _closestPointQuery(const Vec3r& point, const Sim::MeshObject* obj_ptr, const EmbreeMeshGeometry* geom, RTCScene scene) const;

    /** Embree device and scene */
    RTCDevice _device;
    RTCScene _collision_scene;
    RTCScene _ray_scene;
    
    /** maps object pointers to their Embree user geometries */ 
    std::map<const Sim::MeshObject*, EmbreeMeshGeometry*> _mesh_to_embree_geom;
    std::map<const Sim::TetMeshObject*, EmbreeTetMeshGeometry*> _tet_mesh_to_embree_geom;

    /** maps Embree geomID back to object pointers */
    std::map<unsigned, const Sim::MeshObject*> _geomID_to_mesh_obj;

    /** Stores all the Embree user geometries */
    std::vector<EmbreeMeshGeometry> _embree_mesh_geoms;
    std::vector<EmbreeTetMeshGeometry> _embree_tet_mesh_geoms;

};

} // namespace Geometry

#endif // __EMBREE_MANAGER_HPP