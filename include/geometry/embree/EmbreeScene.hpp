#ifndef __EMBREE_MANAGER_HPP
#define __EMBREE_MANAGER_HPP

#include <embree4/rtcore.h>

#include "geometry/Mesh.hpp"
#include "geometry/TetMesh.hpp"

#include "geometry/embree/EmbreeMeshGeometry.hpp"
#include "geometry/embree/EmbreeTetMeshGeometry.hpp"
#include "geometry/embree/EmbreeQueryStructs.hpp"

#include <map>
#include <set>

namespace Sim
{
    class Object;
    class TetMeshObject;
}

namespace Geometry
{

/** A class for interfacing with the Embree API.
 * TODO: add high-level collision scene for broad-phase collision detection
 * TODO: replace custom triangle-ray intersection function with native Embree version. Will require using triangle geometry instead of user geometry.
 * 
 * Supports: ray-tracing queries, closest-point queries, point-in-tetrahedron queries (for tetrahedral volume meshes)
 * 
 * Multiple Embree "scenes" are maintained:
 *   - _collision_scene is for broad-phase collision detection. It maintains a BVH of AABBs for objects added to the EmbreeScene.
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

    /** Adds a MeshObject (i.e. a surface mesh) to the Embree scene */
    void addObject(const Sim::MeshObject* obj_ptr);

    /** Adds a TetMeshObject (i.e. a volumetric mesh) to the Embree scene */
    void addObject(const Sim::TetMeshObject* obj_ptr);

    /** Updates the Embree scene. */
    void update();

    EmbreeHit castRay(const Vec3r& ray_origin, const Vec3r& ray_dir) const;
    
    EmbreeHit closestPointSurfaceMesh(const Vec3r& point, const Sim::MeshObject* obj_ptr) const;
    EmbreeHit closestPointTetMesh(const Vec3r& point, const Sim::TetMeshObject* obj_ptr) const;
    std::set<EmbreeHit> pointInTetrahedraQuery(const Vec3r& point, const Sim::TetMeshObject* obj_ptr) const;

    private:
    EmbreeHit _closestPointQuery(const Vec3r& point, const Sim::MeshObject* obj_ptr, const EmbreeMeshGeometry* geom) const;

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