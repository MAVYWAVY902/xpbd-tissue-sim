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

/** A class for interfacing with the Embree API */
class EmbreeScene
{
    public:
    

    public:
    EmbreeScene();

    void addObject(const Sim::MeshObject* obj_ptr);
    void addObject(const Sim::TetMeshObject* obj_ptr);

    /** Updates the Embree scene. */
    void update();

    EmbreeHit rayCastSurfaceMesh(const Vec3r& ray_origin, const Vec3r& ray_dir, const Sim::MeshObject* obj_ptr);
    
    EmbreeHit closestPointSurfaceMesh(const Vec3r& point, const Sim::MeshObject* obj_ptr);
    EmbreeHit closestPointTetMesh(const Vec3r& point, const Sim::TetMeshObject* obj_ptr);
    std::set<EmbreeHit> pointQueryTetMesh(const Vec3r& point, const Sim::TetMeshObject* obj_ptr);

    private:
    EmbreeHit _closestPointQuery(const Vec3r& point, const Sim::MeshObject* obj_ptr, const EmbreeMeshGeometry* geom);

    /** Embree device and scene */
    RTCDevice _device;
    RTCScene _scene;    // TODO: should there be multiple scenes? (e.g. one for surface meshes and one for volume meshes)
    
    /** maps object pointers to their Embree user geometries */ 
    std::map<const Sim::MeshObject*, EmbreeMeshGeometry> _mesh_embree_geom;
    std::map<const Sim::TetMeshObject*, EmbreeTetMeshGeometry> _tet_mesh_embree_geom;

};

} // namespace Geometry

#endif // __EMBREE_MANAGER_HPP