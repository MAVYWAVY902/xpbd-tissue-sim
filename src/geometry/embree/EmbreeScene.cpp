#include "geometry/embree/EmbreeScene.hpp"
#include "geometry/embree/EmbreeQueryStructs.hpp"

namespace Geometry
{

EmbreeScene::EmbreeScene()
    : _device(nullptr), _collision_scene(nullptr), _ray_scene(nullptr)
{
    _device = rtcNewDevice(NULL);

    if (!_device)
    {
        std::cout << "Error " << rtcGetDeviceError(NULL) << ": cannot create device" << std::endl;
        assert(0);
    }

    _ray_scene = rtcNewScene(_device);
    rtcSetSceneFlags(_ray_scene, RTC_SCENE_FLAG_DYNAMIC);
}

EmbreeScene::~EmbreeScene()
{
    if (_collision_scene)
        rtcReleaseScene(_collision_scene);

    if (_ray_scene)
        rtcReleaseScene(_ray_scene);

    rtcReleaseDevice(_device);
    
}

void EmbreeScene::addObject(const Sim::MeshObject* obj_ptr)
{
    // make sure that object has not already been added to Embree scene
    if (_mesh_to_embree_geom.count(obj_ptr) > 0)
        assert(0 && "Object has already been added to Embree scene!");

    // create new EmbreeMeshGeometry for the object
    _embree_mesh_geoms.emplace_back(obj_ptr->mesh());
    EmbreeMeshGeometry& geom = _embree_mesh_geoms.back();
    geom.copyVertices();

    _mesh_to_embree_geom[obj_ptr] = &geom;

    // create Embree user geometry from newly created EmbreeMeshGeometry
    RTCGeometry rtc_geom = rtcNewGeometry(_device, RTC_GEOMETRY_TYPE_USER);
    geom.setMeshGeomID( rtcAttachGeometry(_ray_scene, rtc_geom) );
    _geomID_to_mesh_obj[geom.meshGeomID()] = obj_ptr;

    // set BVH build quality to REFIT - this will update the BVH rather than do a complete rebuild
    rtcSetGeometryBuildQuality(rtc_geom, RTC_BUILD_QUALITY_REFIT);
    rtcSetGeometryUserPrimitiveCount(rtc_geom, obj_ptr->mesh()->numFaces());
    rtcSetGeometryUserData(rtc_geom, &geom);

    // set custom callbacks
    rtcSetGeometryBoundsFunction(rtc_geom, EmbreeMeshGeometry::boundsFuncTriangle, &geom);
    rtcSetGeometryIntersectFunction(rtc_geom, EmbreeMeshGeometry::intersectFuncTriangle);
    rtcSetGeometryPointQueryFunction(rtc_geom, EmbreeMeshGeometry::pointQueryFuncTriangle);

    // commit geometry to scene
    rtcCommitGeometry(rtc_geom);
    rtcCommitScene(_ray_scene);     // this will build BVH

    rtcReleaseGeometry(rtc_geom);
}

void EmbreeScene::addObject(const Sim::TetMeshObject* obj_ptr)
{
    std::cout << "\n\nADDING TETMESHOBJECT TO EMBREE SCENE\n\n" << std::endl;
    // make sure that object has not already been added to Embree scene
    if (_tet_mesh_to_embree_geom.count(obj_ptr) > 0)
        assert(0 && "Object has already been added to Embree scene!");
    
    // create new EmbreeTetMeshGeometry for the object
    _embree_tet_mesh_geoms.emplace_back(obj_ptr->tetMesh());
    EmbreeTetMeshGeometry& geom = _embree_tet_mesh_geoms.back();
    geom.copyVertices();

    // store the new user geometry by its pointer in the maps
    _tet_mesh_to_embree_geom[obj_ptr] = &geom;
    _mesh_to_embree_geom[obj_ptr] = &geom;

    // create a new scene for the TetMesh exclusively for point-in-tetrahedra queries
    RTCScene tet_mesh_scene = rtcNewScene(_device);
    rtcSetSceneFlags(tet_mesh_scene, RTC_SCENE_FLAG_DYNAMIC);
    geom.setTetScene(tet_mesh_scene);
    

    // create Embree user geometry from newly created EmbreeTetMeshGeometry struct
    // we create 2 Embree geometries - one for the volumetric representation and one for the surface of the mesh
    RTCGeometry rtc_mesh_geom = rtcNewGeometry(_device, RTC_GEOMETRY_TYPE_USER);
    geom.setMeshGeomID( rtcAttachGeometry(_ray_scene, rtc_mesh_geom) );
    _geomID_to_mesh_obj[geom.meshGeomID()] = obj_ptr;

    RTCGeometry rtc_tet_mesh_geom = rtcNewGeometry(_device, RTC_GEOMETRY_TYPE_USER);
    geom.setTetMeshGeomID( rtcAttachGeometry(tet_mesh_scene, rtc_tet_mesh_geom) );

    // set the BVH build quality to REFIT - this will update the BVH rather than do a complete rebuild
    rtcSetGeometryBuildQuality(rtc_mesh_geom, RTC_BUILD_QUALITY_REFIT);
    rtcSetGeometryUserPrimitiveCount(rtc_mesh_geom, obj_ptr->mesh()->numFaces());
    rtcSetGeometryUserData(rtc_mesh_geom, &geom);

    rtcSetGeometryBuildQuality(rtc_tet_mesh_geom, RTC_BUILD_QUALITY_REFIT);
    rtcSetGeometryUserPrimitiveCount(rtc_tet_mesh_geom, obj_ptr->tetMesh()->numElements());
    rtcSetGeometryUserData(rtc_tet_mesh_geom, &geom);

    // set custom callbacks
    rtcSetGeometryBoundsFunction(rtc_mesh_geom, EmbreeMeshGeometry::boundsFuncTriangle, &geom);
    rtcSetGeometryIntersectFunction(rtc_mesh_geom, EmbreeMeshGeometry::intersectFuncTriangle);
    rtcSetGeometryPointQueryFunction(rtc_mesh_geom, EmbreeMeshGeometry::pointQueryFuncTriangle);

    rtcSetGeometryBoundsFunction(rtc_tet_mesh_geom, EmbreeTetMeshGeometry::boundsFuncTetrahedra, &geom);
    rtcSetGeometryIntersectFunction(rtc_tet_mesh_geom, EmbreeTetMeshGeometry::intersectFuncTetrahedra);
    rtcSetGeometryPointQueryFunction(rtc_tet_mesh_geom, EmbreeTetMeshGeometry::pointQueryFuncTetrahedra);

    // commit geometry to scene
    rtcCommitGeometry(rtc_mesh_geom);
    rtcCommitScene(_ray_scene);     // this will build initial BVH

    rtcCommitGeometry(rtc_tet_mesh_geom);
    rtcCommitScene(tet_mesh_scene);

    rtcReleaseGeometry(rtc_mesh_geom);
    rtcReleaseGeometry(rtc_tet_mesh_geom);
}


void EmbreeScene::update()
{
    for (auto& geom : _embree_mesh_geoms)
    {
        geom.copyVertices();
    }

    for (auto& geom : _embree_tet_mesh_geoms)
    {
        geom.copyVertices();
        rtcCommitScene(geom.tetScene());
    }

    rtcCommitScene(_ray_scene);
}

EmbreeHit EmbreeScene::castRay(const Vec3r& ray_origin, const Vec3r& ray_dir) const
{
    RTCRayHit rayhit;
    rayhit.ray.org_x = ray_origin[0];
    rayhit.ray.org_y = ray_origin[1];
    rayhit.ray.org_z = ray_origin[2];

    rayhit.ray.dir_x = ray_dir[0];
    rayhit.ray.dir_y = ray_dir[1];
    rayhit.ray.dir_z = ray_dir[2];

    rayhit.ray.tnear = 0;
    rayhit.ray.tfar = std::numeric_limits<float>::infinity();
    rayhit.ray.flags = 0;
    rayhit.ray.time = 0;
    rayhit.ray.mask = -1;

    rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

    rtcIntersect1(_ray_scene, &rayhit, nullptr);

    // check if we have a hit
    if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID)
    {
        const Sim::MeshObject* obj = _geomID_to_mesh_obj.at(rayhit.hit.geomID);
        const Vec3i& f = obj->mesh()->face(rayhit.hit.primID);
        const Vec3r& v1 = obj->mesh()->vertex(f[0]);
        const Vec3r& v2 = obj->mesh()->vertex(f[1]);
        const Vec3r& v3 = obj->mesh()->vertex(f[2]);

        EmbreeHit hit;
        hit.obj = obj;
        hit.prim_index = rayhit.hit.primID;
        hit.hit_point = v1*rayhit.hit.u + v2*rayhit.hit.v + v3*(1 - rayhit.hit.u - rayhit.hit.v);
        return hit;
    }
    else
    {
        EmbreeHit hit;
        hit.obj = nullptr;
        return hit;
    }
}
    
EmbreeHit EmbreeScene::closestPointSurfaceMesh(const Vec3r& point, const Sim::MeshObject* obj_ptr) const
{
    const EmbreeMeshGeometry* geom = _mesh_to_embree_geom.at(obj_ptr);
    return _closestPointQuery(point, obj_ptr, geom);
}

EmbreeHit EmbreeScene::closestPointTetMesh(const Vec3r& point, const Sim::TetMeshObject* obj_ptr) const
{
    const EmbreeTetMeshGeometry* geom = _tet_mesh_to_embree_geom.at(obj_ptr);
    return _closestPointQuery(point, obj_ptr, geom);
}

EmbreeHit EmbreeScene::_closestPointQuery(const Vec3r& point, const Sim::MeshObject* obj_ptr, const EmbreeMeshGeometry* geom) const
{
    EmbreeClosestPointQueryUserData point_query_data;
    point_query_data.obj_ptr = obj_ptr;
    point_query_data.geom = geom;

    float p[3];
    p[0] = point[0]; p[1] = point[1]; p[2] = point[2];
    point_query_data.point = p;

    RTCPointQuery query;
    query.x = p[0];
    query.y = p[1];
    query.z = p[2];
    query.radius = std::numeric_limits<float>::infinity(); // the query radius will get refined as we go

    RTCPointQueryContext context;
    rtcInitPointQueryContext(&context);
    rtcPointQuery(_ray_scene, &query, &context, EmbreeMeshGeometry::pointQueryFuncTriangle, &point_query_data);

    return point_query_data.result;

}

std::set<EmbreeHit> EmbreeScene::pointInTetrahedraQuery(const Vec3r& point, const Sim::TetMeshObject* obj_ptr) const
{
    const EmbreeTetMeshGeometry* geom = _tet_mesh_to_embree_geom.at(obj_ptr);
    EmbreePointQueryUserData point_query_data;
    point_query_data.obj_ptr = obj_ptr;
    point_query_data.geom = geom;
    
    float p[3];
    p[0] = point[0]; p[1] = point[1]; p[2] = point[2];
    point_query_data.point = p;

    RTCPointQuery query;
    query.x = p[0];
    query.y = p[1];
    query.z = p[2];
    query.radius = 0.0f;

    RTCPointQueryContext context;
    rtcInitPointQueryContext(&context);
    rtcPointQuery(geom->tetScene(), &query, &context, nullptr, &point_query_data);

    return point_query_data.result;
}

} // namespace Geometry