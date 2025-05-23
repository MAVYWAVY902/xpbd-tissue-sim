#include "geometry/embree/EmbreeScene.hpp"
#include "geometry/embree/EmbreeQueryStructs.hpp"

#include "simobject/MeshObject.hpp"

namespace Geometry
{

EmbreeScene::EmbreeScene()
{
    _device = rtcNewDevice(NULL);

    if (!_device)
    {
        std::cout << "Error " << rtcGetDeviceError(NULL) << ": cannot create device" << std::endl;
        assert(0);
    }

    _scene = rtcNewScene(_device);
    rtcSetSceneFlags(_scene, RTC_SCENE_FLAG_DYNAMIC);
}

void EmbreeScene::addObject(const Sim::MeshObject* obj_ptr)
{
    // make sure that object has not already been added to Embree scene
    if (_mesh_embree_geom.count(obj_ptr) > 0)
        assert(0 && "Object has already been added to Embree scene!");

    // create new EmbreeMeshGeometry for the object
    _mesh_embree_geom.emplace(obj_ptr, obj_ptr->mesh());
    EmbreeMeshGeometry& geom = _mesh_embree_geom.at(obj_ptr);
    geom.copyVertices();

    // create Embree user geometry from newly created EmbreeMeshGeometry
    RTCGeometry rtc_geom = rtcNewGeometry(_device, RTC_GEOMETRY_TYPE_USER);
    geom.setMeshGeomID( rtcAttachGeometry(_scene, rtc_geom) );

    // set BVH build quality to REFT - this will update the BVH rather than do a complete rebuild
    rtcSetGeometryBuildQuality(rtc_geom, RTC_BUILD_QUALITY_REFIT);
    rtcSetGeometryUserPrimitiveCount(rtc_geom, obj_ptr->mesh()->numFaces());
    rtcSetGeometryUserData(rtc_geom, &geom);

    // set custom callbacks
    rtcSetGeometryBoundsFunction(rtc_geom, EmbreeMeshGeometry::boundsFuncTriangle, &geom);
    rtcSetGeometryIntersectFunction(rtc_geom, EmbreeMeshGeometry::intersectFuncTriangle);
    rtcSetGeometryPointQueryFunction(rtc_geom, EmbreeMeshGeometry::pointQueryFuncTriangle);

    // commit geometry to scene
    rtcCommitGeometry(rtc_geom);
    rtcCommitScene(_scene);     // this will build BVH

    rtcReleaseGeometry(rtc_geom);
}

void EmbreeScene::addObject(const Sim::TetMeshObject* obj_ptr)
{
    // make sure that object has not already been added to Embree scene
    if (_tet_mesh_embree_geom.count(obj_ptr) > 0)
        assert(0 && "Object has already been added to Embree scene!");
    
    // create new EmbreeTetMeshGeometry for the object
    _tet_mesh_embree_geom.emplace(obj_ptr, obj_ptr->tetMesh());
    EmbreeTetMeshGeometry& geom = _tet_mesh_embree_geom.at(obj_ptr);

    // create Embree user geometry from newly created EmbreeTetMeshGeometry struct
    // we create 2 Embree geometries - one for the volumetric representation and one for the surface of the mesh
    RTCGeometry rtc_mesh_geom = rtcNewGeometry(_device, RTC_GEOMETRY_TYPE_USER);
    geom.setMeshGeomID( rtcAttachGeometry(_scene, rtc_mesh_geom) );
    RTCGeometry rtc_tet_mesh_geom = rtcNewGeometry(_device, RTC_GEOMETRY_TYPE_USER);
    geom.setTetMeshGeomID( rtcAttachGeometry(_scene, rtc_tet_mesh_geom) );

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
    rtcCommitGeometry(rtc_tet_mesh_geom);
    rtcCommitScene(_scene);     // this will build initial BVH

    rtcReleaseGeometry(rtc_mesh_geom);
    rtcReleaseGeometry(rtc_tet_mesh_geom);
}

void EmbreeScene::update()
{
    for (auto& [obj_ptr, geom] : _mesh_embree_geom)
    {
        geom.copyVertices();
    }

    for (auto& [obj_ptr, geom] : _tet_mesh_embree_geom)
    {
        geom.copyVertices();
    }

    rtcCommitScene(_scene);
}

EmbreeHit EmbreeScene::rayCastSurfaceMesh(const Vec3r& ray_origin, const Vec3r& ray_dir, const Sim::MeshObject* obj_ptr)
{
    // TODO
    EmbreeHit hit;
    return hit;
}
    
EmbreeHit EmbreeScene::closestPointSurfaceMesh(const Vec3r& point, const Sim::MeshObject* obj_ptr)
{
    const EmbreeMeshGeometry& geom = _mesh_embree_geom.at(obj_ptr);
    return _closestPointQuery(point, obj_ptr, &geom);
}

EmbreeHit EmbreeScene::closestPointTetMesh(const Vec3r& point, const Sim::TetMeshObject* obj_ptr)
{
    const EmbreeTetMeshGeometry& geom = _tet_mesh_embree_geom.at(obj_ptr);
    return _closestPointQuery(point, obj_ptr, &geom);
}

EmbreeHit EmbreeScene::_closestPointQuery(const Vec3r& point, const Sim::MeshObject* obj_ptr, const EmbreeMeshGeometry* geom)
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
    rtcPointQuery(_scene, &query, &context, EmbreeMeshGeometry::pointQueryFuncTriangle, &point_query_data);

}

std::set<EmbreeHit> EmbreeScene::pointQueryTetMesh(const Vec3r& point, const Sim::TetMeshObject* obj_ptr)
{
    const EmbreeTetMeshGeometry& geom = _tet_mesh_embree_geom.at(obj_ptr);
    EmbreePointQueryUserData point_query_data;
    point_query_data.obj_ptr = obj_ptr;
    point_query_data.geom = &geom;
    
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
    rtcPointQuery(_scene, &query, &context, EmbreeTetMeshGeometry::pointQueryFuncTetrahedra, &point_query_data);

    return point_query_data.result;
}

} // namespace Geometry