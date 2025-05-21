#include "geometry/EmbreeManager.hpp"

#include "simobject/MeshObject.hpp"

namespace Geometry
{

EmbreeManager::EmbreeManager()
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

void EmbreeManager::addObject(const Sim::TetMeshObject* obj_ptr)
{
    // make sure that object has not already been added to Embree scene
    if (_obj_embree_geom.count(obj_ptr) > 0)
        assert(0 && "Object has already been added to Embree scene!");
    
    // create new EmbreeTetMeshGeometry for the object
    _obj_embree_geom.emplace(obj_ptr, obj_ptr->tetMesh());
    _obj_embree_geom.at(obj_ptr).copyVertices();

    // create Embree user geometry from newly created EmbreeTetMeshGeometry struct
    RTCGeometry rtc_geom = rtcNewGeometry(_device, RTC_GEOMETRY_TYPE_USER);
    _obj_embree_geom.at(obj_ptr).geom_id = rtcAttachGeometry(_scene, rtc_geom);

    // set the BVH build quality to REFIT - this will update the BVH rather than do a complete rebuild
    rtcSetGeometryBuildQuality(rtc_geom, RTC_BUILD_QUALITY_REFIT);
    rtcSetGeometryUserPrimitiveCount(rtc_geom, obj_ptr->tetMesh()->numElements());
    rtcSetGeometryUserData(rtc_geom, &_obj_embree_geom.at(obj_ptr));

    // set custom callbacks
    rtcSetGeometryBoundsFunction(rtc_geom, _boundsFuncTetrahedra, &_obj_embree_geom.at(obj_ptr));
    rtcSetGeometryIntersectFunction(rtc_geom, _intersectFuncTetrahedra);
    rtcSetGeometryPointQueryFunction(rtc_geom, _pointQueryFuncTetrahedra);

    // commit geometry to scene
    rtcCommitGeometry(rtc_geom);
    rtcCommitScene(_scene);     // this will build initial BVH

    rtcReleaseGeometry(rtc_geom);
}

void EmbreeManager::update()
{
    for (auto& [obj_ptr, geom] : _obj_embree_geom)
    {
        geom.copyVertices();
    }

    rtcCommitScene(_scene);
}

std::set<EmbreeManager::EmbreeHit> EmbreeManager::pointQuerySingleObject(const Vec3r& point, const Sim::TetMeshObject* obj_ptr)
{
    const EmbreeTetMeshGeometry& geom = _obj_embree_geom.at(obj_ptr);
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
    rtcPointQuery(_scene, &query, &context, _pointQueryFuncTetrahedra, &point_query_data);

    return point_query_data.result;
}

bool EmbreeManager::_isPointInTetrahedron(const float p[3], const float *v0, const float *v1, const float *v2, const float *v3)
{
    // Define vectors for the tetrahedron faces
    // float v0[3] = {tet.v0[0], tet.v0[1], tet.v0[2]};
    // float v1[3] = {tet.v1[0], tet.v1[1], tet.v1[2]};
    // float v2[3] = {tet.v2[0], tet.v2[1], tet.v2[2]};
    // float v3[3] = {tet.v3[0], tet.v3[1], tet.v3[2]};

    // Helper function to compute normal and check same side
    auto sameSide = [](const float p[3], const float a[3], const float b[3],
                       const float c[3], const float d[3]) -> bool
    {
        // Compute normal vector of face (a,b,c)
        float ab[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
        float ac[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
        float normal[3] = {
            ab[1] * ac[2] - ab[2] * ac[1],
            ab[2] * ac[0] - ab[0] * ac[2],
            ab[0] * ac[1] - ab[1] * ac[0]};

        // Compute dot products to check if p and d are on same side
        float dotP = normal[0] * (p[0] - a[0]) + normal[1] * (p[1] - a[1]) + normal[2] * (p[2] - a[2]);
        float dotD = normal[0] * (d[0] - a[0]) + normal[1] * (d[1] - a[1]) + normal[2] * (d[2] - a[2]);

        return (dotP * dotD >= 0);
    };

    // Point is inside if it's on the same side of all faces
    return sameSide(p, v0, v1, v2, v3) &&
           sameSide(p, v0, v1, v3, v2) &&
           sameSide(p, v0, v2, v3, v1) &&
           sameSide(p, v1, v2, v3, v0);
}

void EmbreeManager::_boundsFuncTetrahedra(const struct RTCBoundsFunctionArguments *args)
{
    const EmbreeTetMeshGeometry *geom = static_cast<const EmbreeTetMeshGeometry *>(args->geometryUserPtr);
    const int *indices = geom->indices() + 4 * args->primID;
    const float *v1 = geom->vertices() + 3 * indices[0];
    const float *v2 = geom->vertices() + 3 * indices[1];
    const float *v3 = geom->vertices() + 3 * indices[2];
    const float *v4 = geom->vertices() + 3 * indices[3];

    RTCBounds* bounds = args->bounds_o;
    bounds->lower_x = std::min({v1[0], v2[0], v3[0], v4[0]});
    bounds->lower_y = std::min({v1[1], v2[1], v3[1], v4[1]});
    bounds->lower_z = std::min({v1[2], v2[2], v3[2], v4[2]});

    bounds->upper_x = std::max({v1[0], v2[0], v3[0], v4[0]});
    bounds->upper_y = std::max({v1[1], v2[1], v3[1], v4[1]});
    bounds->upper_z = std::max({v1[2], v2[2], v3[2], v4[2]});
}

void EmbreeManager::_intersectFuncTetrahedra(const RTCIntersectFunctionNArguments *args)
{
    // Basic ray-tetrahedron intersection implementation
    // This function is required for RTCGeometry but we won't use it
    // for point queries directly.
}

bool EmbreeManager::_pointQueryFuncTetrahedra(RTCPointQueryFunctionArguments *args)
{
    // Get user data containing the query point and results vector
    EmbreePointQueryUserData *userData = static_cast<EmbreePointQueryUserData *>(args->userPtr);
    // Get the geometry data
    const EmbreeTetMeshGeometry *geom = userData->geom;

    // only consider point queries for the geometry we're interested in
    // TODO: should we do point queries for 
    if (args->geomID != geom->geom_id)
        return true;

    
    const int *indices = geom->indices() + 4 * args->primID;
    const float *v1 = geom->vertices() + 3 * indices[0];
    const float *v2 = geom->vertices() + 3 * indices[1];
    const float *v3 = geom->vertices() + 3 * indices[2];
    const float *v4 = geom->vertices() + 3 * indices[3];

    
    const float *point = userData->point;

    // Check if the point is inside this tetrahedron
    if (_isPointInTetrahedron(point, v1, v2, v3, v4))
    {
        EmbreeHit hit;
        hit.obj = userData->obj_ptr;
        hit.prim_index = args->primID;
        // Add this tetrahedron's ID to the results
        userData->result.insert(hit);

        // Continue searching for other potential tetrahedra
        // (in case of overlapping tetrahedra or numerical issues)
        return true;
    }

    return true; // Continue traversal to find all potential tetrahedra
}

} // namespace Geometry