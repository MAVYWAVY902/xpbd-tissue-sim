#include "geometry/TetMesh.hpp"
#include "utils/MeshUtils.hpp"

#include <embree4/rtcore.h>

#include <iostream>
#include <limits>
#include <set>
#include <chrono>


struct EmbreeTetrahedraGeometry
{
    float *vertices;
    int *indices;
    RTCDevice device;
    RTCScene scene;
    int geom_id;
};

// Structure to hold results from point queries
struct PointQueryUserData {
    EmbreeTetrahedraGeometry* geom;
    std::set<unsigned>* result;
    const float* point;
};

/*
 * We will register this error handler with the device in initializeDevice(),
 * so that we are automatically informed on errors.
 * This is extremely helpful for finding bugs in your code, prevents you
 * from having to add explicit error checking to each Embree API call.
 */
void errorFunction(void *userPtr, enum RTCError error, const char *str)
{
    printf("error %d: %s\n", error, str);
}

bool isPointInTetrahedron(const float p[3], const float *v0, const float *v1, const float *v2, const float *v3)
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

void calculateTetBounds(const float *v1, const float *v2, const float *v3, const float *v4, RTCBounds* bounds)
{
    bounds->lower_x = std::min({v1[0], v2[0], v3[0], v4[0]});
    bounds->lower_y = std::min({v1[1], v2[1], v3[1], v4[1]});
    bounds->lower_z = std::min({v1[2], v2[2], v3[2], v4[2]});

    bounds->upper_x = std::max({v1[0], v2[0], v3[0], v4[0]});
    bounds->upper_y = std::max({v1[1], v2[1], v3[1], v4[1]});
    bounds->upper_z = std::max({v1[2], v2[2], v3[2], v4[2]});
}

void boundsFuncTetrahedra(const struct RTCBoundsFunctionArguments *args)
{
    const EmbreeTetrahedraGeometry *geom = static_cast<const EmbreeTetrahedraGeometry *>(args->geometryUserPtr);
    const int *indices = geom->indices + 4 * args->primID;
    const float *v1 = geom->vertices + 3 * indices[0];
    const float *v2 = geom->vertices + 3 * indices[1];
    const float *v3 = geom->vertices + 3 * indices[2];
    const float *v4 = geom->vertices + 3 * indices[3];

    calculateTetBounds(v1, v2, v3, v4, args->bounds_o);
}

// Callback for ray-tetrahedron intersection
void intersectFuncTetrahedra(const RTCIntersectFunctionNArguments *args)
{
    // Basic ray-tetrahedron intersection implementation
    // This function is required for RTCGeometry but we won't use it
    // for point queries directly.
}

bool pointQueryFuncTetrahedra(RTCPointQueryFunctionArguments *args)
{
    // Get user data containing the query point and results vector
    PointQueryUserData *userData = static_cast<PointQueryUserData *>(args->userPtr);

    // Get the geometry data
    const EmbreeTetrahedraGeometry *geom = userData->geom;
    const int *indices = geom->indices + 4 * args->primID;
    const float *v1 = geom->vertices + 3 * indices[0];
    const float *v2 = geom->vertices + 3 * indices[1];
    const float *v3 = geom->vertices + 3 * indices[2];
    const float *v4 = geom->vertices + 3 * indices[3];

    
    const float *point = userData->point;

    // Check if the point is inside this tetrahedron
    if (isPointInTetrahedron(point, v1, v2, v3, v4))
    {
        // Add this tetrahedron's ID to the results
        userData->result->insert(args->primID);

        // Continue searching for other potential tetrahedra
        // (in case of overlapping tetrahedra or numerical issues)
        return true;
    }

    return true; // Continue traversal to find all potential tetrahedra
}

/*
 * Embree has a notion of devices, which are entities that can run
 * raytracing kernels.
 * We initialize our device here, and then register the error handler so that
 * we don't miss any errors.
 *
 * rtcNewDevice() takes a configuration string as an argument. See the API docs
 * for more information.
 *
 * Note that RTCDevice is reference-counted.
 */
RTCDevice initializeDevice()
{
    RTCDevice device = rtcNewDevice(NULL);

    if (!device)
        printf("error %d: cannot create device\n", rtcGetDeviceError(NULL));

    rtcSetDeviceErrorFunction(device, errorFunction, NULL);
    return device;
}

/*
 * Create a scene, which is a collection of geometry objects. Scenes are
 * what the intersect / occluded functions work on. You can think of a
 * scene as an acceleration structure, e.g. a bounding-volume hierarchy.
 *
 * Scenes, like devices, are reference-counted.
 */
EmbreeTetrahedraGeometry initializeScene(RTCDevice device, float *vertices, int *indices, int num_elements)
{
    RTCScene scene = rtcNewScene(device);

    // create tetrahedral geometry
    EmbreeTetrahedraGeometry geom;
    geom.geom_id = 0;
    geom.device = device;
    geom.scene = scene;
    geom.vertices = vertices;
    geom.indices = indices;

    RTCGeometry rtcGeom = rtcNewGeometry(geom.device, RTC_GEOMETRY_TYPE_USER);
    geom.geom_id = rtcAttachGeometry(geom.scene, rtcGeom);

    // Set scene flags for refit build quality
    rtcSetSceneFlags(scene, RTC_SCENE_FLAG_DYNAMIC);

    // rtcSetGeometryBuildQuality(rtcGeom, RTC_BUILD_QUALITY_LOW);
    rtcSetGeometryBuildQuality(rtcGeom, RTC_BUILD_QUALITY_REFIT);
    rtcSetGeometryUserPrimitiveCount(rtcGeom, num_elements);

    rtcSetGeometryUserData(rtcGeom, &geom);
    rtcSetGeometryBoundsFunction(rtcGeom, boundsFuncTetrahedra, &geom);
    rtcSetGeometryIntersectFunction(rtcGeom, intersectFuncTetrahedra);

    // Set point query function - this is the key addition for point queries
    rtcSetGeometryPointQueryFunction(rtcGeom, pointQueryFuncTetrahedra);

    // Commit geometry and scene to finalize
    rtcCommitGeometry(rtcGeom);
    rtcCommitScene(geom.scene);
    
    rtcReleaseGeometry(rtcGeom);
    return geom;
}

/* -------------------------------------------------------------------------- */

int main()
{
    gmsh::initialize();
    // load mesh
    // Geometry::TetMesh tet_mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/demos/trachea_virtuoso/tracheal_tumor_v2_refined.msh");
    Geometry::TetMesh tet_mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/general/single.msh");
    // Geometry::TetMesh tet_mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/cube/cube16.msh");

    std::vector<float> float_vertices(3*tet_mesh.numVertices());
    auto copy_t1 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < tet_mesh.numVertices(); i++)
    {
        const Vec3r& v = tet_mesh.vertex(i);
        float_vertices[3*i] = v[0];
        float_vertices[3*i+1] = v[1];
        float_vertices[3*i+2] = v[2];
    }
    auto copy_t2 = std::chrono::high_resolution_clock::now();
    auto copy_millis = std::chrono::duration_cast<std::chrono::nanoseconds>(copy_t2 - copy_t1).count() / 1e6;
    std::cout << "Copy time: " << copy_millis << " ms" << std::endl;


    /* Initialization. All of this may fail, but we will be notified by
     * our errorFunction. */
    RTCDevice device = initializeDevice();

    auto t1 = std::chrono::high_resolution_clock::now();
    EmbreeTetrahedraGeometry geom = initializeScene(device, float_vertices.data(), tet_mesh.elements().data(), tet_mesh.numElements());
    auto t2 = std::chrono::high_resolution_clock::now();
    auto millis = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1e6;
    std::cout << "Construction time: " << millis << " ms" << std::endl;

    // translate mesh +1 in z direction
    for (int i = 0; i < float_vertices.size(); i++)
    {
        float_vertices[i] *= 1.7;
    }
    auto t3 = std::chrono::high_resolution_clock::now();
    // rtcSetSceneBuildQuality(geom.scene, RTC_BUILD_QUALITY_REFIT);
    rtcCommitScene(geom.scene);
    auto t4 = std::chrono::high_resolution_clock::now();
    millis = std::chrono::duration_cast<std::chrono::nanoseconds>(t4 - t3).count() / 1e6;
    std::cout << "Refit time: " << millis << " ms" << std::endl;

    /** Test point query */
    const Eigen::Vector3f query_point(3, 0, 0);

    std::set<unsigned> result;
    PointQueryUserData user_data;
    user_data.geom = &geom;
    user_data.result = &result;
    user_data.point = query_point.data();
    

    RTCPointQuery query;
    query.x = query_point[0];
    query.y = query_point[1];
    query.z = query_point[2];
    query.radius = 0.0f;  // Exact containment

    RTCPointQueryContext context;
    rtcInitPointQueryContext(&context);
    rtcPointQuery(geom.scene, &query, &context, pointQueryFuncTetrahedra, &user_data);


    /** Print results */
    std::cout << "=== Intersection Test === " << std::endl;
    std::cout << "Number of intersecting tetrahedra: " << result.size() << std::endl;
    for (const auto& hit : result)
    {
        std::cout << "\tElement index hit: " << hit << std::endl;
    }

    std::cout << "\n=== IsPointInTetrahedraTest ===" << std::endl;
    std::cout << "Point: " << query_point[0] << ", " << query_point[1] << ", " << query_point[2] << std::endl;
    const int prim_id = 0;
    const int *indices = geom.indices + 4 * prim_id;
    const float *v1 = geom.vertices + 3 * indices[0];
    const float *v2 = geom.vertices + 3 * indices[1];
    const float *v3 = geom.vertices + 3 * indices[2];
    const float *v4 = geom.vertices + 3 * indices[3]; 
    bool is_inside = isPointInTetrahedron(query_point.data(), v1, v2, v3, v4);
    std::cout << "Element " << prim_id << ":" << std::endl;
    std::cout << "  v1: " << v1[0] << ", " << v1[1] << ", " << v1[2] << std::endl;
    std::cout << "  v2: " << v2[0] << ", " << v2[1] << ", " << v2[2] << std::endl;
    std::cout << "  v3: " << v3[0] << ", " << v3[1] << ", " << v3[2] << std::endl;
    std::cout << "  v4: " << v4[0] << ", " << v4[1] << ", " << v4[2] << std::endl;
    std::cout << "In tetrahedra " << prim_id << ": " << is_inside << std::endl;

    /* Though not strictly necessary in this example, you should
     * always make sure to release resources allocated through Embree. */
    rtcReleaseScene(geom.scene);
    rtcReleaseDevice(device);

    return 0;
}