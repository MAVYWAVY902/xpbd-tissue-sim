#include "config/ObjectConfig.hpp"
#include "config/MeshObjectConfig.hpp"

#include "geometry/TetMesh.hpp"
#include "geometry/embree/EmbreeScene.hpp"

#include "simobject/MeshObject.hpp"

#include "utils/MeshUtils.hpp"

#include <embree4/rtcore.h>

#include <iostream>
#include <limits>
#include <set>
#include <chrono>

/* -------------------------------------------------------------------------- */

int main()
{
    gmsh::initialize();
    // load mesh
    // Geometry::TetMesh tet_mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/demos/trachea_virtuoso/tracheal_tumor_v2_refined.msh");
    MeshObjectConfig mesh_config("../resource/cube/cube8.msh", 1, std::nullopt,
        false, false, true, Vec4r(0,0,0,0));

    ObjectConfig object_config("test", Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), true, false);

    Sim::TetMeshObject mesh_obj(&mesh_config, &object_config);
    mesh_obj.loadAndConfigureMesh();
    // Geometry::TetMesh tet_mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/general/single.msh");
    // Geometry::TetMesh tet_mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/cube/cube16.msh");

    // create the EmbreeScene object to interface with Embree
    Geometry::EmbreeScene embree_scene;

    // add object(s) to EmbreeScene
    embree_scene.addObject(&mesh_obj);

    // point-in-tetrahedron query
    const Vec3r query_point(0.4,0.35,0.2);
    std::set<Geometry::EmbreeHit> result = embree_scene.pointInTetrahedraQuery(query_point, &mesh_obj);

    std::cout << "\n=== Results for point-in-tet query for query point (" << query_point[0] << ", " << query_point[1] << ", " << query_point[2] << ") ===" << std::endl;
    for (const auto& hit : result)
    {
        std::cout << " Hit! Details: " << std::endl;
        std::cout << "   Tet index: " << hit.prim_index << std::endl;

        const Eigen::Vector4i& elem = mesh_obj.tetMesh()->element(hit.prim_index);
        const Vec3r& v1 = mesh_obj.tetMesh()->vertex(elem[0]);
        const Vec3r& v2 = mesh_obj.tetMesh()->vertex(elem[1]);
        const Vec3r& v3 = mesh_obj.tetMesh()->vertex(elem[2]);
        const Vec3r& v4 = mesh_obj.tetMesh()->vertex(elem[3]);

        std::cout << "Elem v1: " << v1[0] << ", " << v1[1] << ", " << v1[2] << std::endl;
        std::cout << "Elem v2: " << v2[0] << ", " << v2[1] << ", " << v2[2] << std::endl;
        std::cout << "Elem v3: " << v3[0] << ", " << v3[1] << ", " << v3[2] << std::endl;
        std::cout << "Elem v4: " << v4[0] << ", " << v4[1] << ", " << v4[2] << std::endl;
    }

    // closest point query
    const Vec3r cp_query_point(0.7, 0.1, 0.1);
    Geometry::EmbreeHit cp_result = embree_scene.closestPointTetMesh(cp_query_point, &mesh_obj);
    std::cout << "\n=== Results for closest-point query for query point (" << cp_query_point[0] << ", " << cp_query_point[1] << ", " << cp_query_point[2] << ") ===" << std::endl;
    const Eigen::Vector3i& face = mesh_obj.tetMesh()->face(cp_result.prim_index);
    const Vec3r& v1 = mesh_obj.tetMesh()->vertex(face[0]);
    const Vec3r& v2 = mesh_obj.tetMesh()->vertex(face[1]);
    const Vec3r& v3 = mesh_obj.tetMesh()->vertex(face[2]);

    std::cout << "Face v1: " << v1[0] << ", " << v1[1] << ", " << v1[2] << std::endl;
    std::cout << "Face v2: " << v2[0] << ", " << v2[1] << ", " << v2[2] << std::endl;
    std::cout << "Face v3: " << v3[0] << ", " << v3[1] << ", " << v3[2] << std::endl;

    std::cout << "closest point: " << cp_result.hit_point[0] << ", " << cp_result.hit_point[1] << ", " << cp_result.hit_point[2] << std::endl;

    // ray-tracing
    const Vec3r ray_origin(100, 0.38, 0.31);
    const Vec3r ray_dir(-1, 0, 0);
    Geometry::EmbreeHit rt_result = embree_scene.castRay(ray_origin, ray_dir);
    std::cout << "\n=== Results for ray query for ray (" << ray_origin[0] << ", " << ray_origin[1] << ", " << ray_origin[2] << ") with dir (" << ray_dir[0] << ", " << ray_dir[1] << ", " << ray_dir[2] << ") ===" << std::endl;
    if (rt_result.obj == nullptr)
    {
        std::cout << "  No hit detected!" << std::endl;
    }
    else
    {
        const Eigen::Vector3i& face = rt_result.obj->mesh()->face(rt_result.prim_index);
        const Vec3r& v1 = rt_result.obj->mesh()->vertex(face[0]);
        const Vec3r& v2 = rt_result.obj->mesh()->vertex(face[1]);
        const Vec3r& v3 = rt_result.obj->mesh()->vertex(face[2]);

        std::cout << "Face v1: " << v1[0] << ", " << v1[1] << ", " << v1[2] << std::endl;
        std::cout << "Face v2: " << v2[0] << ", " << v2[1] << ", " << v2[2] << std::endl;
        std::cout << "Face v3: " << v3[0] << ", " << v3[1] << ", " << v3[2] << std::endl;

        std::cout << "hit point: " << rt_result.hit_point[0] << ", " << rt_result.hit_point[1] << ", " << rt_result.hit_point[2] << std::endl;
    }
    
    

    return 0;
}