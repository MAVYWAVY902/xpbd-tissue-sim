/**
 * Test SDF collision detection
 * 
 * Load both meshes, generate bone SDF, query tumor vertices against it
 * to see what distances we should expect during collision detection
 */

#include <iostream>
#include <vector>
#include <algorithm>

#define MESH2SDF_DOUBLE_PRECISION
#include <Mesh2SDF/MeshSDF.hpp>

#include "geometry/Mesh.hpp"
#include "geometry/TetMesh.hpp"
#include "utils/MeshUtils.hpp"

int main(int argc, char* argv[])
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <rigid_bone.obj> <deformable_tumor.msh>" << std::endl;
        return 1;
    }

    std::string bone_file = argv[1];
    std::string tumor_file = argv[2];

    std::cout << "=== SDF COLLISION TEST ===\n" << std::endl;

    // Load bone mesh and generate SDF
    std::cout << "Loading bone mesh: " << bone_file << std::endl;
    Geometry::Mesh bone_mesh = MeshUtils::loadSurfaceMeshFromFile(bone_file);
    
    std::cout << "  Vertices: " << bone_mesh.numVertices() << std::endl;
    std::cout << "  Faces: " << bone_mesh.numFaces() << std::endl;
    
    auto bone_bbox = bone_mesh.boundingBox();
    Vec3r bone_size = bone_bbox.max - bone_bbox.min;
    Vec3r bone_center = bone_mesh.massCenter();
    std::cout << "  Size: " << (bone_size * 1000).transpose() << " mm" << std::endl;
    std::cout << "  Mass center: " << bone_center.transpose() << std::endl;

    // Generate SDF
    std::cout << "\nGenerating bone SDF..." << std::endl;
    const int grid_resolution = 128;
    const int padding = 5;
    mesh2sdf::MeshSDF bone_sdf(bone_mesh.vertices(), bone_mesh.faces(), grid_resolution, padding, true);

    // Load tumor mesh
    std::cout << "\nLoading tumor mesh: " << tumor_file << std::endl;
    Geometry::Mesh tumor_mesh = MeshUtils::loadSurfaceMeshFromFile(tumor_file);
    
    std::cout << "  Vertices: " << tumor_mesh.numVertices() << std::endl;
    std::cout << "  Faces: " << tumor_mesh.numFaces() << std::endl;
    
    auto tumor_bbox = tumor_mesh.boundingBox();
    Vec3r tumor_size = tumor_bbox.max - tumor_bbox.min;
    Vec3r tumor_center = tumor_mesh.massCenter();
    std::cout << "  Size: " << (tumor_size * 1000).transpose() << " mm" << std::endl;
    std::cout << "  Mass center: " << tumor_center.transpose() << std::endl;

    // Distance between centers
    Real center_distance = (tumor_center - bone_center).norm();
    std::cout << "\nDistance between mesh centers: " << (center_distance * 1000) << " mm" << std::endl;

    // Query SDF at tumor vertices
    std::cout << "\n=== QUERYING SDF AT TUMOR VERTICES ===" << std::endl;
    
    std::vector<Real> distances;
    distances.reserve(tumor_mesh.numVertices());
    
    Real min_dist = 1e10;
    Real max_dist = -1e10;
    int negative_count = 0;
    int near_surface_count = 0;
    
    Vec3r closest_vertex_pos;
    Real closest_dist = 1e10;
    
    for (int i = 0; i < tumor_mesh.numVertices(); ++i) {
        const Vec3r& vertex = tumor_mesh.vertex(i);
        Real dist = bone_sdf.evaluate(vertex);
        
        distances.push_back(dist);
        
        if (dist < 0) negative_count++;
        if (std::abs(dist) < 0.001) near_surface_count++;  // Within 1mm
        
        min_dist = std::min(min_dist, dist);
        max_dist = std::max(max_dist, dist);
        
        if (dist < closest_dist) {
            closest_dist = dist;
            closest_vertex_pos = vertex;
        }
    }
    
    // Statistics
    Real mean_dist = 0;
    for (Real d : distances) mean_dist += d;
    mean_dist /= distances.size();
    
    std::sort(distances.begin(), distances.end());
    Real median_dist = distances[distances.size() / 2];
    Real percentile_5 = distances[distances.size() * 5 / 100];
    Real percentile_95 = distances[distances.size() * 95 / 100];
    
    std::cout << "\nDistance statistics:" << std::endl;
    std::cout << "  Min distance:  " << (min_dist * 1000) << " mm" << std::endl;
    std::cout << "  Max distance:  " << (max_dist * 1000) << " mm" << std::endl;
    std::cout << "  Mean distance: " << (mean_dist * 1000) << " mm" << std::endl;
    std::cout << "  Median distance: " << (median_dist * 1000) << " mm" << std::endl;
    std::cout << "  5th percentile:  " << (percentile_5 * 1000) << " mm" << std::endl;
    std::cout << "  95th percentile: " << (percentile_95 * 1000) << " mm" << std::endl;
    
    std::cout << "\nVertex classification:" << std::endl;
    std::cout << "  Inside bone (negative):  " << negative_count << " ("
              << (100.0 * negative_count / tumor_mesh.numVertices()) << "%)" << std::endl;
    std::cout << "  Near surface (< 1mm):    " << near_surface_count << " ("
              << (100.0 * near_surface_count / tumor_mesh.numVertices()) << "%)" << std::endl;
    
    std::cout << "\nClosest tumor vertex to bone:" << std::endl;
    std::cout << "  Position: " << closest_vertex_pos.transpose() << std::endl;
    std::cout << "  Distance: " << (closest_dist * 1000) << " mm" << std::endl;

    // Collision detection threshold
    const Real collision_threshold = 1e-3;  // 1mm
    int would_collide = 0;
    for (Real d : distances) {
        if (d <= collision_threshold) would_collide++;
    }
    
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "COLLISION DETECTION PREDICTION" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    std::cout << "Using threshold: " << (collision_threshold * 1000) << " mm" << std::endl;
    std::cout << "Vertices that would trigger collision: " << would_collide << " / " 
              << tumor_mesh.numVertices() << std::endl;
    
    if (would_collide > 0) {
        std::cout << "\n✓ COLLISION SHOULD BE DETECTED" << std::endl;
        std::cout << "  " << would_collide << " vertices are within collision threshold" << std::endl;
    } else if (min_dist < 0.01) {  // Within 10mm
        std::cout << "\n⚠ NO COLLISION YET, but meshes are close" << std::endl;
        std::cout << "  Closest point is " << (min_dist * 1000) << " mm away" << std::endl;
        std::cout << "  Apply initial velocity or deformation to cause collision" << std::endl;
    } else {
        std::cout << "\n✗ MESHES ARE FAR APART" << std::endl;
        std::cout << "  Minimum distance: " << (min_dist * 1000) << " mm" << std::endl;
        std::cout << "  Collision will NOT occur unless meshes move closer" << std::endl;
        std::cout << "\nPOSSIBLE ISSUES:" << std::endl;
        std::cout << "  - Check 'position' in config (both should be [0,0,0])" << std::endl;
        std::cout << "  - Check 'max-size' is same for both objects" << std::endl;
        std::cout << "  - Verify 'use-original-coords: false' for both" << std::endl;
    }

    return 0;
}
