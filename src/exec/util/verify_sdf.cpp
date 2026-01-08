/**
 * Verify SDF generation and quality
 * 
 * This tool:
 * 1. Loads a mesh
 * 2. Generates SDF (same way as collision detection does)
 * 3. Analyzes SDF properties (inside/outside distribution)
 * 4. Samples the zero-level surface
 * 5. Outputs diagnostics and surface points
 * 
 * Usage: ./VerifySDF <mesh_file.obj> [output_dir]
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

#include "geometry/Mesh.hpp"
#include "geometry/MeshSDF.hpp"
#include "utils/MeshUtils.hpp"

int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <mesh_file.obj> [output_dir]" << std::endl;
        return 1;
    }

    std::string mesh_file = argv[1];
    std::string output_dir = (argc > 2) ? argv[2] : "output/sdf_verification";

    // Create output directory
    system(("mkdir -p " + output_dir).c_str());

    std::cout << "=== SDF VERIFICATION TOOL ===\n" << std::endl;
    std::cout << "Loading mesh: " << mesh_file << std::endl;

    // Load mesh
    Geometry::Mesh mesh = MeshUtils::loadSurfaceMeshFromFile(mesh_file);
    
    std::cout << "  Vertices: " << mesh.numVertices() << std::endl;
    std::cout << "  Faces: " << mesh.numFaces() << std::endl;
    
    auto bbox = mesh.boundingBox();
    std::cout << "  BBox: (" << bbox.min.transpose() << ") to (" 
              << bbox.max.transpose() << ")" << std::endl;
    
    Vec3r mesh_size = bbox.max - bbox.min;
    std::cout << "  Size: " << mesh_size.transpose() << " m" << std::endl;
    std::cout << "  Size: " << (mesh_size * 1000).transpose() << " mm" << std::endl;

    // Compute mass center
    Vec3r mass_center = mesh.massCenter();
    std::cout << "  Mass center: " << mass_center.transpose() << std::endl;

    // Generate SDF using Mesh2SDF (same as MeshSDF.cpp does)
    const int grid_resolution = 128;
    const int padding = 5;
    std::cout << "\nGenerating SDF (" << grid_resolution << "x" << grid_resolution << "x" << grid_resolution << " grid, padding=" << padding << ")..." << std::endl;
    
    mesh2sdf::MeshSDF sdf(mesh.vertices(), mesh.faces(), grid_resolution, padding, true);
    
    std::cout << "\n=== SDF PROPERTIES ===" << std::endl;
    
    auto grid_bbox = sdf.gridBoundingBox();
    std::cout << "Grid bbox: (" << grid_bbox.first[0] << ", " << grid_bbox.first[1] << ", " << grid_bbox.first[2] << ") to ("
              << grid_bbox.second[0] << ", " << grid_bbox.second[1] << ", " << grid_bbox.second[2] << ")" << std::endl;
    
    Vec3r grid_size_vec = grid_bbox.second - grid_bbox.first;
    std::cout << "Grid size: " << grid_size_vec.transpose() << " m" << std::endl;
    
    Vec3r cell_size = sdf.gridCellSize();
    std::cout << "Cell size: " << cell_size.transpose() << " m" << std::endl;
    
    auto mesh_bbox_sdf = sdf.meshBoundingBox();
    Vec3r mesh_center_sdf = sdf.meshMassCenter();
    std::cout << "Mesh center (from SDF): " << mesh_center_sdf.transpose() << std::endl;

    // Sample SDF at key points
    std::cout << "\n=== SAMPLING SDF ===" << std::endl;
    
    struct TestPoint {
        std::string name;
        Vec3r pos;
    };
    
    std::vector<TestPoint> test_points = {
        {"Origin", Vec3r(0, 0, 0)},
        {"Mass center", mass_center},
        {"First vertex", mesh.vertex(0)},
        {"Mid vertex", mesh.vertex(mesh.numVertices()/2)},
        {"BBox center", (bbox.min + bbox.max) / 2},
    };
    
    std::cout << "\nDistance values at test points:" << std::endl;
    for (const auto& tp : test_points) {
        Real dist = sdf.evaluate(tp.pos);
        Vec3r grad = sdf.gradient(tp.pos);
        std::cout << "  " << tp.name << ": pos=" << tp.pos.transpose() 
                  << ", dist=" << dist << " m = " << (dist*1000) << " mm" << std::endl;
        std::cout << "    gradient: " << grad.transpose() << ", norm=" << grad.norm() << std::endl;
    }

    // Analyze distance grid distribution
    std::cout << "\n=== SDF DISTANCE DISTRIBUTION ===" << std::endl;
    
    const auto& distance_grid = sdf.distanceGrid();
    
    // Count negative/positive/zero distances
    int negative_count = 0;
    int positive_count = 0;
    int zero_count = 0;
    Real min_dist = 1e10;
    Real max_dist = -1e10;
    
    // Array3 stores as flat array, calculate dimensions from grid_resolution
    const int ni = grid_resolution + 2 * padding;
    const int nj = grid_resolution + 2 * padding;
    const int nk = grid_resolution + 2 * padding;
    
    for (int i = 0; i < ni; ++i) {
        for (int j = 0; j < nj; ++j) {
            for (int k = 0; k < nk; ++k) {
                Real dist = distance_grid(i, j, k);
                
                if (dist < 0) negative_count++;
                else if (dist > 0) positive_count++;
                else zero_count++;
                
                min_dist = std::min(min_dist, dist);
                max_dist = std::max(max_dist, dist);
            }
        }
    }
    
    int total_voxels = ni * nj * nk;
    
    std::cout << "Grid shape: " << ni << " x " << nj << " x " << nk << std::endl;
    std::cout << "Distance range: [" << min_dist << ", " << max_dist << "] m" << std::endl;
    std::cout << "Distance range: [" << (min_dist*1000) << ", " << (max_dist*1000) << "] mm" << std::endl;
    
    std::cout << "\nVoxel classification:" << std::endl;
    std::cout << "  Negative (inside):  " << negative_count << " (" 
              << (100.0*negative_count/total_voxels) << "%)" << std::endl;
    std::cout << "  Zero (surface):     " << zero_count << " (" 
              << (100.0*zero_count/total_voxels) << "%)" << std::endl;
    std::cout << "  Positive (outside): " << positive_count << " (" 
              << (100.0*positive_count/total_voxels) << "%)" << std::endl;

    // Sample surface points (near zero-level)
    std::cout << "\n=== EXTRACTING SURFACE POINTS ===" << std::endl;
    
    std::vector<Vec3r> surface_points;
    const int step = 2;  // Sample every 2nd voxel
    const Real surface_threshold = 0.002;  // Within 2mm of surface
    
    for (int i = 0; i < ni; i += step) {
        for (int j = 0; j < nj; j += step) {
            for (int k = 0; k < nk; k += step) {
                Real dist = distance_grid(i, j, k);
                
                if (std::abs(dist) < surface_threshold) {
                    // Convert grid indices to world coordinates
                    Vec3r world_pos = grid_bbox.first + Vec3r(i, j, k).cwiseProduct(cell_size);
                    surface_points.push_back(world_pos);
                }
            }
        }
    }
    
    std::cout << "  Found " << surface_points.size() << " points near zero-level surface" << std::endl;
    
    // Save surface points as OBJ
    std::string surface_file = output_dir + "/sdf_surface_points.obj";
    std::ofstream out(surface_file);
    for (const auto& pt : surface_points) {
        out << "v " << pt[0] << " " << pt[1] << " " << pt[2] << "\n";
    }
    out.close();
    std::cout << "  Saved surface points: " << surface_file << std::endl;

    // Summary
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "SUMMARY" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    if (negative_count == 0) {
        std::cout << "❌ SDF IS INVALID - No inside region detected!" << std::endl;
        std::cout << "   Reason: Mesh is not watertight (has holes)" << std::endl;
        std::cout << "   Solution: Close the mesh holes in MeshLab or Blender" << std::endl;
        std::cout << "\n   Inside/outside determination will be WRONG!" << std::endl;
        std::cout << "   Collision detection will report all points as 'outside'" << std::endl;
    } else if (negative_count < 0.01 * total_voxels) {
        std::cout << "⚠️  SDF may be problematic - Very small inside region" << std::endl;
        std::cout << "   Only " << (100.0*negative_count/total_voxels) << "% of voxels are inside" << std::endl;
    } else {
        std::cout << "✓ SDF appears valid - Inside/outside regions detected" << std::endl;
        std::cout << "   " << (100.0*negative_count/total_voxels) << "% of voxels are inside" << std::endl;
    }
    
    std::cout << "\nOutput saved to: " << output_dir << "/" << std::endl;

    return 0;
}
