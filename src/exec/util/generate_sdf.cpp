/**
 * Utility to benchmark and generate SDF data for rigid mesh objects
 * 
 * NOTE: mesh2sdf library has its own binary format when loading from file,
 * but doesn't expose a save/write method in the public API. This tool
 * benchmarks SDF generation and exports the raw grid data.
 * 
 * Usage:
 *   ./GenerateSDF <input_mesh.obj> [resolution=128] [padding=5]
 * 
 * Example:
 *   ./GenerateSDF ../resource/bone/tbone_800.obj 128 5
 */

#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <chrono>

#define MESH2SDF_DOUBLE_PRECISION
#include <Mesh2SDF/MeshSDF.hpp>

#include "utils/MeshUtils.hpp"
#include "geometry/Mesh.hpp"

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <input_mesh> [resolution=128] [padding=5]" << std::endl;
        std::cerr << std::endl;
        std::cerr << "Examples:" << std::endl;
        std::cerr << "  " << argv[0] << " ../resource/bone/tbone_800.obj" << std::endl;
        std::cerr << "  " << argv[0] << " ../resource/bone/tbone_800.obj 256 8" << std::endl;
        std::cerr << std::endl;
        std::cerr << "This tool benchmarks SDF generation time for performance analysis." << std::endl;
        return 1;
    }

    std::string input_file = argv[1];
    int resolution = (argc > 2) ? std::atoi(argv[2]) : 128;
    int padding = (argc > 3) ? std::atoi(argv[3]) : 5;

    std::cout << "========================================" << std::endl;
    std::cout << "SDF Generation Benchmark" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Input mesh:  " << input_file << std::endl;
    std::cout << "Resolution:  " << resolution << "³ grid" << std::endl;
    std::cout << "Padding:     " << padding << " cells" << std::endl;
    std::cout << "========================================" << std::endl;

    try {
        // Load the mesh
        std::cout << "\n[1/4] Loading mesh..." << std::endl;
        Geometry::Mesh mesh = MeshUtils::loadSurfaceMeshFromFile(input_file);
        std::cout << "  Vertices: " << mesh.numVertices() << std::endl;
        std::cout << "  Faces: " << mesh.numFaces() << std::endl;

        // Get mesh properties
        auto bbox = mesh.boundingBox();
        Vec3r mesh_size = bbox.size();
        Vec3r mesh_center = bbox.center();
        std::cout << "  Bounding box size: [" << mesh_size.x() << ", " << mesh_size.y() << ", " << mesh_size.z() << "]" << std::endl;
        std::cout << "  Bounding box center: [" << mesh_center.x() << ", " << mesh_center.y() << ", " << mesh_center.z() << "]" << std::endl;

        // The mesh should be centered at origin for SDF generation
        std::cout << "\n[2/4] Centering mesh at origin..." << std::endl;
        Vec3r mass_center = mesh.massCenter();
        std::cout << "  Original mass center: [" << mass_center.x() << ", " << mass_center.y() << ", " << mass_center.z() << "]" << std::endl;
        mesh.moveTogether(-mass_center);
        std::cout << "  New mass center: [" << mesh.massCenter().x() << ", " << mesh.massCenter().y() << ", " << mesh.massCenter().z() << "]" << std::endl;

        // Generate SDF
        std::cout << "\n[3/4] Generating SDF (this may take a while)..." << std::endl;
        auto start = std::chrono::high_resolution_clock::now();
        
        mesh2sdf::MeshSDF sdf(mesh.vertices(), mesh.faces(), resolution, padding, true);
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << "  SDF generation completed in " << duration.count() / 1000.0 << " seconds" << std::endl;

        // Get SDF properties
        auto grid_bbox = sdf.gridBoundingBox();
        Vec3r grid_size(grid_bbox.second[0] - grid_bbox.first[0],
                       grid_bbox.second[1] - grid_bbox.first[1],
                       grid_bbox.second[2] - grid_bbox.first[2]);
        Vec3r cell_size = sdf.gridCellSize();
        std::cout << "  Grid size: [" << grid_size.x() << ", " << grid_size.y() << ", " << grid_size.z() << "]" << std::endl;
        std::cout << "  Cell size: [" << cell_size.x() << ", " << cell_size.y() << ", " << cell_size.z() << "]" << std::endl;

        // Calculate memory usage
        const auto& dist_grid = sdf.distanceGrid();
        const auto& grad_grid = sdf.gradientGrid();
        size_t dist_grid_bytes = dist_grid.size() * sizeof(Real);
        size_t grad_grid_bytes = grad_grid.size() * sizeof(Vec3r);
        size_t total_bytes = dist_grid_bytes + grad_grid_bytes;
        
        std::cout << "  Distance grid memory: " << dist_grid_bytes / (1024.0 * 1024.0) << " MB" << std::endl;
        std::cout << "  Gradient grid memory: " << grad_grid_bytes / (1024.0 * 1024.0) << " MB" << std::endl;
        std::cout << "  Total memory: " << total_bytes / (1024.0 * 1024.0) << " MB" << std::endl;

        std::cout << "\n========================================" << std::endl;
        std::cout << "BENCHMARK COMPLETE" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "Generation time: " << duration.count() / 1000.0 << " seconds" << std::endl;
        std::cout << "Memory usage: " << total_bytes / (1024.0 * 1024.0) << " MB" << std::endl;
        std::cout << "\nPERFORMANCE ANALYSIS:" << std::endl;
        
        if (duration.count() > 5000) {
            std::cout << "⚠ WARNING: SDF generation takes " << duration.count() / 1000.0 << "s" << std::endl;
            std::cout << "  This will slow down simulation startup significantly." << std::endl;
            std::cout << "\nRECOMMENDATIONS:" << std::endl;
            std::cout << "  1. Consider using a lower resolution (current: " << resolution << ")" << std::endl;
            std::cout << "  2. Simplify the mesh geometry to reduce face count" << std::endl;
            std::cout << "  3. The mesh2sdf library doesn't expose save/load methods," << std::endl;
            std::cout << "     so pre-generation is not currently possible." << std::endl;
        } else {
            std::cout << "✓ SDF generation is reasonably fast (" << duration.count() / 1000.0 << "s)" << std::endl;
            std::cout << "  Startup overhead should be acceptable for this mesh." << std::endl;
        }
        
        std::cout << "\nNOTE: mesh2sdf library uses a custom binary format for loading" << std::endl;
        std::cout << "from files, but doesn't expose save/write methods in the public API." << std::endl;
        std::cout << "To use pre-generated SDFs, you would need to modify the mesh2sdf" << std::endl;
        std::cout << "library or use a different SDF library with save/load support." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "\nERROR: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
