/**
 * Extract surface mesh from SDF using marching cubes
 * 
 * This tool:
 * 1. Loads a mesh
 * 2. Generates SDF
 * 3. Extracts zero-level isosurface (the surface)
 * 4. Saves as OBJ for comparison with original
 * 
 * Usage: ./ExtractSDFSurface <input.obj> <output.obj>
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <cmath>

#include "geometry/Mesh.hpp"
#include "geometry/MeshSDF.hpp"
#include "utils/MeshUtils.hpp"

// Simple marching cubes implementation for zero-level surface extraction
class MarchingCubes
{
public:
    MarchingCubes(const mesh2sdf::Array3<Real>& distance_grid,
                  const Vec3r& grid_origin,
                  const Vec3r& cell_size)
        : _grid(distance_grid), _origin(grid_origin), _cell_size(cell_size)
    {
        // Calculate grid dimensions
        _ni = 128 + 2*5;  // resolution + 2*padding
        _nj = 128 + 2*5;
        _nk = 128 + 2*5;
    }

    void extractSurface(std::vector<Vec3r>& vertices, std::vector<Eigen::Vector3i>& faces)
    {
        vertices.clear();
        faces.clear();
        
        // Edge to vertex index map (for sharing vertices)
        std::unordered_map<uint64_t, int> edge_to_vertex;
        
        // Process each cell
        for (int i = 0; i < _ni-1; ++i) {
            for (int j = 0; j < _nj-1; ++j) {
                for (int k = 0; k < _nk-1; ++k) {
                    processCube(i, j, k, vertices, faces, edge_to_vertex);
                }
            }
        }
        
        std::cout << "Extracted " << vertices.size() << " vertices, " 
                  << faces.size() << " triangles" << std::endl;
    }

private:
    const mesh2sdf::Array3<Real>& _grid;
    Vec3r _origin;
    Vec3r _cell_size;
    int _ni, _nj, _nk;
    
    void processCube(int i, int j, int k,
                     std::vector<Vec3r>& vertices,
                     std::vector<Eigen::Vector3i>& faces,
                     std::unordered_map<uint64_t, int>& edge_to_vertex)
    {
        // Get 8 corner values
        Real values[8];
        values[0] = _grid(i,   j,   k);
        values[1] = _grid(i+1, j,   k);
        values[2] = _grid(i+1, j+1, k);
        values[3] = _grid(i,   j+1, k);
        values[4] = _grid(i,   j,   k+1);
        values[5] = _grid(i+1, j,   k+1);
        values[6] = _grid(i+1, j+1, k+1);
        values[7] = _grid(i,   j+1, k+1);
        
        // Classify cube based on which corners are inside (negative)
        int cube_index = 0;
        for (int v = 0; v < 8; ++v) {
            if (values[v] < 0) cube_index |= (1 << v);
        }
        
        // Skip if all inside or all outside
        if (cube_index == 0 || cube_index == 255) return;
        
        // Get corner positions
        Vec3r corners[8];
        corners[0] = _origin + Vec3r(i,   j,   k).cwiseProduct(_cell_size);
        corners[1] = _origin + Vec3r(i+1, j,   k).cwiseProduct(_cell_size);
        corners[2] = _origin + Vec3r(i+1, j+1, k).cwiseProduct(_cell_size);
        corners[3] = _origin + Vec3r(i,   j+1, k).cwiseProduct(_cell_size);
        corners[4] = _origin + Vec3r(i,   j,   k+1).cwiseProduct(_cell_size);
        corners[5] = _origin + Vec3r(i+1, j,   k+1).cwiseProduct(_cell_size);
        corners[6] = _origin + Vec3r(i+1, j+1, k+1).cwiseProduct(_cell_size);
        corners[7] = _origin + Vec3r(i,   j+1, k+1).cwiseProduct(_cell_size);
        
        // Interpolate vertices on edges where sign changes
        // Using simplified marching cubes: just find crossing edges
        std::vector<Vec3r> edge_vertices;
        
        // Check all 12 edges
        int edges[12][2] = {
            {0,1}, {1,2}, {2,3}, {3,0},  // bottom face
            {4,5}, {5,6}, {6,7}, {7,4},  // top face
            {0,4}, {1,5}, {2,6}, {3,7}   // vertical edges
        };
        
        for (int e = 0; e < 12; ++e) {
            int v0 = edges[e][0];
            int v1 = edges[e][1];
            
            // Check if edge crosses zero level
            if ((values[v0] < 0 && values[v1] >= 0) || 
                (values[v0] >= 0 && values[v1] < 0))
            {
                // Linear interpolation
                Real t = values[v0] / (values[v0] - values[v1]);
                Vec3r vertex = corners[v0] + t * (corners[v1] - corners[v0]);
                edge_vertices.push_back(vertex);
            }
        }
        
        // Create triangles (simple fan triangulation)
        if (edge_vertices.size() >= 3) {
            Vec3r centroid = Vec3r::Zero();
            for (const auto& v : edge_vertices) centroid += v;
            centroid /= edge_vertices.size();
            
            int centroid_idx = vertices.size();
            vertices.push_back(centroid);
            
            for (size_t v = 0; v < edge_vertices.size(); ++v) {
                vertices.push_back(edge_vertices[v]);
                
                int v0 = centroid_idx;
                int v1 = centroid_idx + 1 + v;
                int v2 = centroid_idx + 1 + ((v + 1) % edge_vertices.size());
                
                faces.push_back(Eigen::Vector3i(v0, v1, v2));
            }
        }
    }
};

int main(int argc, char* argv[])
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input_mesh.obj> <output_surface.obj>" << std::endl;
        return 1;
    }

    std::string input_file = argv[1];
    std::string output_file = argv[2];

    std::cout << "=== SDF SURFACE EXTRACTION ===\n" << std::endl;
    std::cout << "Input mesh: " << input_file << std::endl;
    std::cout << "Output surface: " << output_file << std::endl;

    // Load mesh
    std::cout << "\nLoading mesh..." << std::endl;
    Geometry::Mesh mesh = MeshUtils::loadSurfaceMeshFromFile(input_file);
    
    std::cout << "  Vertices: " << mesh.numVertices() << std::endl;
    std::cout << "  Faces: " << mesh.numFaces() << std::endl;

    // Generate SDF
    const int grid_resolution = 128;
    const int padding = 5;
    std::cout << "\nGenerating SDF (" << grid_resolution << "^3 grid)..." << std::endl;
    
    mesh2sdf::MeshSDF sdf(mesh.vertices(), mesh.faces(), grid_resolution, padding, true);
    
    auto grid_bbox = sdf.gridBoundingBox();
    Vec3r grid_origin = grid_bbox.first;
    Vec3r cell_size = sdf.gridCellSize();
    
    std::cout << "  Grid origin: " << grid_origin.transpose() << std::endl;
    std::cout << "  Cell size: " << cell_size.transpose() << std::endl;

    // Extract surface
    std::cout << "\nExtracting zero-level surface..." << std::endl;
    
    MarchingCubes mc(sdf.distanceGrid(), grid_origin, cell_size);
    
    std::vector<Vec3r> surface_vertices;
    std::vector<Eigen::Vector3i> surface_faces;
    
    mc.extractSurface(surface_vertices, surface_faces);

    // Save as OBJ
    std::cout << "\nSaving to " << output_file << "..." << std::endl;
    
    std::ofstream out(output_file);
    if (!out) {
        std::cerr << "Error: Cannot open output file!" << std::endl;
        return 1;
    }
    
    out << "# Surface extracted from SDF\n";
    out << "# Original mesh: " << input_file << "\n";
    out << "# Vertices: " << surface_vertices.size() << "\n";
    out << "# Faces: " << surface_faces.size() << "\n\n";
    
    for (const auto& v : surface_vertices) {
        out << "v " << v[0] << " " << v[1] << " " << v[2] << "\n";
    }
    
    for (const auto& f : surface_faces) {
        // OBJ uses 1-based indexing
        out << "f " << (f[0]+1) << " " << (f[1]+1) << " " << (f[2]+1) << "\n";
    }
    
    out.close();

    std::cout << "\n=== DONE ===" << std::endl;
    std::cout << "Original mesh: " << mesh.numVertices() << " vertices, " 
              << mesh.numFaces() << " faces" << std::endl;
    std::cout << "Extracted surface: " << surface_vertices.size() << " vertices, " 
              << surface_faces.size() << " faces" << std::endl;
    std::cout << "\nCompare the meshes visually to verify SDF quality." << std::endl;

    return 0;
}
