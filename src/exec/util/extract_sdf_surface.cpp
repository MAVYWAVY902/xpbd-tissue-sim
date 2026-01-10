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
#include <set>
#include <queue>
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
        
        // IMPORTANT: Skip padding regions to avoid boundary artifacts
        // Only process the core region (exclude 2 voxels from each boundary)
        const int margin = 2;  // Skip outer layers to avoid grid boundary artifacts
        
        std::cout << "Processing core region: [" << margin << ":" << (_ni-1-margin) 
                  << "] x [" << margin << ":" << (_nj-1-margin) 
                  << "] x [" << margin << ":" << (_nk-1-margin) << "]" << std::endl;
        
        // Process each cell (avoiding grid boundaries)
        for (int i = margin; i < _ni-1-margin; ++i) {
            for (int j = margin; j < _nj-1-margin; ++j) {
                for (int k = margin; k < _nk-1-margin; ++k) {
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
        
        // Edge table: which edges intersect for each cube configuration
        // Using proper marching cubes with edge sharing
        
        // Check all 12 edges and create/reuse vertices on crossings
        int edges[12][2] = {
            {0,1}, {1,2}, {2,3}, {3,0},  // bottom face (k)
            {4,5}, {5,6}, {6,7}, {7,4},  // top face (k+1)
            {0,4}, {1,5}, {2,6}, {3,7}   // vertical edges
        };
        
        std::vector<int> edge_vertex_indices;
        
        for (int e = 0; e < 12; ++e) {
            int v0_local = edges[e][0];
            int v1_local = edges[e][1];
            
            // Check if edge crosses zero level
            if ((values[v0_local] < 0 && values[v1_local] >= 0) || 
                (values[v0_local] >= 0 && values[v1_local] < 0))
            {
                // Compute global edge ID for sharing across cells
                int gi0, gj0, gk0, gi1, gj1, gk1;
                
                // Map local corner index to grid position
                int di0 = v0_local & 1;
                int dj0 = (v0_local >> 1) & 1;
                int dk0 = (v0_local >> 2) & 1;
                int di1 = v1_local & 1;
                int dj1 = (v1_local >> 1) & 1;
                int dk1 = (v1_local >> 2) & 1;
                
                gi0 = i + di0; gj0 = j + dj0; gk0 = k + dk0;
                gi1 = i + di1; gj1 = j + dj1; gk1 = k + dk1;
                
                // Create unique edge key (sorted to ensure consistency)
                uint64_t edge_key;
                if (gi0 < gi1 || (gi0 == gi1 && gj0 < gj1) || 
                    (gi0 == gi1 && gj0 == gj1 && gk0 < gk1)) {
                    edge_key = ((uint64_t)gi0 << 40) | ((uint64_t)gj0 << 20) | gk0 |
                               (((uint64_t)gi1 << 40) | ((uint64_t)gj1 << 20) | gk1) << 32;
                } else {
                    edge_key = ((uint64_t)gi1 << 40) | ((uint64_t)gj1 << 20) | gk1 |
                               (((uint64_t)gi0 << 40) | ((uint64_t)gj0 << 20) | gk0) << 32;
                }
                
                // Check if vertex already exists for this edge
                auto it = edge_to_vertex.find(edge_key);
                int vert_idx;
                
                if (it != edge_to_vertex.end()) {
                    // Reuse existing vertex
                    vert_idx = it->second;
                } else {
                    // Create new vertex with linear interpolation
                    Real t = values[v0_local] / (values[v0_local] - values[v1_local]);
                    Vec3r vertex = corners[v0_local] + t * (corners[v1_local] - corners[v0_local]);
                    
                    vert_idx = vertices.size();
                    vertices.push_back(vertex);
                    edge_to_vertex[edge_key] = vert_idx;
                }
                
                edge_vertex_indices.push_back(vert_idx);
            }
        }
        
        // Create triangles using fan triangulation
        if (edge_vertex_indices.size() >= 3) {
            // Use first vertex as pivot for fan
            int v0 = edge_vertex_indices[0];
            for (size_t i = 1; i + 1 < edge_vertex_indices.size(); ++i) {
                int v1 = edge_vertex_indices[i];
                int v2 = edge_vertex_indices[i + 1];
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

    std::cout << "\nFiltering artifacts (keeping only largest component)..." << std::endl;
    
    // Build adjacency: face -> neighboring faces
    std::unordered_map<int, std::vector<int>> face_neighbors;
    std::unordered_map<int, std::vector<int>> vertex_faces;
    
    for (int f = 0; f < surface_faces.size(); f++) {
        for (int i = 0; i < 3; i++) {
            vertex_faces[surface_faces[f][i]].push_back(f);
        }
    }
    
    for (int f = 0; f < surface_faces.size(); f++) {
        std::set<int> neighbors;
        for (int i = 0; i < 3; i++) {
            int v = surface_faces[f][i];
            for (int nf : vertex_faces[v]) {
                if (nf != f) neighbors.insert(nf);
            }
        }
        face_neighbors[f] = std::vector<int>(neighbors.begin(), neighbors.end());
    }
    
    // Find connected components via BFS
    std::vector<bool> visited(surface_faces.size(), false);
    std::vector<std::vector<int>> components;
    
    for (int start_face = 0; start_face < surface_faces.size(); start_face++) {
        if (visited[start_face]) continue;
        
        std::vector<int> component;
        std::queue<int> queue;
        queue.push(start_face);
        visited[start_face] = true;
        
        while (!queue.empty()) {
            int f = queue.front();
            queue.pop();
            component.push_back(f);
            
            for (int neighbor : face_neighbors[f]) {
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    queue.push(neighbor);
                }
            }
        }
        
        components.push_back(component);
    }
    
    // Find largest component
    int largest_idx = 0;
    for (int i = 1; i < components.size(); i++) {
        if (components[i].size() > components[largest_idx].size()) {
            largest_idx = i;
        }
    }
    
    std::cout << "  Found " << components.size() << " components" << std::endl;
    std::cout << "  Largest component: " << components[largest_idx].size() 
              << " faces (" << (100.0 * components[largest_idx].size() / surface_faces.size()) 
              << "%)" << std::endl;
    
    if (components.size() > 1) {
        int removed_faces = surface_faces.size() - components[largest_idx].size();
        std::cout << "  Filtering out " << (components.size() - 1) 
                  << " artifact components (" << removed_faces << " faces)" << std::endl;
    }
    
    // Keep only largest component
    std::set<int> used_vertices;
    std::vector<Eigen::Vector3i> filtered_faces;
    
    for (int face_idx : components[largest_idx]) {
        filtered_faces.push_back(surface_faces[face_idx]);
        for (int i = 0; i < 3; i++) {
            used_vertices.insert(surface_faces[face_idx][i]);
        }
    }
    
    // Remap vertices (remove unused)
    std::map<int, int> old_to_new;
    std::vector<Vec3r> filtered_vertices;
    
    for (int old_idx : used_vertices) {
        old_to_new[old_idx] = filtered_vertices.size();
        filtered_vertices.push_back(surface_vertices[old_idx]);
    }
    
    // Update face indices
    for (auto& face : filtered_faces) {
        face[0] = old_to_new[face[0]];
        face[1] = old_to_new[face[1]];
        face[2] = old_to_new[face[2]];
    }
    
    surface_vertices = filtered_vertices;
    surface_faces = filtered_faces;

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
