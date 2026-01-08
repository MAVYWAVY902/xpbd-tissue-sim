/**
 * Convert MSH (GMSH tetrahedral mesh) to OBJ (surface mesh)
 * 
 * Extracts the surface triangles from a volumetric tetrahedral mesh
 * 
 * Usage: ./MshToObj <input.msh> <output.obj>
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <set>

#include "geometry/Mesh.hpp"
#include "utils/MeshUtils.hpp"

int main(int argc, char* argv[])
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input.msh> <output.obj>" << std::endl;
        return 1;
    }

    std::string input_file = argv[1];
    std::string output_file = argv[2];

    std::cout << "=== MSH TO OBJ CONVERTER ===\n" << std::endl;
    std::cout << "Input: " << input_file << std::endl;
    std::cout << "Output: " << output_file << std::endl;

    // Load mesh
    std::cout << "\nLoading mesh..." << std::endl;
    Geometry::Mesh mesh = MeshUtils::loadSurfaceMeshFromFile(input_file);
    
    std::cout << "  Vertices: " << mesh.numVertices() << std::endl;
    std::cout << "  Faces: " << mesh.numFaces() << std::endl;
    
    auto bbox = mesh.boundingBox();
    Vec3r size = bbox.max - bbox.min;
    std::cout << "  BBox size: " << size.transpose() << " m" << std::endl;
    std::cout << "  BBox size: " << (size * 1000).transpose() << " mm" << std::endl;

    // Save as OBJ
    std::cout << "\nSaving to " << output_file << "..." << std::endl;
    
    std::ofstream out(output_file);
    if (!out) {
        std::cerr << "Error: Cannot open output file!" << std::endl;
        return 1;
    }
    
    out << "# Converted from " << input_file << "\n";
    out << "# Vertices: " << mesh.numVertices() << "\n";
    out << "# Faces: " << mesh.numFaces() << "\n\n";
    
    // Write vertices
    for (int i = 0; i < mesh.numVertices(); ++i) {
        const Vec3r& v = mesh.vertex(i);
        out << "v " << v[0] << " " << v[1] << " " << v[2] << "\n";
    }
    
    out << "\n";
    
    // Write faces
    const auto& faces = mesh.faces();
    for (int i = 0; i < faces.cols(); ++i) {
        const Eigen::Vector3i& f = faces.col(i);
        // OBJ uses 1-based indexing
        out << "f " << (f[0]+1) << " " << (f[1]+1) << " " << (f[2]+1) << "\n";
    }
    
    out.close();

    std::cout << "\n=== DONE ===" << std::endl;
    std::cout << "Saved " << mesh.numVertices() << " vertices, " 
              << mesh.numFaces() << " faces" << std::endl;

    return 0;
}
