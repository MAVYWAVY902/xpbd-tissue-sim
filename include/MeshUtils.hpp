#ifndef __MESH_UTILS_HPP
#define __MESH_UTILS_HPP

#include <gmsh.h>

#include <filesystem>

#include <Eigen/Dense>

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/Exporter.hpp>
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

namespace MeshUtils
{

void loadSurfaceMeshFromFile(const std::string& filename, Eigen::Matrix<double, -1, 3>& verts, Eigen::Matrix<unsigned, -1, 3>& faces);

void convertToSTL(const std::string& filename);


void convertSTLtoMSH(const std::string& filename);


void loadMeshDataFromGmshFile(const std::string& filename, Eigen::Matrix<double, -1, 3>& verts, Eigen::Matrix<unsigned, -1, 4>& elems); 

}

#endif // __MESH_UTILS_HPP