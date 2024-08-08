#include "MeshUtils.hpp"

#include <iostream>

void MeshUtils::loadSurfaceMeshFromFile(const std::string& filename, Eigen::Matrix<double, -1, 3>& verts, Eigen::Matrix<unsigned, -1, 3>& faces)
{
    Assimp::Importer importer;

    // And have it read the given file with some example postprocessing
    // Usually - if speed is not the most important aspect for you - you'll
    // probably to request more postprocessing than we do in this example.
    const aiScene* scene = importer.ReadFile( filename,
        aiProcess_Triangulate            |
        aiProcess_JoinIdenticalVertices  |
        aiProcess_SortByPType);

    // If the import failed, report it
    if (scene == nullptr)
    {
        std::cerr << "\tAssimp::Importer could not open " << filename << std::endl;
        std::cerr << "\tEnsure that the file is in a format that assimp can handle." << std::endl;
        return;
    }

    const aiMesh* mesh = scene->mMeshes[0];

    // Extract vertices
    verts.conservativeResize(mesh->mNumVertices, 3);
    for (unsigned i = 0; i < mesh->mNumVertices; i++)
    {
        verts(i,0) = mesh->mVertices[i].x;
        verts(i,1) = mesh->mVertices[i].y;
        verts(i,2) = mesh->mVertices[i].z;
    }

    // Extract faces
    faces.conservativeResize(mesh->mNumFaces, 3);
    for (unsigned i = 0; i < mesh->mNumFaces; i++)
    {
        faces(i,0) = mesh->mFaces[i].mIndices[0];
        faces(i,1) = mesh->mFaces[i].mIndices[1];
        faces(i,2) = mesh->mFaces[i].mIndices[2];
    }
}

void MeshUtils::convertToSTL(const std::string& filename)
{
    std::cout << "MeshUtils::convertToSTL - converting " << filename << " to .stl format..." << std::endl;
    std::filesystem::path file_path(filename);

    Assimp::Importer importer;

    // And have it read the given file with some example postprocessing
    // Usually - if speed is not the most important aspect for you - you'll
    // probably to request more postprocessing than we do in this example.
    const aiScene* scene = importer.ReadFile( filename,
        aiProcess_Triangulate            |
        aiProcess_JoinIdenticalVertices  |
        aiProcess_SortByPType);

    // If the import failed, report it
    if (scene == nullptr)
    {
        std::cerr << "\tAssimp::Importer could not open " << filename << std::endl;
        std::cerr << "\tEnsure that the file is in a format that assimp can handle." << std::endl;
        return;
    }

    std::cout << "\tAssimp import successful" << std::endl;

    Assimp::Exporter exporter;
    const std::string& stl_filename = file_path.replace_extension(".stl").string();
    exporter.Export(scene, "stl", stl_filename);

    std::cout << "\tAssimp export successful - new file is " << stl_filename << "\n" << std::endl;
}

void MeshUtils::convertSTLtoMSH(const std::string& filename)
{
    std::cout << "MeshUtils::convertSTLtoMSH - converting " << filename << " from .stl to .msh format..." << std::endl;

    // ensure the file exists
    if (!std::filesystem::exists(filename))
    {
        std::cerr << "\t" << filename << " does not exist!" << std::endl;
        return;
    }


    std::filesystem::path file_path(filename);

    // ensure the file is an STL file
    if (file_path.extension() != ".stl" && file_path.extension() != ".STL")
    {
        std::cerr << "\t" << filename << " is not a .stl file!" << std::endl;
        return;
    }

    // open the STL file with GMSH
    gmsh::open(filename);


    int surface_loop_tag = gmsh::model::geo::addSurfaceLoop(std::vector<int>{1});

    int volume_tag = gmsh::model::geo::addVolume(std::vector<int>{surface_loop_tag});

    gmsh::model::geo::synchronize();
    gmsh::model::mesh::generate(3);

    const std::string& msh_filename = file_path.replace_extension(".msh").string();
    gmsh::write(msh_filename);

    std::cout << "\tGMSH conversion to .msh successful - new file is " << msh_filename << "\n" << std::endl;
    
}

void MeshUtils::loadMeshDataFromGmshFile(const std::string& filename, Eigen::Matrix<double, -1, 3>& verts, Eigen::Matrix<unsigned, -1, 4>& elems)
{
    std::cout << "MeshUtils::loadMeshDataFromGmshFile - loading mesh data from " << filename << " as a MeshObject..." << std::endl;

    // ensure the file exists
    if (!std::filesystem::exists(filename))
    {
        std::cerr << "\t" << filename << " does not exist!" << std::endl;
        return;
    }


    std::filesystem::path file_path(filename);

    // ensure the file is a .msh file
    if (file_path.extension() != ".msh" && file_path.extension() != ".MSH")
    {
        std::cerr << "\t" << filename << " is not a .msh file!" << std::endl;
        return;
    }

    gmsh::open(filename);

    // Get all the elementary entities in the model, as a vector of (dimension,
    // tag) pairs:
    std::vector<std::pair<int, int> > entities;
    gmsh::model::getEntities(entities);

    for(auto e : entities) {
        // Dimension and tag of the entity:
        int dim = e.first, tag = e.second;

        // Mesh data is made of `elements' (points, lines, triangles, ...), defined
        // by an ordered list of their `nodes'. Elements and nodes are identified by
        // `tags' as well (strictly positive identification numbers), and are stored
        // ("classified") in the model entity they discretize. Tags for elements and
        // nodes are globally unique (and not only per dimension, like entities).

        // A model entity of dimension 0 (a geometrical point) will contain a mesh
        // element of type point, as well as a mesh node. A model curve will contain
        // line elements as well as its interior nodes, while its boundary nodes
        // will be stored in the bounding model points. A model surface will contain
        // triangular and/or quadrangular elements and all the nodes not classified
        // on its boundary or on its embedded entities. A model volume will contain
        // tetrahedra, hexahedra, etc. and all the nodes not classified on its
        // boundary or on its embedded entities.

        // Get the mesh nodes for the entity (dim, tag):
        std::vector<std::size_t> nodeTags;
        std::vector<double> nodeCoords, nodeParams;
        gmsh::model::mesh::getNodes(nodeTags, nodeCoords, nodeParams, dim, tag);

        unsigned vert_offset = verts.rows();
        verts.conservativeResize(vert_offset + nodeTags.size(), 3);
        for (unsigned i = 0; i < nodeTags.size(); i++)
        {
            verts(vert_offset + i, 0) = nodeCoords[i*3];
            verts(vert_offset + i, 1) = nodeCoords[i*3 + 1];
            verts(vert_offset + i, 2) = nodeCoords[i*3 + 2];
        }

        // Get the mesh elements for the entity (dim, tag):
        std::vector<int> elemTypes;
        std::vector<std::vector<std::size_t> > elemTags, elemNodeTags;
        gmsh::model::mesh::getElements(elemTypes, elemTags, elemNodeTags, dim, tag);

        // Extract all tetrahedra into a flat vector
        std::vector<unsigned> tetrahedra_vertex_indices;
        for (unsigned i = 0; i < elemTypes.size(); i++)
        {
            if (elemTypes[i] == 4)
            {
                tetrahedra_vertex_indices.insert(tetrahedra_vertex_indices.end(), elemNodeTags[i].begin(), elemNodeTags[i].end());
            }
        }

        unsigned elem_offset = elems.rows();
        unsigned num_tetrahedra = tetrahedra_vertex_indices.size()/4;
        elems.conservativeResize(elem_offset + num_tetrahedra, 4);
        for (unsigned i = 0; i < num_tetrahedra; i++)
        {
            elems(elem_offset + i, 0) = tetrahedra_vertex_indices[i*4] - 1;
            elems(elem_offset + i, 1) = tetrahedra_vertex_indices[i*4 + 1] - 1;
            elems(elem_offset + i, 2) = tetrahedra_vertex_indices[i*4 + 2] - 1;
            elems(elem_offset + i, 3) = tetrahedra_vertex_indices[i*4 + 3] - 1;
        }
    }

}