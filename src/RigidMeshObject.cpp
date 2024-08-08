#include "RigidMeshObject.hpp"
#include "MeshUtils.hpp"

RigidMeshObject::RigidMeshObject(const std::string& name)
    : MeshObject(name)
{

}

RigidMeshObject::RigidMeshObject(const std::string& name, const YAML::Node& config)
    : MeshObject(name)
{
    // read in filename and load from file if specified
    YAML::Node filename_yaml_node = config["filename"];
    if (filename_yaml_node.Type() != YAML::NodeType::Null)
    {
        _loadMeshFromFile(filename_yaml_node.as<std::string>());
    }
}

RigidMeshObject::RigidMeshObject(const std::string& name, const std::string& filename)
    : MeshObject(name)
{
    // load from the file specified
    _loadMeshFromFile(filename);
}

RigidMeshObject::RigidMeshObject(const std::string& name, const VerticesMat& verts, const FacesMat& faces)
    : MeshObject(name, verts, faces)
{

}

void RigidMeshObject::_loadMeshFromFile(const std::string& filename)
{
    VerticesMat loaded_verts;
    FacesMat loaded_faces;

    // load vertices and faces from file
    MeshUtils::loadSurfaceMeshFromFile(filename, loaded_verts, loaded_faces);

    // set the new vertices and faces
    setVertices(loaded_verts);
    setFaces(loaded_faces);
}

void RigidMeshObject::createPlaneGeometry(const Eigen::Vector3d& center_pos, const double size)
{
    // create 4 vertices in a square
    VerticesMat verts { {-size/2 + center_pos(0),    -size/2 + center_pos(1),    0 + center_pos(2)},
                        {size/2 + center_pos(0),     -size/2 + center_pos(1),    0 + center_pos(2)},
                        {size/2 + center_pos(0),     size/2 + center_pos(1),     0 + center_pos(2)},
                        {-size/2 + center_pos(0),    size/2 + center_pos(1),     0 + center_pos(2)} };

    // create 2 triangular faces spanning the square
    FacesMat faces { {0, 1, 2},
                     {0, 2, 3} };

    // set the new vertices and faces
    setVertices(verts);
    setFaces(faces);
}

void RigidMeshObject::update(const double dt)
{
    // do nothing for now
}
