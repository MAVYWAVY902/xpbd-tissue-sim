#include "RigidMeshObject.hpp"
#include "MeshUtils.hpp"

RigidMeshObject::RigidMeshObject(const std::string& name)
    : MeshObject(name)
{

}

// RigidMeshObject::RigidMeshObject(const std::string& name, const YAML::Node& config)
//     : MeshObject(name)
// {
//     // read in filename and load from file if specified
//     YAML::Node filename_yaml_node = config["filename"];
//     if (filename_yaml_node.Type() != YAML::NodeType::Null)
//     {
//         _loadMeshFromFile(filename_yaml_node.as<std::string>());
//     }
// }
RigidMeshObject::RigidMeshObject(const RigidMeshObjectConfig* config)
    : MeshObject(config)
{
    if (config->filename().has_value())
    {
        _loadMeshFromFile(config->filename().value());
    }
    else if (config->primitiveType().has_value())
    {
        createPrimitiveGeometry(config->primitiveType().value());
    }
    

    if (config->initialPosition().has_value())
    {
        moveTo(config->initialPosition().value(), PositionReference::CENTER);
    }

    if (config->maxSize().has_value())
    {
        resize(config->maxSize().value());
    }

    if (config->size().has_value())
    {
        resize(config->size().value());
    }

    if (config->initialVelocity().has_value())
    {
        _v = config->initialVelocity().value();
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

void RigidMeshObject::createPrimitiveGeometry(const RigidMeshPrimitiveType primitive_type)
{
    if (primitive_type == RigidMeshPrimitiveType::PLANE)
    {
        createPlaneGeometry();
    }
}

void RigidMeshObject::createPlaneGeometry(const Eigen::Vector3d& center_pos, const double size)
{
    // create 4 vertices in a square
    // VerticesMat verts { {-size/2 + center_pos(0),    -size/2 + center_pos(1),    0 + center_pos(2)},
    //                     {size/2 + center_pos(0),     -size/2 + center_pos(1),    0 + center_pos(2)},
    //                     {size/2 + center_pos(0),     size/2 + center_pos(1),     0 + center_pos(2)},
    //                     {-size/2 + center_pos(0),    size/2 + center_pos(1),     0 + center_pos(2)} };

    // // create 2 triangular faces spanning the square
    // FacesMat faces { {0, 1, 2},
    //                  {0, 2, 3} };

    createPlaneGeometry();
    resize(size, size, 0);
    moveTo(center_pos, PositionReference::CENTER);
}

void RigidMeshObject::createPlaneGeometry()
{
    // create 4 vertices in a 1x1 square
    VerticesMat verts { { -1, -1, 0 },
                        { 1, -1, 0 },
                        { 1, 1, 0 },
                        { -1, 1, 0 } };
    
    FacesMat faces { {0, 1, 2},
                     {0, 2, 3} };
    
    // set the new vertices and faces
    setVertices(verts);
    setFaces(faces);
}

void RigidMeshObject::setup()
{
    
}

void RigidMeshObject::update()
{
    // do nothing for now
}
