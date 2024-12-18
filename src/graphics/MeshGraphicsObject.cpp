#include "graphics/MeshGraphicsObject.hpp"

namespace Graphics {

MeshGraphicsObject::MeshGraphicsObject(const std::string& name, const Geometry::Mesh* mesh)
    : GraphicsObject(name), _mesh(mesh)
{
    _draw_faces = true;
    _draw_points = false;
    _draw_edges = true;
}

MeshGraphicsObject::MeshGraphicsObject(const std::string& name, const Geometry::Mesh* mesh, const MeshObjectConfig* mesh_object_config)
    : GraphicsObject(name), _mesh(mesh)
{
    _draw_faces = mesh_object_config->drawFaces();
    _draw_edges = mesh_object_config->drawEdges();
    _draw_points = mesh_object_config->drawPoints();

    _color = mesh_object_config->color();
}

MeshGraphicsObject::~MeshGraphicsObject()
{

}

} // namespace Graphics