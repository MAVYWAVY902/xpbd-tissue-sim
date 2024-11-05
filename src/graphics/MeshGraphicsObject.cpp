#include "graphics/MeshGraphicsObject.hpp"

namespace Graphics {

MeshGraphicsObject::MeshGraphicsObject(const std::string& name, std::shared_ptr<MeshObject> mesh_object)
    : GraphicsObject(name), _mesh_object(mesh_object)
{
    _draw_faces = true;
    _draw_points = false;
    _draw_edges = true;
}

MeshGraphicsObject::MeshGraphicsObject(const std::string& name, std::shared_ptr<MeshObject> mesh_object, MeshObjectConfig* mesh_object_config)
    : GraphicsObject(name), _mesh_object(mesh_object)
{
    _draw_faces = mesh_object_config->drawFaces().value();
    _draw_edges = mesh_object_config->drawEdges().value();
    _draw_points = mesh_object_config->drawPoints().value();

    _color = mesh_object_config->color().value();
}

MeshGraphicsObject::~MeshGraphicsObject()
{

}

} // namespace Graphics