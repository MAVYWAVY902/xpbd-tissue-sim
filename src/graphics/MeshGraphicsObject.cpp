#include "graphics/MeshGraphicsObject.hpp"

namespace Graphics {

MeshGraphicsObject::MeshGraphicsObject(const std::string& name)
    : GraphicsObject(name)
{

}

MeshGraphicsObject::MeshGraphicsObject(const std::string& name, std::shared_ptr<MeshObject> mesh_object)
    : GraphicsObject(name), _mesh_object(mesh_object)
{
    _draw_faces = true;
    _draw_points = false;
    _draw_edges = true;
}

MeshGraphicsObject::~MeshGraphicsObject()
{

}

} // namespace Graphics