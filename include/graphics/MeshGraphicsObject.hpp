#ifndef __MESH_GRAPHICS_OBJECT_HPP
#define __MESH_GRAPHICS_OBJECT_HPP

#include "graphics/GraphicsObject.hpp"
#include "simobject/MeshObject.hpp"

namespace Graphics
{

class MeshGraphicsObject : public GraphicsObject
{
    public:
    explicit MeshGraphicsObject(const std::string& name);
    explicit MeshGraphicsObject(const std::string& name, std::shared_ptr<MeshObject> mesh_object);

    virtual ~MeshGraphicsObject();

    MeshObject* meshObject() { return _mesh_object.get(); }

    protected:
    std::shared_ptr<MeshObject> _mesh_object;

    bool _draw_points;
    bool _draw_edges;
    bool _draw_faces;
};

} // namespace Graphics


#endif // __MESH_GRAPHICS_OBJECT_HPP