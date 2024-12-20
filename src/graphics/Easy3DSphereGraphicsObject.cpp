#include "graphics/Easy3DSphereGraphicsObject.hpp"

#include <easy3d/algo/surface_mesh_factory.h>
#include <easy3d/renderer/renderer.h>

namespace Graphics
{

Easy3DSphereGraphicsObject::Easy3DSphereGraphicsObject(const std::string& name, const Sim::RigidSphere* sphere)
    : SphereGraphicsObject(name, sphere)
{
    _e3d_mesh = easy3d::SurfaceMeshFactory::quad_sphere(3);
    for (auto& p : _e3d_mesh.points())
    {
        p *= sphere->radius();
    }
    _initial_points = _e3d_mesh.points();

    _transformPoints();
    _e3d_mesh.set_renderer(new easy3d::Renderer(&_e3d_mesh, true));
    set_renderer(_e3d_mesh.renderer());
}

void Easy3DSphereGraphicsObject::update() 
{
    // TODO: transform mesh according to current position
    _transformPoints();
    renderer()->update();
}

void Easy3DSphereGraphicsObject::_transformPoints()
{
    std::vector<easy3d::vec3>& mesh_points = _e3d_mesh.points();
    const easy3d::vec3 e3d_position(_sphere->position()[0], _sphere->position()[1], _sphere->position()[2]);
    for (size_t i = 0; i < mesh_points.size(); i++)
    {
        mesh_points[i] = _initial_points[i] + e3d_position;
    }
}

} // namespace Graphics