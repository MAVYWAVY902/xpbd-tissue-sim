#include "graphics/easy3d/Easy3DSphereGraphicsObject.hpp"

#include <easy3d/algo/surface_mesh_factory.h>
#include <easy3d/renderer/renderer.h>

namespace Graphics
{

Easy3DSphereGraphicsObject::Easy3DSphereGraphicsObject(const std::string& name, const Sim::RigidSphere* sphere)
    : SphereGraphicsObject(name, sphere)
{
    _e3d_mesh = easy3d::SurfaceMeshFactory::quad_sphere(3);
    // Store unit sphere points
    _initial_points = _e3d_mesh.points();

    _transformPoints();
    
    // We must pass false for "own_mesh" logic if we want to manage it ourselves?
    // Actually Renderer(SurfaceMesh* mesh, bool is_surface_mesh) - if true, it treats it as surface mesh property extraction
    // It keeps a pointer. The pointer must stay valid.
    std::shared_ptr<easy3d::Renderer> renderer = std::make_shared<easy3d::Renderer>(&_e3d_mesh, true);
    _e3d_mesh.set_renderer(renderer);
    set_renderer(renderer);
}

void Easy3DSphereGraphicsObject::update() 
{
    // No need to recreate mesh if radius changes, just update points in _transformPoints
    _transformPoints();
    renderer()->update();

    _last_radius = _sphere->radius();
}

void Easy3DSphereGraphicsObject::_transformPoints()
{
    std::vector<easy3d::vec3>& mesh_points = _e3d_mesh.points();
    const easy3d::vec3 e3d_position(_sphere->position()[0], _sphere->position()[1], _sphere->position()[2]);
    const float r = static_cast<float>(_sphere->radius());
    
    for (size_t i = 0; i < mesh_points.size(); i++)
    {
        mesh_points[i] = _initial_points[i] * r + e3d_position;
    }
}

} // namespace Graphics