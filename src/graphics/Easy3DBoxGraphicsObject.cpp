#include "graphics/Easy3DBoxGraphicsObject.hpp"

#include <easy3d/algo/surface_mesh_factory.h>
#include <easy3d/renderer/renderer.h>

namespace Graphics
{

Easy3DBoxGraphicsObject::Easy3DBoxGraphicsObject(const std::string& name, const Sim::RigidBox* box)
    : BoxGraphicsObject(name, box)
{
    _e3d_mesh = easy3d::SurfaceMeshFactory::hexahedron();
    for (auto& p : _e3d_mesh.points())
    {
        p[0] *= box->size()[0];
        p[1] *= box->size()[1];
        p[2] *= box->size()[2];
    }

    _initial_points = _e3d_mesh.points();

    _transformPoints();

    _e3d_mesh.set_renderer(new easy3d::Renderer(&_e3d_mesh, true));
    set_renderer(_e3d_mesh.renderer());
}

void Easy3DBoxGraphicsObject::update() 
{
    _transformPoints();    
    renderer()->update();
}

void Easy3DBoxGraphicsObject::_transformPoints()
{
    std::vector<easy3d::vec3>& mesh_points = _e3d_mesh.points();
    const easy3d::vec3 e3d_position(_box->position()[0], _box->position()[1], _box->position()[2]);

    const Eigen::Vector4d& quat = _box->orientation();
    const easy3d::Quat e3d_quat(static_cast<float>(quat[0]), static_cast<float>(quat[1]), static_cast<float>(quat[2]), static_cast<float>(quat[3]));
    const easy3d::mat3 e3d_rot_mat = easy3d::mat3::rotation(e3d_quat);
    for (size_t i = 0; i < mesh_points.size(); i++)
    {
        mesh_points[i] = e3d_rot_mat * _initial_points[i] + e3d_position;
    }
}


} // namespace Graphics