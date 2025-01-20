#include "graphics/Easy3DVirtuosoArmGraphicsObject.hpp"

#include <easy3d/algo/surface_mesh_factory.h>
#include <easy3d/renderer/renderer.h>

namespace Graphics
{

Easy3DVirtuosoArmGraphicsObject::Easy3DVirtuosoArmGraphicsObject(const std::string& name, const Sim::VirtuosoArm* virtuoso_arm)
    : VirtuosoArmGraphicsObject(name, virtuoso_arm)
{
    // _e3d_mesh = easy3d::SurfaceMeshFactory::torus();
    _generateTorusMesh(1, 0.25, 5, 40, 20);

    std::shared_ptr<easy3d::Renderer> renderer = std::make_shared<easy3d::Renderer>(&_e3d_mesh, true);
    _e3d_mesh.set_renderer(renderer);
    set_renderer(renderer);
}

void Easy3DVirtuosoArmGraphicsObject::update()
{
    renderer()->update();
}

void Easy3DVirtuosoArmGraphicsObject::_generateTorusMesh(double radius, double thickness, double max_angle, int radial_res, int tubular_res)
{
    _e3d_mesh.clear();

    // get discrete points along center curve - [0, max_angle]
    // center curve will be in the XY plane (arbitrarily)
    std::vector<easy3d::vec3> center_pts;
    double center_angle = 0;
    for (int i = 0; i < radial_res; i++)
    {
        const easy3d::vec3 center_pt(radius*std::cos(center_angle), radius*std::sin(center_angle), 0.0);
        center_pts.push_back(center_pt);

        center_angle += max_angle / (radial_res - 1);
    }

    const int num_vertices = center_pts.size()*tubular_res + 2;
    const int num_edges = 2*tubular_res + center_pts.size()*tubular_res + (center_pts.size() - 1)*(tubular_res + tubular_res - 1);
    const int num_faces = (center_pts.size() - 1)*tubular_res*2 + 2*tubular_res;
    _e3d_mesh.reserve(num_vertices, num_edges, num_faces);

    // create "tube" (i.e. a circle) around each point on the center curve
    for (int ci = 0; ci < center_pts.size(); ci++)
    {
        const double center_angle = ci * max_angle / (radial_res - 1);
        const easy3d::vec3& center_pt = center_pts[ci];
        for (int ti = 0; ti < tubular_res; ti++)
        {
            const double tube_angle = ti * (2*3.1415 / tubular_res);
            const easy3d::vec3 circle_pt(thickness/2.0 * std::cos(tube_angle), 0.0, thickness/2.0 * std::sin(tube_angle));
            const easy3d::vec3 tube_pt(center_pt[0] + std::cos(center_angle) * circle_pt[0] - std::sin(center_angle) * circle_pt[1],
                                       center_pt[1] + std::cos(center_angle) * circle_pt[1] + std::sin(center_angle) * circle_pt[0],
                                       center_pt[2] + circle_pt[2]);
            _e3d_mesh.add_vertex(tube_pt);
        }
    }

    // add middle vertices on each end
    _e3d_mesh.add_vertex(center_pts[0]);
    _e3d_mesh.add_vertex(center_pts.back());

    // create faces

    // tube faces
    for (int ci = 0; ci < center_pts.size()-1; ci++)
    {
        for (int ti = 0; ti < tubular_res; ti++)
        {
            const int v1 = ci*tubular_res + ti;
            const int v2 = (ti != tubular_res-1) ? v1 + 1 : ci*tubular_res;
            const int v3 = v2 + tubular_res;
            const int v4 = v1 + tubular_res;
            _e3d_mesh.add_quad(easy3d::SurfaceMesh::Vertex(v1), easy3d::SurfaceMesh::Vertex(v4), easy3d::SurfaceMesh::Vertex(v3), easy3d::SurfaceMesh::Vertex(v2));
        }
    }

    // end cap faces
    for (int ti = 0; ti < tubular_res; ti++)
    {
        const int v1 = num_vertices - 2;
        const int v2 = (ti != tubular_res-1) ? ti + 1 : 0;
        const int v3 = ti;
        _e3d_mesh.add_triangle(easy3d::SurfaceMesh::Vertex(v1), easy3d::SurfaceMesh::Vertex(v3), easy3d::SurfaceMesh::Vertex(v2));
    }

    for (int ti = 0; ti < tubular_res; ti++)
    {
        const int v1 = num_vertices - 1;
        const int v2 = (center_pts.size()-1)*tubular_res + ti;
        const int v3 = (ti != tubular_res-1) ? v2 + 1 : (center_pts.size()-1)*tubular_res;
        _e3d_mesh.add_triangle(easy3d::SurfaceMesh::Vertex(v1), easy3d::SurfaceMesh::Vertex(v3), easy3d::SurfaceMesh::Vertex(v2));
    }




}

} // namespace Graphics