#include "graphics/Easy3DVirtuosoArmGraphicsObject.hpp"

#include <easy3d/algo/surface_mesh_factory.h>
#include <easy3d/renderer/renderer.h>
#include <easy3d/renderer/drawable_triangles.h>

namespace Graphics
{

Easy3DVirtuosoArmGraphicsObject::Easy3DVirtuosoArmGraphicsObject(const std::string& name, const Sim::VirtuosoArm* virtuoso_arm)
    : VirtuosoArmGraphicsObject(name, virtuoso_arm)
{
    _generateInitialMesh();

    std::shared_ptr<easy3d::Renderer> renderer = std::make_shared<easy3d::Renderer>(&_e3d_mesh, true);
    _e3d_mesh.set_renderer(renderer);
    set_renderer(renderer);

    // color the mesh gray
    for (auto& tri_drawable : renderer->triangles_drawables())
    {
        easy3d::vec4 color(0.7, 0.7, 0.7, 1.0);
        tri_drawable->set_uniform_coloring(color);
    }
}

void Easy3DVirtuosoArmGraphicsObject::update()
{
    _updateMesh();
    renderer()->update();
}

void Easy3DVirtuosoArmGraphicsObject::_generateInitialMesh()
{
    // generate the initial mesh by combining the torus (outer tube) and the cylinder (inner tube) meshes
    easy3d::SurfaceMesh torus_mesh = _generateTorusMesh(_virtuoso_arm->outerTubeCurvature(), _virtuoso_arm->outerTubeDiameter(), 0, _OT_RADIAL_RES, _OT_TUBULAR_RES);
    easy3d::SurfaceMesh cyl_mesh = easy3d::SurfaceMeshFactory::cylinder(_IT_TUBULAR_RES, _virtuoso_arm->innerTubeDiameter()/2.0, _virtuoso_arm->innerTubeTranslation());
    _e3d_mesh = torus_mesh.join(cyl_mesh);
}

void Easy3DVirtuosoArmGraphicsObject::_updateMesh()
{
    _updateOuterTubeMesh();
    _updateInnerTubeMesh();

    // right now, the center of the base of the outer tube is located at (0,0,0)
    // rotate the whole mesh around (0,0,0) according to the outer tube rotation and then translate the mesh to the appropriate location
    const double ot_rot = _virtuoso_arm->outerTubeRotation();
    const Eigen::Vector3d& ot_pos = _virtuoso_arm->outerTubePosition();
    for (auto& p : _e3d_mesh.points())
    {
        // rotate around Y-axis
        const double x = p[0];
        const double z = p[2];
        p[0] = std::cos(ot_rot)*x + std::sin(ot_rot)*z;
        p[2] = -std::sin(ot_rot)*x + std::cos(ot_rot)*z;

        // translate mesh to outer tube position
        p[0] += ot_pos[0];
        p[1] += ot_pos[1];
        p[2] += ot_pos[2];
    }
}

void Easy3DVirtuosoArmGraphicsObject::_updateInnerTubeMesh()
{
    int ind_offset = _OT_RADIAL_RES*_OT_TUBULAR_RES + 2; // number of vertices in outer tube mesh

    const double max_angle = _virtuoso_arm->outerTubeTranslation() / _virtuoso_arm->outerTubeCurvature();
    // starting point for the inner tube mesh
    const double offset_x = _virtuoso_arm->outerTubeCurvature()*std::cos(max_angle) - _virtuoso_arm->outerTubeCurvature();
    const double offset_y = _virtuoso_arm->outerTubeCurvature()*std::sin(max_angle);
    const double it_rot = _virtuoso_arm->innerTubeRotation();

    easy3d::SurfaceMesh cyl = easy3d::SurfaceMeshFactory::cylinder(_IT_TUBULAR_RES, _virtuoso_arm->innerTubeDiameter()/2.0, _virtuoso_arm->innerTubeTranslation());
    for (int i = ind_offset; i < _e3d_mesh.points().size(); i++)
    {
        auto& p = _e3d_mesh.points()[i];
        p = cyl.points()[i-ind_offset];

        // rotate about the Z-axis by the inner tube rotation
        // rotate about Z-axis by max_angle to align the tube's angle with the end of the outer tube
        const double x1 = p[0];
        const double y1 = p[1];
        p[0] = std::cos(it_rot)*x1 - std::sin(it_rot)*y1;
        p[1] = std::sin(it_rot)*x1 + std::cos(it_rot)*y1;

        // rotate about X-axis 90 degrees
        const double p1 = p[1];
        p[1] = p[2];
        p[2] = -p1;

        // rotate about Z-axis by max_angle to align the tube's angle with the end of the outer tube
        const double x2 = p[0];
        const double y2 = p[1];
        p[0] = std::cos(max_angle)*x2 - std::sin(max_angle)*y2;
        p[1] = std::sin(max_angle)*x2 + std::cos(max_angle)*y2;

        // translate the inner tube the end of the outer tube
        p[0] += offset_x;
        p[1] += offset_y;
    }

}

void Easy3DVirtuosoArmGraphicsObject::_updateOuterTubeMesh()
{
    const double radius = _virtuoso_arm->outerTubeCurvature();
    const double thickness = _virtuoso_arm->outerTubeDiameter();
    const double max_angle = _virtuoso_arm->outerTubeTranslation() / _virtuoso_arm->outerTubeCurvature();
    // get discrete points along center curve - [0, max_angle]
    // center curve will be in the XY plane (arbitrarily)
    std::vector<easy3d::vec3> center_pts;
    double center_angle = 0;
    for (int i = 0; i < _OT_RADIAL_RES; i++)
    {
        const easy3d::vec3 center_pt(radius*std::cos(center_angle) - radius, radius*std::sin(center_angle), 0.0);
        center_pts.push_back(center_pt);

        center_angle += max_angle / (_OT_RADIAL_RES - 1);
    }

    const int num_ot_vertices = center_pts.size()*_OT_TUBULAR_RES + 2;
    int cur_ind = 0;
    // create "tube" (i.e. a circle) around each point on the center curve
    for (unsigned ci = 0; ci < center_pts.size(); ci++)
    {
        const double center_angle = ci * max_angle / (_OT_RADIAL_RES - 1);
        const easy3d::vec3& center_pt = center_pts[ci];
        for (int ti = 0; ti < _OT_TUBULAR_RES; ti++)
        {
            const double tube_angle = ti * (2*3.1415 / _OT_TUBULAR_RES);
            const easy3d::vec3 circle_pt(thickness/2.0 * std::cos(tube_angle), 0.0, thickness/2.0 * std::sin(tube_angle));
            const easy3d::vec3 tube_pt(center_pt[0] + std::cos(center_angle) * circle_pt[0] - std::sin(center_angle) * circle_pt[1],
                                       center_pt[1] + std::cos(center_angle) * circle_pt[1] + std::sin(center_angle) * circle_pt[0],
                                       center_pt[2] + circle_pt[2]);
            _e3d_mesh.points()[cur_ind++] = tube_pt;
        }
    }

    // update middle vertices on each end
    _e3d_mesh.points()[num_ot_vertices-2] = center_pts[0];
    _e3d_mesh.points()[num_ot_vertices-1] = center_pts.back();
}

easy3d::SurfaceMesh Easy3DVirtuosoArmGraphicsObject::_generateTorusMesh(double radius, double thickness, double max_angle, int radial_res, int tubular_res) const
{
    easy3d::SurfaceMesh mesh;

    // get discrete points along center curve - [0, max_angle]
    // center curve will be in the XY plane (arbitrarily)
    std::vector<easy3d::vec3> center_pts;
    double center_angle = 0;
    for (int i = 0; i < radial_res; i++)
    {
        const easy3d::vec3 center_pt(radius*std::cos(center_angle) - radius, radius*std::sin(center_angle), 0.0);
        center_pts.push_back(center_pt);

        center_angle += max_angle / (radial_res - 1);
    }

    const int num_vertices = center_pts.size()*tubular_res + 2;
    const int num_edges = 2*tubular_res + center_pts.size()*tubular_res + (center_pts.size() - 1)*(tubular_res + tubular_res - 1);
    const int num_faces = (center_pts.size() - 1)*tubular_res*2 + 2*tubular_res;
    mesh.reserve(num_vertices, num_edges, num_faces);

    // create "tube" (i.e. a circle) around each point on the center curve
    for (unsigned ci = 0; ci < center_pts.size(); ci++)
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
            mesh.add_vertex(tube_pt);
        }
    }

    // add middle vertices on each end
    mesh.add_vertex(center_pts[0]);
    mesh.add_vertex(center_pts.back());

    // create faces

    // tube faces
    for (unsigned ci = 0; ci < center_pts.size()-1; ci++)
    {
        for (int ti = 0; ti < tubular_res; ti++)
        {
            const int v1 = ci*tubular_res + ti;
            const int v2 = (ti != tubular_res-1) ? v1 + 1 : ci*tubular_res;
            const int v3 = v2 + tubular_res;
            const int v4 = v1 + tubular_res;
            mesh.add_quad(easy3d::SurfaceMesh::Vertex(v1), easy3d::SurfaceMesh::Vertex(v4), easy3d::SurfaceMesh::Vertex(v3), easy3d::SurfaceMesh::Vertex(v2));
        }
    }

    // end cap faces
    for (int ti = 0; ti < tubular_res; ti++)
    {
        const int v1 = num_vertices - 2;
        const int v2 = (ti != tubular_res-1) ? ti + 1 : 0;
        const int v3 = ti;
        mesh.add_triangle(easy3d::SurfaceMesh::Vertex(v1), easy3d::SurfaceMesh::Vertex(v3), easy3d::SurfaceMesh::Vertex(v2));
    }

    for (int ti = 0; ti < tubular_res; ti++)
    {
        const int v1 = num_vertices - 1;
        const int v2 = (center_pts.size()-1)*tubular_res + ti;
        const int v3 = (ti != tubular_res-1) ? v2 + 1 : (center_pts.size()-1)*tubular_res;
        mesh.add_triangle(easy3d::SurfaceMesh::Vertex(v1), easy3d::SurfaceMesh::Vertex(v3), easy3d::SurfaceMesh::Vertex(v2));
    }


    return mesh;

}

} // namespace Graphics