#include "graphics/Easy3DVirtuosoArmGraphicsObject.hpp"

#include "geometry/TransformationMatrix.hpp"
#include "geometry/CoordinateFrame.hpp"

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
    easy3d::SurfaceMesh torus_mesh = _generateTorusMesh(_virtuoso_arm->outerTubeRadiusOfCurvature(), _virtuoso_arm->outerTubeDiameter(), 0, _OT_RADIAL_RES, _OT_TUBULAR_RES);
    easy3d::SurfaceMesh ot_cyl_mesh = easy3d::SurfaceMeshFactory::cylinder(_OT_TUBULAR_RES, _virtuoso_arm->outerTubeDiameter()/2.0, _virtuoso_arm->outerTubeDistalStraightLength());
    const double l = std::max(_virtuoso_arm->innerTubeTranslation() - _virtuoso_arm->outerTubeTranslation(), 0.0);
    easy3d::SurfaceMesh cyl_mesh = easy3d::SurfaceMeshFactory::cylinder(_IT_TUBULAR_RES, _virtuoso_arm->innerTubeDiameter()/2.0, l);
    _e3d_mesh = torus_mesh.join(ot_cyl_mesh).join(cyl_mesh);
}

void Easy3DVirtuosoArmGraphicsObject::_updateMesh()
{
    _updateOuterTubeMesh();
    _updateInnerTubeMesh();

    // right now, the center of the base of the outer tube is located at (0,0,0)
    // rotate the whole mesh around (0,0,0) according to the outer tube rotation and then translate the mesh to the appropriate location
    const double ot_rot = _virtuoso_arm->outerTubeRotation();

    Geometry::TransformationMatrix endoscope_transform = _virtuoso_arm->endoscopeFrame().transform();
    Eigen::Matrix3d endoscope_rot_mat = endoscope_transform.rotMat();
    easy3d::Mat3<float> rot_mat_easy3d;
    rot_mat_easy3d(0,0) = endoscope_rot_mat(0,0); rot_mat_easy3d(0,1) = endoscope_rot_mat(0,1); rot_mat_easy3d(0,2) = endoscope_rot_mat(0,2);
    rot_mat_easy3d(1,0) = endoscope_rot_mat(1,0); rot_mat_easy3d(1,1) = endoscope_rot_mat(1,1); rot_mat_easy3d(1,2) = endoscope_rot_mat(1,2);
    rot_mat_easy3d(2,0) = endoscope_rot_mat(2,0); rot_mat_easy3d(2,1) = endoscope_rot_mat(2,1); rot_mat_easy3d(2,2) = endoscope_rot_mat(2,2);

    Eigen::Vector3d endoscope_pos = endoscope_transform.translation();
    for (auto& p : _e3d_mesh.points())
    {
        // rotate around z-axis according to outer tube rotation
        const double x = p[0];
        const double y = p[1];
        p[0] = std::cos(ot_rot)*x - std::sin(ot_rot)*y;
        p[1] = std::sin(ot_rot)*x + std::cos(ot_rot)*y;

        p = rot_mat_easy3d*p;

        // apply endoscope transformation
        p[0] += endoscope_pos[0];
        p[1] += endoscope_pos[1];
        p[2] += endoscope_pos[2];
    }
}

void Easy3DVirtuosoArmGraphicsObject::_updateInnerTubeMesh()
{
    int ind_offset = _OT_RADIAL_RES*_OT_TUBULAR_RES + 2 + _OT_TUBULAR_RES*2; // number of vertices in outer tube mesh

    const double max_angle = std::max(_virtuoso_arm->outerTubeTranslation() - _virtuoso_arm->outerTubeDistalStraightLength(), 0.0) / _virtuoso_arm->outerTubeRadiusOfCurvature();
    // starting point for the inner tube mesh
    double ot_straight_cyl_length = std::min(_virtuoso_arm->outerTubeDistalStraightLength(), _virtuoso_arm->outerTubeTranslation());
    const double offset_x = -_virtuoso_arm->outerTubeRadiusOfCurvature()*std::cos(max_angle) + _virtuoso_arm->outerTubeRadiusOfCurvature() + ot_straight_cyl_length*std::sin(max_angle);
    const double offset_z = _virtuoso_arm->outerTubeRadiusOfCurvature()*std::sin(max_angle) + ot_straight_cyl_length*std::cos(max_angle);
    const double it_rot = _virtuoso_arm->innerTubeRotation();
    const double it_length = std::max(_virtuoso_arm->innerTubeTranslation() - _virtuoso_arm->outerTubeTranslation(), 0.0);

    easy3d::SurfaceMesh cyl = easy3d::SurfaceMeshFactory::cylinder(_IT_TUBULAR_RES, _virtuoso_arm->innerTubeDiameter()/2.0, it_length);
    for (int i = ind_offset; i < ind_offset + cyl.points().size(); i++)
    {
        auto& p = _e3d_mesh.points()[i];
        p = cyl.points()[i-ind_offset];

        // rotate about the Z-axis by the inner tube rotation
        const double x1 = p[0];
        const double y1 = p[1];
        p[0] = std::cos(it_rot)*x1 - std::sin(it_rot)*y1;
        p[1] = std::sin(it_rot)*x1 + std::cos(it_rot)*y1;

        // rotate about y-axis by max_angle to align the tube's angle with the end of the outer tube
        const double x2 = p[0];
        const double z2 = p[2];
        p[0] = std::cos(max_angle)*x2 + std::sin(max_angle)*z2;
        p[2] = -std::sin(max_angle)*x2 + std::cos(max_angle)*z2;

        // translate the inner tube the end of the outer tube
        p[0] += offset_x;
        p[2] += offset_z;
    }

}

void Easy3DVirtuosoArmGraphicsObject::_updateOuterTubeMesh()
{
    const double radius = _virtuoso_arm->outerTubeRadiusOfCurvature();
    const double thickness = _virtuoso_arm->outerTubeDiameter();
    const double max_angle = std::max(_virtuoso_arm->outerTubeTranslation() - _virtuoso_arm->outerTubeDistalStraightLength(), 0.0) / radius;
    // get discrete points along center curve - [0, max_angle]
    // center curve will be in the XZ plane, initial slope in the positive Z-direction, curving towards positive X (arbitrarily)
    std::vector<easy3d::vec3> center_pts;
    double center_angle = 0;
    for (int i = 0; i < _OT_RADIAL_RES; i++)
    {
        const easy3d::vec3 center_pt(-radius*std::cos(center_angle) + radius, 0.0, radius*std::sin(center_angle));
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
            const easy3d::vec3 circle_pt(thickness/2.0 * std::cos(tube_angle), thickness/2.0 * std::sin(tube_angle), 0.0);
            const easy3d::vec3 tube_pt(center_pt[0] + std::cos(center_angle) * circle_pt[0] + std::sin(center_angle) * circle_pt[2],
                                       center_pt[1] + circle_pt[1],
                                       center_pt[2] + std::cos(center_angle) * circle_pt[2] - std::sin(center_angle) * circle_pt[0]
                                       );
            _e3d_mesh.points()[cur_ind++] = tube_pt;
        }
    }

    // update middle vertices on each end
    _e3d_mesh.points()[num_ot_vertices-2] = center_pts[0];
    _e3d_mesh.points()[num_ot_vertices-1] = center_pts.back();

    int ind_offset = _OT_RADIAL_RES*_OT_TUBULAR_RES + 2; // number of vertices in curved outer tube mesh
    double cyl_length = std::min(_virtuoso_arm->outerTubeDistalStraightLength(), _virtuoso_arm->outerTubeTranslation());
    easy3d::SurfaceMesh cyl = easy3d::SurfaceMeshFactory::cylinder(_OT_TUBULAR_RES, _virtuoso_arm->outerTubeDiameter()/2.0, cyl_length);
    for (int i = ind_offset; i < ind_offset + cyl.points().size(); i++)
    {
        auto& p = _e3d_mesh.points()[i];
        p = cyl.points()[i-ind_offset];

        // rotate about y-axis by max_angle to align the tube's angle with the end of the curved part of the outer tube
        const double x2 = p[0];
        const double z2 = p[2];
        p[0] = std::cos(max_angle)*x2 + std::sin(max_angle)*z2;
        p[2] = -std::sin(max_angle)*x2 + std::cos(max_angle)*z2;

        // translate the straight tube the end of the curved part of the outer tube
        p[0] += center_pts.back()[0];
        p[2] += center_pts.back()[2];
    }
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