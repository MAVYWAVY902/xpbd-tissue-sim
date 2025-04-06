#include "graphics/Easy3DVirtuosoArmGraphicsObject.hpp"

#include "geometry/TransformationMatrix.hpp"
#include "geometry/CoordinateFrame.hpp"

#include <easy3d/algo/surface_mesh_factory.h>
#include <easy3d/renderer/renderer.h>
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/renderer/drawable_triangles.h>

namespace Graphics
{

Easy3DVirtuosoArmGraphicsObject::Easy3DVirtuosoArmGraphicsObject(const std::string& name, const Sim::VirtuosoArm* virtuoso_arm)
    : VirtuosoArmGraphicsObject(name, virtuoso_arm)
{
    _generateInitialMesh();

    std::shared_ptr<easy3d::Renderer> renderer = std::make_shared<easy3d::Renderer>(&_e3d_mesh, true);
    _e3d_mesh.set_renderer(renderer);

    // std::shared_ptr<easy3d::Renderer> renderer = std::make_shared<easy3d::Renderer>(this, true);

    // create a PointsDrawable for the points of the tetrahedral mesh
    // easy3d::PointsDrawable* points_drawable = renderer->add_points_drawable("vertices");
    // // specify the update function for the points
    // points_drawable->set_update_func([](easy3d::Model* m, easy3d::Drawable* d) {
    //     // update the vertex buffer with the vertices of the mesh
    //     d->update_vertex_buffer(m->points(), true);
    // });

    // set a uniform color for the mesh
    // easy3d::vec4 color(0, 0, 0, 0);
    // points_drawable->set_uniform_coloring(color);

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
    // easy3d::SurfaceMesh torus_mesh = _generateTorusMesh(_virtuoso_arm->outerTubeRadiusOfCurvature(), _virtuoso_arm->outerTubeDiameter(), 0, _OT_RADIAL_RES, _OT_TUBULAR_RES);
    // easy3d::SurfaceMesh ot_cyl_mesh = easy3d::SurfaceMeshFactory::cylinder(_OT_TUBULAR_RES, _virtuoso_arm->outerTubeDiameter()/2.0, _virtuoso_arm->outerTubeDistalStraightLength());
    // const double l = std::max(_virtuoso_arm->innerTubeTranslation() - _virtuoso_arm->outerTubeTranslation(), 0.0);
    // easy3d::SurfaceMesh cyl_mesh = easy3d::SurfaceMeshFactory::cylinder(_IT_TUBULAR_RES, _virtuoso_arm->innerTubeDiameter()/2.0, l);
    // _e3d_mesh = torus_mesh.join(ot_cyl_mesh).join(cyl_mesh);



    const Sim::VirtuosoArm::OuterTubeFramesArray& ot_frames = _virtuoso_arm->outerTubeFrames();
    const Sim::VirtuosoArm::InnerTubeFramesArray& it_frames = _virtuoso_arm->innerTubeFrames();

    int num_ot_vertices = (ot_frames.size() * _OT_TUBULAR_RES + 2);
    int num_it_vertices = (it_frames.size() * _IT_TUBULAR_RES + 2);

    const double ot_r = _virtuoso_arm->outerTubeDiameter() / 2.0;
    const double it_r = _virtuoso_arm->innerTubeDiameter() / 2.0;
    const double ot_trans = _virtuoso_arm->outerTubeTranslation();
    const double it_trans = _virtuoso_arm->innerTubeTranslation();
    for (unsigned fi = 0; fi < ot_frames.size(); fi++)
    {
        for (int i = 0; i < _OT_TUBULAR_RES; i++)
        {
            double angle = i * 2 * 3.1415 / _OT_TUBULAR_RES;
            _e3d_mesh.add_vertex(easy3d::vec3(ot_r * std::cos(angle), ot_r * std::sin(angle), fi*ot_trans/(ot_frames.size() - 1)));
        }
    }

    _e3d_mesh.add_vertex(easy3d::vec3(0.0, 0.0, 0.0));
    _e3d_mesh.add_vertex(easy3d::vec3(0.0, 0.0, ot_trans));


    for (unsigned fi = 0; fi < it_frames.size(); fi++)
    {
        for (int i = 0; i < _IT_TUBULAR_RES; i++)
        {
            double angle = i* 2 * 3.1415 / _IT_TUBULAR_RES;
            _e3d_mesh.add_vertex(easy3d::vec3(it_r * std::cos(angle), it_r * std::sin(angle), ot_trans + fi*it_trans/(it_frames.size() - 1)));
        }
    }

    _e3d_mesh.add_vertex(easy3d::vec3(0.0, 0.0, ot_trans));
    _e3d_mesh.add_vertex(easy3d::vec3(0.0, 0.0, ot_trans + it_trans));

    // update mesh will do the vertices of the mesh
    // _updateMesh();

    // add face information

    // outer tube faces
    for (unsigned fi = 0; fi < ot_frames.size()-1; fi++)
    {
        for (int ti = 0; ti < _OT_TUBULAR_RES; ti++)
        {
            const int v1 = fi*_OT_TUBULAR_RES + ti;
            const int v2 = (ti != _OT_TUBULAR_RES-1) ? v1 + 1 : fi*_OT_TUBULAR_RES;
            const int v3 = v2 + _OT_TUBULAR_RES;
            const int v4 = v1 + _OT_TUBULAR_RES;
            _e3d_mesh.add_quad(easy3d::SurfaceMesh::Vertex(v1), easy3d::SurfaceMesh::Vertex(v4), easy3d::SurfaceMesh::Vertex(v3), easy3d::SurfaceMesh::Vertex(v2));
        }
    }

    // end cap faces
    for (int ti = 0; ti < _OT_TUBULAR_RES; ti++)
    {
        const int v1 = num_ot_vertices - 2;
        const int v2 = (ti != _OT_TUBULAR_RES-1) ? ti + 1 : 0;
        const int v3 = ti;
        _e3d_mesh.add_triangle(easy3d::SurfaceMesh::Vertex(v1), easy3d::SurfaceMesh::Vertex(v3), easy3d::SurfaceMesh::Vertex(v2));
    }

    for (int ti = 0; ti < _OT_TUBULAR_RES; ti++)
    {
        const int v1 = num_ot_vertices - 1;
        const int v2 = (ot_frames.size()-1)*_OT_TUBULAR_RES + ti;
        const int v3 = (ti != _OT_TUBULAR_RES-1) ? v2 + 1 : (ot_frames.size()-1)*_OT_TUBULAR_RES;
        _e3d_mesh.add_triangle(easy3d::SurfaceMesh::Vertex(v1), easy3d::SurfaceMesh::Vertex(v3), easy3d::SurfaceMesh::Vertex(v2));
    }


    // inner tube faces
    for (unsigned fi = 0; fi < it_frames.size()-1; fi++)
    {
        for (int ti = 0; ti < _IT_TUBULAR_RES; ti++)
        {
            const int v1 = fi*_IT_TUBULAR_RES + ti;
            const int v2 = (ti != _IT_TUBULAR_RES-1) ? v1 + 1 : fi*_IT_TUBULAR_RES;
            const int v3 = v2 + _IT_TUBULAR_RES;
            const int v4 = v1 + _IT_TUBULAR_RES;
            _e3d_mesh.add_quad(easy3d::SurfaceMesh::Vertex(num_ot_vertices+v1), 
                easy3d::SurfaceMesh::Vertex(num_ot_vertices+v4), 
                easy3d::SurfaceMesh::Vertex(num_ot_vertices+v3), 
                easy3d::SurfaceMesh::Vertex(num_ot_vertices+v2));
        }
    }

    // end cap faces
    for (int ti = 0; ti < _IT_TUBULAR_RES; ti++)
    {
        const int v1 = num_it_vertices - 2;
        const int v2 = (ti != _IT_TUBULAR_RES-1) ? ti + 1 : 0;
        const int v3 = ti;
        _e3d_mesh.add_triangle(easy3d::SurfaceMesh::Vertex(num_ot_vertices+v1), easy3d::SurfaceMesh::Vertex(num_ot_vertices+v3), easy3d::SurfaceMesh::Vertex(num_ot_vertices+v2));
    }

    for (int ti = 0; ti < _IT_TUBULAR_RES; ti++)
    {
        const int v1 = num_it_vertices - 1;
        const int v2 = (it_frames.size()-1)*_IT_TUBULAR_RES + ti;
        const int v3 = (ti != _IT_TUBULAR_RES-1) ? v2 + 1 : (it_frames.size()-1)*_IT_TUBULAR_RES;
        _e3d_mesh.add_triangle(easy3d::SurfaceMesh::Vertex(num_ot_vertices+v1), easy3d::SurfaceMesh::Vertex(num_ot_vertices+v3), easy3d::SurfaceMesh::Vertex(num_ot_vertices+v2));
    }
}

void Easy3DVirtuosoArmGraphicsObject::_updateMesh()
{
    const Sim::VirtuosoArm::OuterTubeFramesArray& ot_frames = _virtuoso_arm->outerTubeFrames();
    const Sim::VirtuosoArm::InnerTubeFramesArray& it_frames = _virtuoso_arm->innerTubeFrames();
    // for (int i = 0; i < ot_frames.size(); i++)
    // {
    //     _e3d_mesh.points()[i] = easy3d::vec3(ot_frames[i].origin()[0], ot_frames[i].origin()[1], ot_frames[i].origin()[2]);
    // }
    // for (int i = 0; i < it_frames.size(); i++)
    // {
    //     _e3d_mesh.points()[ot_frames.size() + i] = easy3d::vec3(it_frames[i].origin()[0], it_frames[i].origin()[1], it_frames[i].origin()[2]);
    // }
    // const Sim::VirtuosoArm::OuterTubeFramesArray& ot_frames = _virtuoso_arm->outerTubeFrames();
    // const Sim::VirtuosoArm::InnerTubeFramesArray& it_frames = _virtuoso_arm->innerTubeFrames();

    // make an "outer tube" circle in the XY plane
    double ot_r = _virtuoso_arm->outerTubeDiameter() / 2.0;
    std::array<Eigen::Vector3d, _OT_TUBULAR_RES> ot_circle_pts;
    for (int i = 0; i < _OT_TUBULAR_RES; i++)
    {
        double angle = i * 2 * 3.1415 / _OT_TUBULAR_RES;
        Eigen::Vector3d circle_pt(ot_r * std::cos(angle), ot_r * std::sin(angle), 0.0);
        ot_circle_pts[i] = circle_pt;
    }

    // make an "inner tube" circle in the XY plane
    double it_r = _virtuoso_arm->innerTubeDiameter() / 2.0;
    std::array<Eigen::Vector3d, _IT_TUBULAR_RES> it_circle_pts;
    for (int i = 0; i < _IT_TUBULAR_RES; i++)
    {
        double angle = i * 2 * 3.1415 / _IT_TUBULAR_RES;
        Eigen::Vector3d circle_pt(it_r * std::cos(angle), it_r * std::sin(angle), 0.0);
        it_circle_pts[i] = circle_pt;
    } 

    // outer tube
    for (unsigned fi = 0; fi < ot_frames.size(); fi++)
    {
        const Eigen::Matrix3d rot_mat = ot_frames[fi].transform().rotMat();
        const Eigen::Vector3d translation = ot_frames[fi].transform().translation();
        for (unsigned pi = 0; pi < ot_circle_pts.size(); pi++)
        {
            Eigen::Vector3d transformed_pt = rot_mat * ot_circle_pts[pi] + translation;
            _e3d_mesh.points()[fi*_OT_TUBULAR_RES + pi] = easy3d::vec3(transformed_pt[0], transformed_pt[1], transformed_pt[2]);
        }
    }

    _e3d_mesh.points()[ot_frames.size()*_OT_TUBULAR_RES] = easy3d::vec3(ot_frames[0].origin()[0], ot_frames[0].origin()[1], ot_frames[0].origin()[2]);
    _e3d_mesh.points()[ot_frames.size()*_OT_TUBULAR_RES+1] = easy3d::vec3(ot_frames.back().origin()[0], ot_frames.back().origin()[1], ot_frames.back().origin()[2]);

    // inner tube
    int it_index_offset = ot_frames.size()*_OT_TUBULAR_RES + 2;
    for (unsigned fi = 0; fi < it_frames.size(); fi++)
    {
        const Eigen::Matrix3d rot_mat = it_frames[fi].transform().rotMat();
        const Eigen::Vector3d translation = it_frames[fi].transform().translation();
        for (unsigned pi = 0; pi < it_circle_pts.size(); pi++)
        {
            Eigen::Vector3d transformed_pt = rot_mat * it_circle_pts[pi] + translation;
            _e3d_mesh.points()[it_index_offset + fi*_IT_TUBULAR_RES + pi] = easy3d::vec3(transformed_pt[0], transformed_pt[1], transformed_pt[2]);
        }
    }
    

    _e3d_mesh.points()[it_index_offset + it_frames.size()*_IT_TUBULAR_RES] = easy3d::vec3(it_frames[0].origin()[0], it_frames[0].origin()[1], it_frames[0].origin()[2]);
    _e3d_mesh.points()[it_index_offset + it_frames.size()*_IT_TUBULAR_RES+1] = easy3d::vec3(it_frames.back().origin()[0], it_frames.back().origin()[1], it_frames.back().origin()[2]);


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