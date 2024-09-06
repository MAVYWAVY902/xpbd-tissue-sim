#include "MeshObject.hpp"
#include "Simulation.hpp"

#include <easy3d/renderer/renderer.h> 

void MeshObject::init()
{
    // first ensure that the vertex cache has enough space for each vertex
    _vertex_cache.resize(_vertices.rows());
    // then update the vertex cache to populate it initially
    updateVertexCache();

    // create a new Renderer for this Model so that the Drawables (below) get updated
    set_renderer(new easy3d::Renderer(this, false));

    // create a TrianglesDrawable for the faces of the tetrahedral mesh
    easy3d::TrianglesDrawable* tri_drawable = renderer()->add_triangles_drawable("faces");
    // specify the update function for the faces
    tri_drawable->set_update_func([](easy3d::Model* m, easy3d::Drawable* d) {
        // downcast to MeshObject for access to facesAsFlatList
        MeshObject* mo = dynamic_cast<MeshObject*>(m);
        if (mo)
        {
            // update the vertex buffer and element buffer
            d->update_vertex_buffer(mo->points(), true);
            d->update_element_buffer(mo->facesAsFlatList());
        }
        
    });
    tri_drawable->set_uniform_coloring(_color);

    if (_draw_points)
    {
        // create a PointsDrawable for the points of the tetrahedral mesh
        easy3d::PointsDrawable* points_drawable = renderer()->add_points_drawable("vertices");
        // specify the update function for the points
        points_drawable->set_update_func([](easy3d::Model* m, easy3d::Drawable* d) {
            // update the vertex buffer with the vertices of the mesh
            d->update_vertex_buffer(m->points(), true);
        });
    }
}

MeshObject::MeshObject(const std::string& name)
    : easy3d::Model(name), _color(1.0f, 1.0f, 1.0f, 1.0f)
{
    init();
}

// MeshObject::MeshObject(const std::string& name, const YAML::Node& config)
//     : easy3d::Model(name), _color(1.0f, 1.0f, 1.0f, 1.0f)
// {
//     // read color from config, if it exists
//     YAML::Node color_yaml_node = config["color"];
//     if (color_yaml_node.Type() != YAML::NodeType::Null)
//     {
//         _color = easy3d::vec4(color_yaml_node[0].as<double>(), color_yaml_node[1].as<double>(), color_yaml_node[2].as<double>(), color_yaml_node[3].as<double>());
//     }

//     // read draw points flag from config
//     YAML::Node draw_points_yaml_node = config["draw-points"];
//     if (draw_points_yaml_node.Type() != YAML::NodeType::Null)
//     {
//         _draw_points = draw_points_yaml_node.as<bool>();
//     }

//     init();
// }
MeshObject::MeshObject(const MeshObjectConfig* config)
    : easy3d::Model(config->name().value_or(""))
{
    Eigen::Vector4d color_eigen_vec = config->color().value_or(Eigen::Vector4d::Constant(1.0));
    _color = easy3d::vec4(color_eigen_vec(0), color_eigen_vec(1), color_eigen_vec(2), color_eigen_vec(3));

    _draw_points = config->drawPoints().value_or(false);

    _dt = config->timeStep().value();

    init();

}

MeshObject::MeshObject(const std::string& name, const VerticesMat& verts, const FacesMat& faces)
    : easy3d::Model(name), _vertices(verts), _faces(faces), _vertex_cache(verts.rows()), _color(1.0f, 1.0f, 1.0f, 1.0f)
{
    init();
}

void MeshObject::updateGraphics()
{
    // update the vertex cache, which is what the renderer uses to update the vertex positions
    updateVertexCache();
    // then call update on the renderer, which will invoke the Drawable update functions
    renderer()->update();
}

void MeshObject::updateVertexCache()
{
    // make sure the vertex cache is big enough for all the vertices
    assert(_vertex_cache.size() == _vertices.rows());
    // loop through and update each vertex in the cache
    for (size_t i = 0; i < _vertices.rows(); i++)
    {
        _vertex_cache.at(i) = (easy3d::vec3(_vertices(i,0), _vertices(i,1), _vertices(i,2)));
    }
}

const std::vector<easy3d::vec3>& MeshObject::points() const
{
    return _vertex_cache;
}

std::vector<easy3d::vec3>& MeshObject::points()
{
    return _vertex_cache;
}

Eigen::Vector3d MeshObject::bboxMinCoords() const
{
    return _vertices.colwise().minCoeff();
}

Eigen::Vector3d MeshObject::bboxMaxCoords() const
{
    return _vertices.colwise().maxCoeff();
}

std::vector<unsigned int> MeshObject::facesAsFlatList() const
{
    // each face (triangle) has 3 vertices
    std::vector<unsigned int> faces_flat_list(_faces.rows()*3);

    for (const auto& face : _faces.rowwise())
    {
        faces_flat_list.insert(faces_flat_list.end(), {face(0), face(1), face(2)});
    }

    return faces_flat_list;
}

void MeshObject::setVertices(const VerticesMat& verts)
{
    _vertices = verts;
    // ensure that the vertex cache has the right size before updating it
    _vertex_cache.resize(_vertices.rows());
    updateVertexCache();
}

Eigen::Vector3d MeshObject::getVertex(const unsigned index) const
{
    assert(index < _vertices.rows());
    return _vertices.row(index);
}

unsigned MeshObject::getClosestVertex(const double x, const double y, const double z) const
{
    auto sq_dist_to_vertex = [x, y, z] (const Eigen::Vector3d& vertex)
    {
        return (x-vertex(0))*(x-vertex(0)) + (y-vertex(1))*(y-vertex(1)) + (z-vertex(2))*(z-vertex(2));
    };

    unsigned closest_index = 0;
    double closest_dist = sq_dist_to_vertex(_vertices.row(0));
    // I'm sure there is a good way to do this with std
    for (int i = 0; i < _vertices.rows(); i++)
    {
        double dist = sq_dist_to_vertex(_vertices.row(i));
        if (dist < closest_dist)
        {
            closest_index = i;
            closest_dist = dist;
        }
    }

    return closest_index;
}

void MeshObject::setFaces(const FacesMat& faces)
{
    _faces = faces;
}

void MeshObject::resize(const double size_of_max_dim)
{
    if (_vertices.rows() == 0)
    {
        std::cerr << "Vertices matrix is empty!" << std::endl;
        return;
    }

    // compute the size of the maximum dimension
    Eigen::Vector3d min_coords = _vertices.colwise().minCoeff();
    Eigen::Vector3d max_coords = _vertices.colwise().maxCoeff();
    // compute the scaling factor such that when the mesh is scaled it has the desired size
    double scaling_factor = size_of_max_dim / (max_coords - min_coords).maxCoeff();

    // move all vertices to be centered around (0, 0, 0), apply scaling, and then move them back
    Eigen::Vector3d center_of_bbox = min_coords + (max_coords - min_coords) / 2;
    _vertices.rowwise() += -center_of_bbox.transpose();
    _vertices *= scaling_factor;
    _vertices.rowwise() += center_of_bbox.transpose();
}

void MeshObject::resize(const double x_size, const double y_size, const double z_size)
{
    if (_vertices.rows() == 0)
    {
        std::cerr << "Vertices matrix is empty!" << std::endl;
        return;
    }

    // compute the current size in each dimension
    Eigen::Vector3d min_coords = _vertices.colwise().minCoeff();
    Eigen::Vector3d max_coords = _vertices.colwise().maxCoeff();
    Eigen::Vector3d size = max_coords - min_coords;

    // compute the scaling factors for each dimension
    double scaling_factor_x = (size(0) != 0) ? x_size / size(0) : 1;
    double scaling_factor_y = (size(1) != 0) ? y_size / size(1) : 1;
    double scaling_factor_z = (size(2) != 0) ? z_size / size(2) : 1;

    // move all vertices to be centered around (0, 0, 0), apply scaling, then move them back
    Eigen::Vector3d center_of_bbox = min_coords + size / 2;
    _vertices.rowwise() += -center_of_bbox.transpose();
    _vertices.col(0) *= scaling_factor_x;
    _vertices.col(1) *= scaling_factor_y;
    _vertices.col(2) *= scaling_factor_z;
    _vertices.rowwise() += center_of_bbox.transpose();

}

void MeshObject::resize(const Eigen::Vector3d& size)
{
    resize(size(0), size(1), size(2));
}

void MeshObject::moveTo(const Eigen::Vector3d& position, PositionReference ref)
{
    if (_vertices.rows() == 0)
    {
        std::cerr << "Vertices matrix is empty!" << std::endl;
        return;
    }

    // the position offset to be applied to each vertex in the mesh
    // depends on not only the desired position but the reference
    Eigen::Vector3d pos_offset;

    // mesh should be moved such that the (min_x, min_y, min_z) of the bounding box is at the specified position
    if (ref == PositionReference::LOWER_LEFT)
    {
        Eigen::Vector3d min_coords = _vertices.colwise().minCoeff();
        pos_offset = (position - min_coords);
        
    }
    // mesh should be moved such that the (max_x, max_y, max_z) of the bounding box is at the specified position
    else if (ref == PositionReference::UPPER_RIGHT)
    {
        Eigen::Vector3d max_coords = _vertices.colwise().maxCoeff();
        pos_offset = (position - max_coords);
    }
    // mesh should be moved such that the center of the bounding box is at the specified position
    else if (ref == PositionReference::CENTER)
    {
        Eigen::Vector3d min_coords = _vertices.colwise().minCoeff();
        Eigen::Vector3d max_coords = _vertices.colwise().maxCoeff();
        Eigen::Vector3d center_of_bbox = min_coords + (max_coords - min_coords) / 2;
        pos_offset = (position - center_of_bbox);
    }

    // apply the position offset
    _vertices.rowwise() += pos_offset.transpose();
}

void MeshObject::rotate(const Eigen::Vector3d& xyz_angles)
{
    const double x = xyz_angles(0) * M_PI / 180.0;
    const double y = xyz_angles(1) * M_PI / 180.0;
    const double z = xyz_angles(2) * M_PI / 180.0;
    // using the "123" convention: rotate first about x axis, then about y, then about z
    Eigen::Matrix3d rot_mat;
    rot_mat(0,0) = std::cos(y) * std::cos(z);
    rot_mat(0,1) = std::sin(x)*std::sin(y)*std::cos(z) - std::cos(x)*std::sin(z);
    rot_mat(0,2) = std::cos(x)*std::sin(y)*std::cos(z) + std::sin(x)*std::sin(z);

    rot_mat(1,0) = std::cos(y)*std::sin(z);
    rot_mat(1,1) = std::sin(x)*std::sin(y)*std::sin(z) + std::cos(x)*std::cos(z);
    rot_mat(1,2) = std::cos(x)*std::sin(y)*std::sin(z) - std::sin(x)*std::cos(z);

    rot_mat(2,0) = -std::sin(y);
    rot_mat(2,1) = std::sin(x)*std::cos(y);
    rot_mat(2,2) = std::cos(x)*std::cos(y);
    
    rotate(rot_mat);
}

void MeshObject::rotate(const Eigen::Matrix3d& rot_mat)
{
    // compute the current size in each dimension
    Eigen::Vector3d min_coords = _vertices.colwise().minCoeff();
    Eigen::Vector3d max_coords = _vertices.colwise().maxCoeff();
    Eigen::Vector3d size = max_coords - min_coords;
    Eigen::Vector3d center_of_bbox = min_coords + size / 2;
    _vertices.rowwise() += -center_of_bbox.transpose();
    // vertices are row vectors, so transpose rotation matrix
    _vertices = _vertices * rot_mat.transpose();
    _vertices.rowwise() += center_of_bbox.transpose();
}