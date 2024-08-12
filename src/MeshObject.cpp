#include "MeshObject.hpp"

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

void MeshObject::setFaces(const FacesMat& faces)
{
    _faces = faces;
}

void MeshObject::resize(const double size_of_max_dim)
{
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

void MeshObject::moveTo(const Eigen::Vector3d& position, PositionReference ref)
{
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