#include "MeshObject.hpp"
#include "Simulation.hpp"

#include <easy3d/renderer/renderer.h> 

void MeshObject::init()
{
    
}

MeshObject::MeshObject(const std::string& name)
    : _name(name)
{
    init();
}

MeshObject::MeshObject(const MeshObjectConfig* config)
{
    _name = config->name().value_or("");
    _dt = config->timeStep().value();

    init();

}

MeshObject::MeshObject(const std::string& name, const VerticesMat& verts, const FacesMat& faces)
    : _vertices(verts), _faces(faces)
{
    init();
}

std::string MeshObject::toString() const
{
    return type() + " '" + name() + "':\n\tNum Vertices: " + std::to_string(_vertices.rows()) + "\n\tNum faces: " + std::to_string(_faces.rows());  
}

Eigen::Vector3d MeshObject::bboxMinCoords() const
{
    return _vertices.colwise().minCoeff();
}

Eigen::Vector3d MeshObject::bboxMaxCoords() const
{
    return _vertices.colwise().maxCoeff();
}

Eigen::Vector3d MeshObject::bboxCenterCoords() const
{
    Eigen::Vector3d mins = bboxMinCoords();
    Eigen::Vector3d maxs = bboxMaxCoords();
    return mins + (maxs - mins)/2;
}

// std::vector<unsigned int> MeshObject::facesAsFlatList() const
// {
//     // each face (triangle) has 3 vertices
//     std::vector<unsigned int> faces_flat_list(_faces.rows()*3);

//     for (const auto& face : _faces.rowwise())
//     {
//         faces_flat_list.insert(faces_flat_list.end(), {face(0), face(1), face(2)});
//     }

//     return faces_flat_list;
// }

// std::vector<unsigned int> MeshObject::edgesAsFlatList() const
// {
//     // TODO: filter duplicate edges
//     std::vector<unsigned int> edges_flat_list(_faces.rows()*3);

//     for (const auto& face : _faces.rowwise())
//     {
//         edges_flat_list.insert(edges_flat_list.end(), {face(0), face(1), face(1), face(2), face(0), face(2)});
//     }

//     return edges_flat_list;
// }

void MeshObject::setVertices(const VerticesMat& verts)
{
    _vertices = verts;
    // ensure that surface vertices vector has the right size
    _vertex_on_surface.resize(_vertices.rows());
}

void MeshObject::setVertex(const unsigned index, const Eigen::Vector3d& new_pos)
{
    _vertices.row(index) = new_pos;
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

unsigned MeshObject::getClosestSurfaceVertex(const double x, const double y, const double z) const
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
        // only consider vertices on the mesh surface
        if (!_vertex_on_surface.at(i))
            continue;

        double dist = sq_dist_to_vertex(_vertices.row(i));
        if (dist < closest_dist)
        {
            closest_index = i;
            closest_dist = dist;
        }
    }

    return closest_index;
}

std::vector<unsigned> MeshObject::getVerticesWithX(const double x) const
{
    std::vector<unsigned> verts;
    for (int i = 0; i < _vertices.rows(); i++)
    {
        if (_vertices(i,0) == x)
        {
            verts.push_back(i);
        }
    }

    return verts;
}

std::vector<unsigned> MeshObject::getVerticesWithY(const double y) const
{
    std::vector<unsigned> verts;
    for (int i = 0; i < _vertices.rows(); i++)
    {
        if (_vertices(i,1) == y)
        {
            verts.push_back(i);
        }
    }

    return verts;
}

std::vector<unsigned> MeshObject::getVerticesWithZ(const double z) const
{
    std::vector<unsigned> verts;
    for (int i = 0; i < _vertices.rows(); i++)
    {
        if (_vertices(i,2) == z)
        {
            verts.push_back(i);
        }
    }

    return verts;
}

void MeshObject::setFaces(const FacesMat& faces)
{
    _faces = faces;

    // iterate through surface faces to see which vertices are on the surface
    _vertex_on_surface.assign(_vertices.rows(), 0);
    for (const auto& face : _faces.rowwise())
    {
        _vertex_on_surface.at(face(0)) = true;
        _vertex_on_surface.at(face(1)) = true;
        _vertex_on_surface.at(face(2)) = true;
    }
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