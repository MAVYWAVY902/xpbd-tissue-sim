#include "geometry/Mesh.hpp"

#include <set>
#include <iostream>

namespace Geometry
{

Mesh::Mesh(const VerticesMat& vertices, const FacesMat& faces)
    : _vertices(vertices), _faces(faces)
{
    AABB bbox = boundingBox();
    _unrotated_size_xyz = bbox.size();
}

double* Mesh::vertexPointer(const int index) const
{
    assert(index < numVertices());
    double* p = const_cast<double*>(_vertices.col(index).data());
    return p;
}

AABB Mesh::boundingBox() const
{
    Eigen::Vector3d min = _vertices.rowwise().minCoeff();
    Eigen::Vector3d max = _vertices.rowwise().maxCoeff();
    return AABB(min, max);
}

std::pair<int,double> Mesh::averageFaceEdgeLength() const
{
    // keep track of unique edges, stored in ascending-index order
    std::set<std::pair<int, int>> edges;

    // add all the unique edges to the set
    for (const auto& face : _faces.colwise())
    {
        // 3 edges per face
        // make sure to insert them into the set in ascending-index order
        if (face[0] > face[1])  edges.insert(std::pair<int,int>(face[1], face[0]));
        else                    edges.insert(std::pair<int,int>(face[0], face[1]));

        if (face[0] > face[2])  edges.insert(std::pair<int,int>(face[2], face[0]));
        else                    edges.insert(std::pair<int,int>(face[0], face[2]));

        if (face[1] > face[2])  edges.insert(std::pair<int,int>(face[2], face[1]));
        else                    edges.insert(std::pair<int,int>(face[1], face[2]));
    }

    // go through set of edges and add up total edge length
    double total_edge_length = 0;
    for (const auto& edge : edges)
    {
        total_edge_length += (vertex(edge.first) - vertex(edge.second)).norm();
    }

    // return the number of unqiue edges as well as the average edge length in the mesh
    return std::pair<int,double>(edges.size(), total_edge_length / edges.size());
}

int Mesh::getClosestVertex(const Eigen::Vector3d& p) const
{
    unsigned closest_index = 0;
    double closest_dist = (p - vertex(0)).squaredNorm();
    // I'm sure there is a better way to do this with std
    for (int i = 0; i < _vertices.cols(); i++)
    {
        double dist = (p - vertex(i)).squaredNorm();
        if (dist < closest_dist)
        {
            closest_index = i;
            closest_dist = dist;
        }
    }

    return closest_index;

}

std::vector<int> Mesh::getVerticesWithX(const double x) const
{
    std::vector<int> verts;
    for (int i = 0; i < _vertices.rows(); i++)
    {
        if (_vertices(i,0) == x)
        {
            verts.push_back(i);
        }
    }

    return verts;
}

std::vector<int> Mesh::getVerticesWithY(const double y) const
{
    std::vector<int> verts;
    for (int i = 0; i < _vertices.rows(); i++)
    {
        if (_vertices(i,1) == y)
        {
            verts.push_back(i);
        }
    }

    return verts;
}

std::vector<int> Mesh::getVerticesWithZ(const double z) const
{
    std::vector<int> verts;
    for (int i = 0; i < _vertices.rows(); i++)
    {
        if (_vertices(i,2) == z)
        {
            verts.push_back(i);
        }
    }

    return verts;
}

void Mesh::resize(const double size_of_max_dim)
{
    // compute the AABB
    const AABB aabb = boundingBox();
    // find the scaling factor such that when the mesh is scaled the largest dimension has the specified size
    double scaling_factor = size_of_max_dim / (aabb.max - aabb.min).maxCoeff();

    // move all vertices to be centered around (0,0,0), apply the scaling, and then move them back
    // moveTogether(-aabb.center());
    _vertices *= scaling_factor;
    // moveTogether(aabb.center());

    // scale the unrotated size
    _unrotated_size_xyz *= scaling_factor;
}

void Mesh::resize(const Eigen::Vector3d& new_size)
{
    // compute the AABB
    const AABB aabb = boundingBox();
    const Eigen::Vector3d size = aabb.size();
    // compute the scaling factors for each dimension
    // compute the scaling factors for each dimension
    double scaling_factor_x = (size(0) != 0) ? new_size(0) / size(0) : 1;
    double scaling_factor_y = (size(1) != 0) ? new_size(1) / size(1) : 1;
    double scaling_factor_z = (size(2) != 0) ? new_size(2) / size(2) : 1;

    // move all vertices to be centered around (0,0,0), apply the scaling, then move them back
    // moveTogether(-aabb.center());
    _vertices.row(0) *= scaling_factor_x;
    _vertices.row(1) *= scaling_factor_y;
    _vertices.row(2) *= scaling_factor_z;
    // moveTogether(aabb.center());

    // scale the unrotated size
    _unrotated_size_xyz[0] *= scaling_factor_x;
    _unrotated_size_xyz[1] *= scaling_factor_y;
    _unrotated_size_xyz[2] *= scaling_factor_z;
}

void Mesh::moveTogether(const Eigen::Vector3d& delta)
{
    _vertices.colwise() += delta;
}

void Mesh::moveSeparate(const VerticesMat& delta)
{
    _vertices.noalias() += delta;
}

void Mesh::moveTo(const Eigen::Vector3d& position)
{

    // calculate the required position offset based on the current center of the AABB
    const AABB aabb = boundingBox();
    const Eigen::Vector3d offset = position - aabb.center();

    // apply the position offset
    moveTogether(offset);
}

void Mesh::rotateAbout(const Eigen::Vector3d& p, const Eigen::Vector3d& xyz_angles)
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
    
    rotateAbout(p, rot_mat);
}

void Mesh::rotateAbout(const Eigen::Vector3d& p, const Eigen::Matrix3d& rot_mat)
{
    moveTogether(-p);
    _vertices = rot_mat * _vertices;
    moveTogether(p);
}

std::tuple<double, Eigen::Vector3d, Eigen::Matrix3d> Mesh::massProperties(double density) const
{
    // uses the algorithm described here: http://number-none.com/blow/inertia/index.html
    double total_volume = 0;
    Eigen::Vector3d center_of_mass(0,0,0);
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();

    // covariance of "canonical" tetrahedron which is (0,0,0), (1,0,0), (0,1,0), (0,0,1)
    Eigen::Matrix3d C_canonical;
    C_canonical <<  1.0/60.0, 1.0/120.0, 1.0/120.0,
                    1.0/120.0, 1.0/60.0, 1.0/120.0,
                    1.0/120.0, 1.0/120.0, 1.0/60.0;
    for (const auto& f : _faces.colwise())
    {
        // each triangle in the mesh + origin forms a tetrahedron
        // v0=origin, v1=f[0], v2=f[1], v3=f[2]
        const Eigen::Vector3d v0(0,0,0);
        const Eigen::Vector3d v1 = _vertices.col(f[0]);
        const Eigen::Vector3d v2 = _vertices.col(f[1]);
        const Eigen::Vector3d v3 = _vertices.col(f[2]);

        // tet basis matrix
        Eigen::Matrix3d A;
        A.col(0) = (v1 - v0);
        A.col(1) = (v2 - v0);
        A.col(2) = (v3 - v0);

        // find signed volume of tet
        const double volume = A.determinant() / 6.0;

        // calculate the center of mass of this tetrahedron - just average of 4 vertices
        const Eigen::Vector3d tet_cm = 0.25*(v0 + v1 + v2 + v3);
        // update overall center of mass using a weighted average
        if (total_volume + volume > 0)
            center_of_mass = (center_of_mass*total_volume + tet_cm*volume) / (total_volume + volume);

        // add covariance matrix from this tet
        covariance += A.determinant() * A * C_canonical * A.transpose();
        // update overall volume
        total_volume += volume;
    }

    // move covariance matrix to center of mass
    covariance = covariance + total_volume * ( 2*(-center_of_mass) * (center_of_mass).transpose() + (-center_of_mass)*(-center_of_mass).transpose());

    // compute moment of inertia tensor from covariance mat
    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity() * covariance.trace() - covariance;

    return std::tuple<double, Eigen::Vector3d, Eigen::Matrix3d>(density*total_volume, center_of_mass, density*I);
}

Eigen::Vector3d Mesh::massCenter() const
{
    double total_volume = 0;
    Eigen::Vector3d center_of_mass(0,0,0);
    for (const auto& f : _faces.colwise())
    {
        // each triangle in the mesh + origin forms a tetrahedron
        // v0=origin, v1=f[0], v2=f[1], v3=f[2]
        const Eigen::Vector3d v0(0,0,0);
        const Eigen::Vector3d v1 = _vertices.col(f[0]);
        const Eigen::Vector3d v2 = _vertices.col(f[1]);
        const Eigen::Vector3d v3 = _vertices.col(f[2]);

        // tet basis matrix
        Eigen::Matrix3d A;
        A.col(0) = (v1 - v0);
        A.col(1) = (v2 - v0);
        A.col(2) = (v3 - v0);

        // find signed volume of tet
        const double volume = A.determinant() / 6.0;

        // calculate the center of mass of this tetrahedron - just average of 4 vertices
        const Eigen::Vector3d tet_cm = 0.25*(v0 + v1 + v2 + v3);
        // update overall center of mass using a weighted average
        if (total_volume + volume > 0)
            center_of_mass = (center_of_mass*total_volume + tet_cm*volume) / (total_volume + volume);

        // update overall volume
        total_volume += volume;
    }

    return center_of_mass;
}

} // namespace Geometry