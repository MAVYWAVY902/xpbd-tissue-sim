#ifndef __MESH_HPP
#define __MESH_HPP

#include "geometry/AABB.hpp"
#include <Eigen/Dense>

namespace Geometry
{

class Mesh
{
    // public typedefs
    public:
    typedef Eigen::Matrix<double, 3, -1, Eigen::ColMajor> VerticesMat;
    typedef Eigen::Matrix<int, 3, -1, Eigen::ColMajor> FacesMat;
    typedef Eigen::Matrix<int, 4, -1, Eigen::ColMajor> ElementsMat;

    public:

    Mesh(const VerticesMat& vertices, const FacesMat& faces);

    virtual ~Mesh() = default;

    const VerticesMat& vertices() const { return _vertices; }
    const FacesMat& faces() const { return _faces; }

    int numVertices() const { return _vertices.cols(); }
    int numFaces() const { return _faces.cols(); }

    const Eigen::Vector3d& vertex(const int index) const { return _vertices.col(index); }
    double* vertexPointer(const int index) const;
    void setVertex(const int index, const Eigen::Vector3d& new_pos) { _vertices.col(index) = new_pos; }

    const Eigen::Vector3i& face(const int index) const { return _faces.col(index); }

    /** Returns the axis-aligned bounding-box (AABB) for the mesh. */
    AABB boundingBox() const; 

    /** Returns the number of edges along with the average edge length in the faces of the mesh. */
    std::pair<int,double> averageFaceEdgeLength() const;

    /** Finds the closest vertex to the specified (x,y,z) points, and returns the row index in the _vertices matrix.
     * For now, just does an O(n) brute-force search through the vertices.
     * @param p : the point for which to find the closest vertex in the mesh
     * @returns the index of the closest vertex to (x,y,z)
    */
    int getClosestVertex(const Eigen::Vector3d& p) const;

    /** Returns a list of vertex indices for vertices with the specified x-coordinate. */
    std::vector<int> getVerticesWithX(const double x) const;

    /** Returns a list of vertex indices for vertices with the specified y-coordinate. */
    std::vector<int> getVerticesWithY(const double y) const;

    /** Returns a list of vertex indices for vertices with the specified z-coordinate. */
    std::vector<int> getVerticesWithZ(const double z) const;

    /** Resizes the mesh such that its maximum dimension is no larger than the specified size.
     * @param size_of_max_dim : the new size of the largest dimension of the mesh
    */
    void resize(const double size_of_max_dim);

    /** Resizes the mesh to the specified x, y, and z sizes, with a Eigen::Vector3 as an input. 
     * @param size : the Eigen::Vector3d describing the new size of the mesh
    */
    void resize(const Eigen::Vector3d& size);

    /** Moves the mesh by a specified amount. */
    void move(const Eigen::Vector3d& delta);

    /** Moves the center of the AABB of the mesh to a specified position.
     * @param position : the position to move the center of the AABB mesh to
    */
    void moveTo(const Eigen::Vector3d& position);

    /** Rotates the mesh according to a vector of (x angle, y angle, and z angle) around its bounding box center
     * Rotates x degrees about x-axis, then y degrees about y-axis, and then z degrees about z-axis
     * @param xyz_angles : a 3-vector corresponding to successive rotation angles
    */
    void rotate(const Eigen::Vector3d& xyz_angles);

    /** Rotates the mesh according to a rotation matrix around its bounding box center
     * First moves the mesh to have its bounding box center at the origin, applies the rotation, and moves the mesh back its previous position.
     * @param rot_mat : the rotation matrix used to rotate the vertices
    */
    void rotate(const Eigen::Matrix3d& rot_mat);
    
    protected:

    VerticesMat _vertices;
    FacesMat _faces;
};

} // namespace Geometry

#endif // __MESH_HPP