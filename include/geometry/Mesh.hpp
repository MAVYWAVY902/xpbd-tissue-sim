#ifndef __MESH_HPP
#define __MESH_HPP

#include "geometry/AABB.hpp"
#include <Eigen/Dense>

namespace Geometry
{

/** A class for a surface mesh which consists of a set of vertices and a set of faces connecting those vertices.
 * The vertices are specified as 3-vectors of vertex coordinates.
 * The faces are specified as 3-vectors of vertex indices.
 * No face normal checking is performed - this class assumes all face normals are pointed correctly outwards from the interior of the mesh.
 */
class Mesh
{
    // public typedefs
    public:
    typedef Eigen::Matrix<double, 3, -1, Eigen::ColMajor> VerticesMat;  // vertex matrix type
    typedef Eigen::Matrix<int, 3, -1, Eigen::ColMajor> FacesMat;        // faces matrix type
    typedef Eigen::Matrix<int, 4, -1, Eigen::ColMajor> ElementsMat;     // elements matrix type (used by tetrahedral meshes)

    public:

    /** Constructs a mesh from a set of vertices and faces.
     * This is usually done using helper methods in the MeshUtils library.
    */
    Mesh(const VerticesMat& vertices, const FacesMat& faces);

    virtual ~Mesh() = default;

    /** Returns a const-reference to the vertices of the mesh. */
    const VerticesMat& vertices() const { return _vertices; }
    /** Returns a const-reference to the faces of the mesh. */
    const FacesMat& faces() const { return _faces; }

    /** Number of verticees in the mesh. */
    int numVertices() const { return _vertices.cols(); }
    /** Number of faces in the mesh. */
    int numFaces() const { return _faces.cols(); }

    /** Returns a single vertex as an Eigen 3-vector, given the vertex index. */
    Eigen::Vector3d vertex(const int index) const { return _vertices.col(index); }

    /** Returns a pointer to the vertex data in memory, given the vertex index.
     * This is useful to avoid multiple pointer dereferences in critical code sections (i.e. the Constraint projection)
     */
    double* vertexPointer(const int index) const;

    /** Sets the vertex at the specified to a new position. */
    void setVertex(const int index, const Eigen::Vector3d& new_pos) { _vertices.col(index) = new_pos; }
    
    /** Displaces the vertex at the specified index by a certain amount. */
    void displaceVertex(const int index, const double dx, const double dy, const double dz)
    {
        _vertices(0, index) += dx;
        _vertices(1, index) += dy;
        _vertices(2, index) += dz;
    }

    /** Returns a single face as an Eigen 3-vector, given the vertex index. */
    Eigen::Vector3i face(const int index) const { return _faces.col(index); }

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

    /** Moves each vertex in the mesh by the same amount. */
    void moveTogether(const Eigen::Vector3d& delta);

    /** Moves each vertex in the mesh by a per-vertex amount.
     * Up to the caller to ensure that the per-vertex displacement matrix is the same dimensions as the mesh's vertices matrix.
     */
    void moveSeparate(const VerticesMat& delta);

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

    VerticesMat _vertices;  // the vertices of the mesh
    FacesMat _faces;    // the faces of the mesh
};

} // namespace Geometry

#endif // __MESH_HPP