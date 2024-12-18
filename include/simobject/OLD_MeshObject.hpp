#ifndef __MESH_OBJECT_HPP
#define __MESH_OBJECT_HPP

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>

#include "config/MeshObjectConfig.hpp"

class Simulation;

/** A class for representing mesh-based objects in the simulation.
 * Inherited classes will implement the material behavior associated with the meshes.
 */
class MeshObject
{

    // public typedefs
    public:
    typedef Eigen::Matrix<double, -1, 3, Eigen::RowMajor> VerticesMat;   // matrix type for vertices
    typedef Eigen::Matrix<unsigned, -1, 3> FacesMat;    // matrix type for faces

    /** Enum that defines which point of reference to take the "position" of a mesh 
     * Used when placing the mesh initially.
    */
    enum PositionReference
    {
        LOWER_LEFT,     // a mesh's position = (min_x, min_y, min_z) of its bounding box
        CENTER,         // a mesh's position = center of its bounding box
        UPPER_RIGHT     // a mesh's position = (max_x, max_y, max_z) of its bounding box
    };

    public:
    
    /** Creates an empty MeshObject with the specified name
     * @param name : the name of the new MeshObject
    */
    explicit MeshObject(const std::string& name);
    
    // explicit MeshObject(const std::string& name, const YAML::Node& config);
    explicit MeshObject(const MeshObjectConfig* config);

    /** Creates MeshObject directly from vertices and elements
     * @param name : the name of the new MeshObject
     * @param verts : the vertices of the new MeshObject
     * @param faces : the triangular faces of the new MeshObject, specified as 3 vertex indexes
    */
    explicit MeshObject(const std::string& name, const VerticesMat& verts, const FacesMat& faces);

    virtual ~MeshObject() = default;

    virtual std::string toString() const;
    virtual std::string type() const { return "MeshObject"; }

    std::string name() const { return _name; }

    VerticesMat vertices() const { return _vertices; }

    int numVertices() const { return _vertices.rows(); }

    FacesMat faces() const { return _faces; }

    int numFaces() const { return _faces.rows(); }

    virtual VerticesMat velocities() const = 0;

    bool vertexOnSurface(const unsigned index) const { return _vertex_on_surface[index]; }

    /** Returns the minimum coordinates from the bounding box of the mesh */
    Eigen::Vector3d bboxMinCoords() const;

    /** Returns the maximum coordinates from the bounding box of the mesh */
    Eigen::Vector3d bboxMaxCoords() const; 

    /** Returns the coordinates of the center from the bounding box of the mesh. */
    Eigen::Vector3d bboxCenterCoords() const;

    /** Performs any setup for the MeshObject (i.e. constraint creation, etc.) */
    virtual void setup() = 0;

    /** Updates the mesh based on a time step 
     * @param dt : the time delta since the last update
     * @param g_accel : the acceleration due to gravity
    */
    virtual void update() = 0;

    std::pair<unsigned,double> averageSurfaceEdgeLength() const;

    /** Sets new vertices for the mesh. Also updates the vertex cache.
     * @param verts : the new matrix of vertices
     */
    virtual void setVertices(const VerticesMat& verts);

    void setVertex(const unsigned index, const Eigen::Vector3d& new_pos);

    void displaceVertex(const unsigned index, const double dx, const double dy, const double dz)
    {
        _vertices(index,0) += dx;
        _vertices(index,1) += dy;
        _vertices(index,2) += dz;
    }

    /** Returns the vertex at the row index specified by the user.
     * @param index : the row in the _vertices matrix
     * @returns the vertex as an Eigen::Vector3d
     */
    Eigen::Vector3d getVertex(const unsigned index) const;

    double* getVertexPointer(const unsigned index) const;

    /** Finds the closest vertex to the specified (x,y,z) points, and returns the row index in the _vertices matrix.
     * For now, just does an O(n) search through the vertices.
     * @param x : the x coordinate
     * @param y : the y coordinate
     * @param z : the z coordinate
     * @returns the row index of the closest vertex to (x,y,z)
    */
    unsigned getClosestVertex(const double x, const double y, const double z) const;

    /** Finds the closest vertex on the mesh surface to the specified (x,y,z) points, and returns the row index in the _vertices matrix.
     * For now, just does an O(n) search through the vertices.
     * @param x : the x coordinate
     * @param y : the y coordinate
     * @param z : the z coordinate
     * @returns the row index of the closest surface vertex to (x,y,z)
     */
    unsigned getClosestSurfaceVertex(const double x, const double y, const double z) const;

    std::vector<unsigned> getVerticesWithX(const double x) const;
    std::vector<unsigned> getVerticesWithY(const double y) const;
    std::vector<unsigned> getVerticesWithZ(const double z) const;

    /** Sets new faces for the mesh.
     * @param faces : the new matrix of faces
     */
    void setFaces(const FacesMat& faces);

    void setSimulation(const Simulation* sim) { _sim = sim; }

    /** Resizes the mesh such that its maximum dimension is no larger than the specified size.
     * @param size_of_max_dim : the new size of the largest dimension of the mesh
     */
    void resize(const double size_of_max_dim);

    /** Resizes the mesh to the specified x, y, and z sizes.
     * @param x_size : the new size of the mesh in the x direction
     * @param y_size : the new size of the mesh in the y direction
     * @param z_size : the new size of the mesh in the z direction
     */
    void resize(const double x_size, const double y_size, const double z_size);

    /** Resizes the mesh to the specified x, y, and z sizes, with a Eigen::Vector3 as an input. 
     * @param size : the Eigen::Vector3d describing the new size of the mesh
    */
   void resize(const Eigen::Vector3d& size);

    /** Moves the mesh to a specified position, with respect to a specified reference point on the mesh's bounding box.
     * @param position : the position to move the mesh to
     * @param reference : the reference point on the mesh that the position corresponds to 
    */
    void moveTo(const Eigen::Vector3d& position, PositionReference reference = PositionReference::CENTER);


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

    private:
    /** Shared initialization code across constructors.
     * Should be called AFTER vertices and faces are set.
     */
    void init();
    

    protected:
    /** Name of the MeshObject */
    std::string _name;
    /** Nx3 vertex matrix */
    VerticesMat _vertices;
    /** Nx3 faces matrix */
    FacesMat _faces;

    /** Keeps track of surface vertices
     * 1: on the surface of the mesh
     * 0: not on the surface of the mesh
     */
    std::vector<bool> _vertex_on_surface;

    /** Store simulation object so we can query properties (such as current sim time) */
    const Simulation* _sim;

    /** Time taken per time step (in seconds).
     * We assume that this is constant throughout the simulation.
     */
    double _dt;

    

};

#endif