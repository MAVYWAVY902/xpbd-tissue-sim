#ifndef __MESH_OBJECT_HPP
#define __MESH_OBJECT_HPP

#include <easy3d/renderer/drawable_lines.h>
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/renderer/drawable_triangles.h>
#include <easy3d/core/types.h>
#include <easy3d/core/model.h>

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>

#include "config/MeshObjectConfig.hpp"

/** A class for representing mesh-based objects in the simulation.
 * Inherited classes will implement the material behavior associated with the meshes.
 * 
 * 
 * Inherits from easy3d::Model so it handles its own rendering.
 */
class MeshObject : public easy3d::Model
{

    // public typedefs
    public:
    typedef Eigen::Matrix<double, -1, 3> VerticesMat;   // matrix type for vertices
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

    /** Creates MeshObject by loading from a file
     * @param name : the name of the new MeshObject
     * @param filename : the filename to read elements and vertices from
     */
    // explicit MeshObject(const std::string& name, const std::string& filename);

    /** Creates MeshObject directly from vertices and elements
     * @param name : the name of the new MeshObject
     * @param verts : the vertices of the new MeshObject
     * @param faces : the triangular faces of the new MeshObject, specified as 3 vertex indexes
    */
    explicit MeshObject(const std::string& name, const VerticesMat& verts, const FacesMat& faces);

    /** Returns the vec3 vertex cache.
     * Does NOT check if vertices are stale.
     * 
     * A required override for the Model class.
     */
    std::vector<easy3d::vec3>& points() override;

    /** Returns the vec3 vertex cache.
     * Does NOT check if vertices are stale.
     * 
     * A required override for the Model class.
     */
    const std::vector<easy3d::vec3>& points() const override;

    /** Updates the mesh based on a time step 
     * @param dt : the time delta since the last update
    */
    virtual void update(const double dt) = 0;

    /** Updates the graphics buffers associated with this mesh
     */
    virtual void updateGraphics();

    /** Returns the faces of the mesh as a flat vector of vertex indices. Used for moving face information to GPU via Easy3d.
     * As per specified by TrianglesDrawable docs, each 3 consecutive vertices represents a face.
     * @returns a 1d vecor of vertex indices - 3 consecutive entries corresponds to a face to be rendered.
     */
    std::vector<unsigned int> facesAsFlatList() const;

    /** Sets new vertices for the mesh. Also updates the vertex cache.
     * @param verts : the new matrix of vertices
     */
    void setVertices(const VerticesMat& verts);

    /** Sets new faces for the mesh.
     * @param faces : the new matrix of faces
     */
    void setFaces(const FacesMat& faces);

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
     * 
     */
    void moveTo(const Eigen::Vector3d& position, PositionReference reference = PositionReference::CENTER);


    protected:
    /** Updates the vertex cache. Should be called when the _vertices matrix has been changed.
     * Does NOT reallocate storage for the vertices, it has a fixed amount of space.
     * Each vertex is written over the top of the previous version of itself.
     */
    void updateVertexCache();

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

    /** Vector of vec3 vertices that is continually updated as _vertices changes.
     * Used by easy3d to update the vertex buffers (i.e. the positions) of the geometry on the graphics side.
     */
    std::vector<easy3d::vec3> _vertex_cache;

    /** Constant color to be used by the mesh. */
    easy3d::vec4 _color;

    /** Whether or not to draw the points of the mesh */
    bool _draw_points;

};

#endif