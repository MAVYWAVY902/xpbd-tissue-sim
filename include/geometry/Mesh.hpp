#ifndef __MESH_HPP
#define __MESH_HPP

#include "geometry/AABB.hpp"
#include "common/types.hpp"
#include "common/VariadicVectorContainer.hpp"

#include "geometry/MeshProperty.hpp"

#include <optional>

#ifdef HAVE_CUDA
#include <memory>
#include "gpu/resource/GPUResource.hpp"
#endif

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
    typedef Eigen::Matrix<Real, 3, -1, Eigen::ColMajor> VerticesMat; // vertex matrix type
    typedef Eigen::Matrix<int, 3, -1, Eigen::ColMajor> FacesMat;     // faces matrix type
    typedef Eigen::Matrix<int, 4, -1, Eigen::ColMajor> ElementsMat;  // elements matrix type (used by tetrahedral meshes)

public:
    /** Constructs a mesh from a set of vertices and faces.
     * This is usually done using helper methods in the MeshUtils library.
     */
    Mesh(const VerticesMat &vertices, const FacesMat &faces);

    Mesh(const Mesh &other);

    Mesh(Mesh &&other);

    virtual ~Mesh() = default;

    /** Returns a const-reference to the vertices of the mesh. */
    const VerticesMat &vertices() const { return _vertices; }
    /** Returns a const-reference to the faces of the mesh. */
    const FacesMat &faces() const { return _faces; }

    /** Returns a non-const-reference to the vertices of the mesh. */
    VerticesMat &vertices() { return _vertices; }
    /** Returns a non-const-reference to the faces of the mesh. */
    FacesMat &faces() { return _faces; }

    /** Number of verticees in the mesh. */
    int numVertices() const { return _vertices.cols(); }
    /** Number of faces in the mesh. */
    int numFaces() const { return _faces.cols(); }

    /** Essentially "sets up" the mesh - treats the current state as the initial, undeformed state of the mesh.
     * This should be called after performing the initial translations and rotations setting up the mesh.
     */
    virtual void setCurrentStateAsUndeformedState();

    /** Updates the vertex normals in the mesh */
    void updateVertexNormals();

    /** Returns the vertex normal at vertex i */
    Vec3r vertexNormal(int index);

    /** Returns a single vertex as an Eigen 3-vector, given the vertex index. */
    Vec3r vertex(const int index) const { return _vertices.col(index); }

    /** Returns whether or not the vertex is on the surface of the mesh. */
    bool vertexOnSurface(int index) const { const auto& prop = getVertexProperty<bool>("surface"); return prop.get(index); }

    /** Returns a pointer to the vertex data in memory, given the vertex index.
     * This is useful to avoid multiple pointer dereferences in critical code sections (i.e. the Constraint projection)
     */
    Real *vertexPointer(const int index) const;

    /** Sets the vertex at the specified to a new position. */
    void setVertex(int index, const Vec3r &new_pos) { _vertices.col(index) = new_pos; }

    void displaceVertex(int index, const Vec3r &offset) { _vertices.col(index) += offset; }

    const std::vector<int>& vertexAdjacentVertices(int index) { return _vertex_adjacent_vertices[index]; }

    /** Displaces the vertex at the specified index by a certain amount. */
    //  void displaceVertex(const int index, const Real dx, const Real dy, const Real dz)
    //  {
    //      _vertices(0, index) += dx;
    //      _vertices(1, index) += dy;
    //      _vertices(2, index) += dz;
    //  }

    /** Returns a single face as an Eigen 3-vector, given the vertex index. */
    Eigen::Vector3i face(const int index) const { return _faces.col(index); }

    /** Returns the axis-aligned bounding-box (AABB) for the mesh. */
    AABB boundingBox() const;

    /** Returns the unrotated size of the mesh.
     * This does not change when the mesh rotates - only when the mesh is resized.
     */
    Vec3r unrotatedSize() const { return _unrotated_size_xyz; }

    /** Returns the number of edges along with the average edge length in the faces of the mesh. */
    std::pair<int, Real> averageFaceEdgeLength() const;

    /** Finds the closest vertex to the specified (x,y,z) points, and returns the row index in the _vertices matrix.
     * For now, just does an O(n) brute-force search through the vertices.
     * @param p : the point for which to find the closest vertex in the mesh
     * @returns the index of the closest vertex to (x,y,z)
     */
    int getClosestVertex(const Vec3r &p) const;

    /** Returns a list of vertex indices for vertices with the specified x-coordinate. */
    std::vector<int> getVerticesWithX(const Real x) const;

    /** Returns a list of vertex indices for vertices with the specified y-coordinate. */
    std::vector<int> getVerticesWithY(const Real y) const;

    /** Returns a list of vertex indices for vertices with the specified z-coordinate. */
    std::vector<int> getVerticesWithZ(const Real z) const;

    /** Resizes the mesh such that its maximum dimension is no larger than the specified size.
     * @param size_of_max_dim : the new size of the largest dimension of the mesh
     */
    void resize(const Real size_of_max_dim);

    /** Resizes the mesh to the specified x, y, and z sizes, with a Eigen::Vector3 as an input.
     * @param size : the Vec3r describing the new size of the mesh
     */
    void resize(const Vec3r &size);

    /** Moves each vertex in the mesh by the same amount. */
    void moveTogether(const Vec3r &delta);

    /** Moves each vertex in the mesh by a per-vertex amount.
     * Up to the caller to ensure that the per-vertex displacement matrix is the same dimensions as the mesh's vertices matrix.
     */
    void moveSeparate(const VerticesMat &delta);

    /** Moves the center of the AABB of the mesh to a specified position.
     * @param position : the position to move the center of the AABB mesh to
     */
    void moveTo(const Vec3r &position);

    /** Rotates the mesh according to a vector of (x angle, y angle, and z angle) Euler angles around some point p.
     * Usees the XYZ Euler angle convention - rotates x degrees about x-axis, then y degrees about y-axis, and then z degrees about z-axis
     * @param p : the point around which to rotate the mesh
     * @param xyz_angles : a 3-vector corresponding to successive rotation angles
     */
    void rotateAbout(const Vec3r &p, const Vec3r &xyz_angles);

    /** Rotates the mesh using a 3x3 rotation matrix around some point p.
     * @param p : the point around which to rotate the mesh
     * @param rot_mat : the rotation matrix used to rotate the vertices
     */
    void rotateAbout(const Vec3r &p, const Mat3r &rot_mat);

    /** Computes the current total mass, center of mass, and moment of inertia tensor (about center of mass) for the mesh, given a density.
     * Uses the algorithm described here: http://number-none.com/blow/inertia/index.html
     * @param density : the density to be used in the calculation (DEFAULT = 1). If omitted, the "mass" returned is actually the volume of the mesh.
     * @returns these quantities as a 3-tuple: (mass, center-of-mass, moment-of-inertia)
     */
    std::tuple<Real, Vec3r, Mat3r> massProperties(Real density = 1.0) const;

    /** Computes the current center of mass for the mesh.
     * Uses the same algorithm as massProperties(), but without calculating the moment of inertia.
     */
    Vec3r massCenter() const;

    /** Writes the mesh to .obj file.
     * Only writes vertices and faces.
     */
    void writeMeshToObjFile(const std::string &filename) const;

    /** Creates a vertex property with the specified name, and optional default value. */
    template <typename T>
    void addVertexProperty(const std::string &name, std::optional<T> default_value = std::nullopt)
    {
        static_assert(type_list_contains_v<T, MeshPropertyTypeList> && "Mesh property type not supported!");

        // make sure name doesn't already exist
        for (const auto& vprop : _vertex_properties.get<MeshProperty<T>>())
        {
            assert(name != vprop.name() && "Vertex property with name already exists!");
        }
    
        if (default_value.has_value())
        {
            _vertex_properties.template emplace_back<MeshProperty<T>>(name, numVertices(), default_value.value());
        }
        else
        {
            _vertex_properties.template emplace_back<MeshProperty<T>>(name, numVertices());
        }
    }

    /** Fetches a vertex property with name. */
    template <typename T>
    const MeshProperty<T>& getVertexProperty(const std::string& name) const
    {
        static_assert(type_list_contains_v<T, MeshPropertyTypeList> && "Mesh property type not supported!");

        for (const auto& vprop : _vertex_properties.template get<MeshProperty<T>>())
        {
            if (name == vprop.name())
                return vprop;
        }
    
        assert(0 && "Vertex property not found!");
    }

    template <typename T>
    MeshProperty<T>& getVertexProperty(const std::string& name)
    {
        static_assert(type_list_contains_v<T, MeshPropertyTypeList> && "Mesh property type not supported!");

        for (auto& vprop : _vertex_properties.template get<MeshProperty<T>>())
        {
            if (name == vprop.name())
                return vprop;
        }
    
        assert(0 && "Vertex property not found!");
    }

    template <typename T>
    bool hasVertexProperty(const std::string& name) const
    {
        static_assert(type_list_contains_v<T, MeshPropertyTypeList> && "Mesh property type not supported!");

        for (auto& vprop : _vertex_properties.template get<MeshProperty<T>>())
        {
            if (name == vprop.name())
                return true;
        }

        return false;
    }

    /** Creates a face property with the specified name, and optional default value. */
    template <typename T>
    void addFaceProperty(const std::string &name, std::optional<T> default_value = std::nullopt)
    {
        static_assert(type_list_contains_v<T, MeshPropertyTypeList> && "Mesh property type not supported!");

        // make sure name doesn't already exist
        for (const auto& fprop : _face_properties.get<MeshProperty<T>>())
        {
            assert(name != fprop.name() && "Vertex property with name already exists!");
        }
    
        if (default_value.has_value())
        {
            _face_properties.template emplace_back<MeshProperty<T>>(name, numFaces(), default_value.value());
        }
        else
        {
            _face_properties.template emplace_back<MeshProperty<T>>(name, numFaces());
        }
    }

    /** Fetches a face property with name. */
    template <typename T>
    const MeshProperty<T>& getFaceProperty(const std::string& name) const
    {
        static_assert(type_list_contains_v<T, MeshPropertyTypeList> && "Mesh property type not supported!");

        for (const auto& fprop : _face_properties.template get<MeshProperty<T>>())
        {
            if (name == fprop.name())
                return fprop;
        }
    
        assert(0 && "Face property not found!");
    }

    template <typename T>
    MeshProperty<T>& getFaceProperty(const std::string& name)
    {
        static_assert(type_list_contains_v<T, MeshPropertyTypeList> && "Mesh property type not supported!");
        
        for (auto& fprop : _face_properties.template get<MeshProperty<T>>())
        {
            if (name == fprop.name())
                return fprop;
        }
    
        assert(0 && "Face property not found!");
    }

    template <typename T>
    bool hasFaceProperty(const std::string& name) const
    {
        static_assert(type_list_contains_v<T, MeshPropertyTypeList> && "Mesh property type not supported!");

        for (const auto& fprop : _face_properties.template get<MeshProperty<T>>())
        {
            if (name == fprop.name())
                return true;
        }

        return false;
    }

#ifdef HAVE_CUDA
    virtual void createGPUResource();
    virtual const Sim::HostReadableGPUResource *gpuResource() const
    {
        assert(_gpu_resource);
        return _gpu_resource.get();
    }
#endif

protected:
    /** Finds adjacent vertices for each vertex in the mesh.
     * Two vertices are "adjacent" if they are connected by a face or element.
     */
    virtual void _computeAdjacentVertices();

protected:
    VerticesMat _vertices; // the vertices of the mesh
    FacesMat _faces;       // the faces of the mesh
    VerticesMat _vertex_normals; // vertex normals of the mesh

    std::vector<std::vector<int>> _vertex_adjacent_vertices;

    Vec3r _unrotated_size_xyz; // the size of the mesh in each dimension in its unrotated state

    // mesh properties
    PropertyContainer<MeshPropertyTypeList> _vertex_properties;
    PropertyContainer<MeshPropertyTypeList> _face_properties;

#ifdef HAVE_CUDA
    std::unique_ptr<Sim::HostReadableGPUResource> _gpu_resource;
#endif
};

} // namespace Geometry

#endif // __MESH_HPP