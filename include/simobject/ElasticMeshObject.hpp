#ifndef __ELASTIC_MESH_OBJECT_HPP
#define __ELASTIC_MESH_OBJECT_HPP

#include "MeshObject.hpp"
#include "ElasticMaterial.hpp"
#include "config/ElasticMeshObjectConfig.hpp"

/** A class for driving a vertex */
class VertexDriver
{
    public:
    typedef std::function<Eigen::Vector3d(const double t)> DriverFunction;

    explicit VertexDriver(const std::string& name, unsigned vert_index, DriverFunction driver_function)
        : _name(name), _vert_index(vert_index), _driver_function(driver_function)
    {}
    
    Eigen::Vector3d evaluate(const double t) const { return _driver_function(t); }
    unsigned vertexIndex() const { return _vert_index; }
    std::string name() const { return _name; }

    protected:
    std::string _name;
    unsigned _vert_index;
    DriverFunction _driver_function;
};

/** A class for representing tetrahedral meshes with elastic material properties.
 * Each vertex in the mesh has its own velocity.
 * 
 */
class ElasticMeshObject : public MeshObject
{
    public:
    typedef Eigen::Matrix<unsigned, -1, 4> ElementsMat; // matrix type for elements

    public:

    /** Creates an empty ElasticMeshObject with the specified name.
     * @param name : the name of the new ElasticMeshObject
     */
    explicit ElasticMeshObject(const std::string& name);

    /** Creates an ElasticMeshObject from a YAML node
     * @param name : the name of the new ElasticMeshObject
     * @param config : the YAML node dictionary describing the parameters for the ElasticMeshObject
     */
    // explicit ElasticMeshObject(const std::string& name, const YAML::Node& config);
    explicit ElasticMeshObject(const ElasticMeshObjectConfig* config);
    
    /** Creates an ElasticMeshObject by loading data from a mesh file
     * @param name : the name of the new ElasticMeshObject
     * @param filename : the filename to load the mesh from. Does not have to be a .msh file, any .obj or .stl file will be volumetrically meshed using gmsh.
     * @param material : describes the material properties of the elastic material
     */
    explicit ElasticMeshObject(const std::string& name, const std::string& filename, const ElasticMaterial& material);

    /** Creates an ElasticMeshObject directly from a matrix of vertices and elements.
     * @param name : the name of the new ElasticMeshObject
     * @param verts : the matrix of vertices
     * @param elems : the matrix of elements - specified as a Nx4 matrix of vertex indices
     * @param material : describes the material properties of the elastic material
     */
    explicit ElasticMeshObject(const std::string& name, const VerticesMat& verts, const ElementsMat& elems, const ElasticMaterial& material);

    virtual std::string toString() const override;
    virtual std::string type() const override { return "ElasticMeshObject"; }

    const ElasticMaterial& material() { return _material; }

    /** Fixes all vertices in the mesh with the minimum y-value.
     * Used in setting up cantilever beam experiments
     */
    void fixVerticesWithMinY();

    /** Fixes all vertices in the mesh with the minimum z-value.
     * Used in setting up tissue experiments.
     */
    void fixVerticesWithMinZ();

    /** Adds a new vertex driver */
    void addVertexDriver(const VertexDriver& vd);

    /** Removes a vertex driver */
    void removeVertexDriver(const unsigned vertex);

    /** Stretches the mesh in each direction by a specified amount.
     * I.e. for an x_stretch of 2, the mesh will be stretched by a factor of 2x in the x direction
     * @param x_stretch : the factor by which to stretch (or compress) the mesh in the x direction
     * @param y_stretch : the factor by which to stretch (or compress) the mesh in the y direction
     * @param z_stretch : the factor by which to stretch (or compress) the mesh in the z direction
     */
    void stretch(const double x_stretch, const double y_stretch, const double z_stretch);

    /** Collapses all vertices to the mesh's minimum z
     * Used in stability experiments
     */
    void collapseToMinZ();

    /** Overrides setVertices in order to properly set size of _fixed_vertices
     * @param verts : the new vertices matrix
     */
    void setVertices(const VerticesMat& verts) override;

    /** Sets new elements for the tetrahedral.
     * @param elems : the new matrix of elements - specified as a Nx4 matrix of vertex indices
     */
    void setElements(const ElementsMat& elems);

    private:
    /** Helper function to load mesh information from a file.
     * Uses Assimp importer and exporter to convert files to .stl, and then uses gmsh to create a volume mesh from a surface mesh (if needed).
     * @param filename : the filename of the mesh file to open
     */
    void _loadMeshFromFile(const std::string& filename);
    
    /** Helper function to set the faces matrix from a matrix of elements.
     * For each tetrahedral element, there are 4 faces.
     */
    void _setFacesFromElements();

    protected:
    /** Nx4 element matrix */
    ElementsMat _elements;

    /** Keeps track of fixed vertices
     * If entry i is true, vertex i is fixed
     */
    Eigen::Vector<bool, -1> _fixed_vertices;

    /** Keeps track of vertex drivers */
    std::vector<VertexDriver> _vertex_drivers;

    /** Previous vertex positions */
    VerticesMat _x_prev;

    /** Per vertex velocity
     * Velocity is updated to be (x - x_prev) / dt
     */
    VerticesMat _v;
    
    /** Specifies the material properties */
    ElasticMaterial _material;
};

#endif // __ELASTIC_MESH_OBJECT_HPP