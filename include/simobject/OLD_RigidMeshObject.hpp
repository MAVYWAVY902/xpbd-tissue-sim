#ifndef __RIGID_MESH_OBJECT_HPP
#define __RIGID_MESH_OBJECT_HPP

#include "simobject/MeshObject.hpp"
#include "config/RigidMeshObjectConfig.hpp"

/** A class for representing rigid body meshes.
 * These meshes are just run-of-the-mill surface meshes loaded from .obj or .stl files and do not deform.
 * It uses Assimp to load vertices and faces from file.
 */
class RigidMeshObject : public MeshObject
{

    public:
    /** Creates an empty RigidMeshObject with a specified name.
     * @param name : the name of the new RigidMeshObject
     */
    explicit RigidMeshObject(const std::string& name);

    /** Creates a RigidMeshObject from a YAML config node.
     * @param name : the name of the new RigidMeshObject
     * @param config : the YAML config node to load parameters from
     */
    // explicit RigidMeshObject(const std::string& name, const YAML::Node& config);
    explicit RigidMeshObject(const RigidMeshObjectConfig* config);

    /** Creates a RigidMeshObject by loading from a mesh file.
     * @param name : the name of the new RigidMeshObject
     * @param filename : the name of the file to load mesh data from. Loading uses Assimp, so the file must be of a format that Assimp can handle.
     */
    explicit RigidMeshObject(const std::string& name, const std::string& filename);

    /** Creates a RigidMeshObject directly from vertices and faces.
     * @param name : the name of the new RigidMeshObject
     * @param verts : the vertices matrix
     * @param faces : the faces matrix - Nx3 matrix of unsigned vertex indices
     */
    explicit RigidMeshObject(const std::string& name, const VerticesMat& verts, const FacesMat& faces);

    virtual std::string type() const override { return "RigidMeshObject"; }

    virtual VerticesMat velocities() const override { return VerticesMat::Zero(_vertices.rows(), 3); }

    virtual void setup() override;

    /** Updates the object for a given time step.
     * @param dt : the time step
     */
    virtual void update() override;

    /** Creates a primitive geometry based on the specified primitive type.
     * No other parameters are taken - use successive resize() and moveTo() calls to manipulate the geometry.
     */
    void createPrimitiveGeometry(const RigidMeshPrimitiveType primitive_type);

    /** Creates a square, 1x1, XY-planar geometry for this RigidMeshObject.
     * No parameters are taken - use successive resize() and moveTo() calls to manipulate the geometry.
     */
    void createPlaneGeometry();

    /** Creates a square, XY-planar geometry for this RigidMeshObject.
     * @param center_pos : the center position of the planar square
     * @param size : the size of the planar square
     */
    void createPlaneGeometry(const Eigen::Vector3d& center_pos, const double size);

    private:
    /** Helper function to load a mesh from file
     * @param filename : the file to load from using Assimp
     */
    void _loadMeshFromFile(const std::string& filename);

    protected:
    /** Velocity vector for the rigid body */
    Eigen::Vector3d _v;


};

#endif // __RIGID_MESH_OBJECT_HPP