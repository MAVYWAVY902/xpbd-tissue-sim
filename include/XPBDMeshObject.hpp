#ifndef __XPBD_MESH_OBJECT_HPP
#define __XPBD_MESH_OBJECT_HPP

#include "ElasticMeshObject.hpp"
#include "ElasticMaterial.hpp"

/** A class for solving the dynamics of elastic, highly deformable materials with the XPBD method described in
 *  "A Constraint-based Formulation of Stable Neo-Hookean Materials" by Macklin and Muller (2021).
 *  Refer to the paper and preceding papers for details on the XPBD approach.
 */
class XPBDMeshObject : public ElasticMeshObject
{
    public:
    /** Creates a new XPBDMeshObject from a YAML config node
     * @param name : the name of the new XPBDMeshObject
     * @param config : the YAML node dictionary describing the parameters for the new XPBDMeshObject
     */
    explicit XPBDMeshObject(const std::string& name, const YAML::Node& config);

    /** Creates a new XPBDMeshObject from a mesh file
     * @param name : the name of the new XPBDMeshObject
     * @param filename : the filename to load mesh data from. Does not have to be a .msh file, any .obj or .stl file will be volumetrically meshed using gmsh.
     * @param material : describes the elastic material properties of the object
     */
    explicit XPBDMeshObject(const std::string& name, const std::string& filename, const ElasticMaterial& material);

    /** Creates a new XPBDMeshObject directly from vertices and elements
     * @param name : the name of the new XPBDMeshObject
     * @param verts : the matrix of vertices
     * @param elems : the matrix of elements - specified as a Nx4 matrix of vertex indices
     * @param material : describes the material properties of the elastic material
     */
    explicit XPBDMeshObject(const std::string& name, const VerticesMat& verts, const ElementsMat& elems, const ElasticMaterial& material = ElasticMaterial::RUBBER());

    /** Updates the mesh based on a time step 
     * @param dt : the time delta since the last update
    */
    void update(const double dt) override;
    
    private:
    /** Helper method to initialize upon instantiation */
    void _init();

    /** Precomputes static quantities, such as the Q matrix for each element and the volumes of each element */
    void _precomputeQuantities();

    /** Moves the vertices in the absence of constraints.
     * i.e. according to their current velocities and the forces applied to them
     * @param dt : the time step
     */
    void _movePositionsIntertially(const double dt);

    /** Projects the elastic constraints onto the updated positions in a Gauss-Seidel fashion.
     * For XPBD, this is the hydrostatic constraint, C_h and the deviatoric constraint, C_d.
     * See the XPBD paper for more details.
     * @param dt : the time step
    */
    void _projectConstraints(const double dt);

    /** Projects the collision constraints onto the updated positions. */
    void _projectCollisionConstraints();

    /** Update the velocities based on the updated positions.
     * @param dt : the time step
     */
    void _updateVelocities(const double dt);


    /** Per element inverse reference shape matrix (Q)
     * Used in determining F for each element through the the equation
     *      F = xQ
     * where x is the deformed shape matrix found from the current deformed positions of the element.
     * 
     * We compute Q once for each element, upon initialization of the mesh.
     *  
     * For more information, see Sifakis 2012 SIGGRAPH Course Notes (Q is notated by D_m^-1)
     */
    std::vector<Eigen::Matrix3d> _Q;

    /** Per element initial volume
     * 
     * Computed once for each element, upon initialization of the mesh.
     */
    std::vector<double> _vols;

    /** Per vertex mass
     * Each element contributes 1/4 of its mass to each of its vertices.
     * 
     * Computed once for each vertex, upon initializaation of the mesh.
     */
    std::vector<double> _m;

    /** Previous vertex positions */
    VerticesMat _x_prev;

    /** Number of Gauss-Seidel iterations
     * For now, just a hard-coded constant value.
     */
    const unsigned _num_iters = 1;




        
};

#endif // __XPBD_MESH_OBJECT_HPP