#ifndef __XPBD_MESH_OBJECT_HPP
#define __XPBD_MESH_OBJECT_HPP

#include "ElasticMeshObject.hpp"
#include "ElasticMaterial.hpp"
#include "config/XPBDMeshObjectConfig.hpp"

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
    // explicit XPBDMeshObject(const std::string& name, const YAML::Node& config);
    explicit XPBDMeshObject(const XPBDMeshObjectConfig* config);

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
    void update(const double dt, const double g_accel) override;

    double primaryResidual() { return _primary_residual; }
    double dynamicsResidual() { return _dynamics_residual; }
    double constraintResidual() { return _constraint_residual; }
    double volumeRatio() { return _vol_ratio; }

    unsigned numSolverIters() { return _num_iters; }
    std::string solveMode();
    
    private:
    /** Helper method to initialize upon instantiation */
    void _init();

    /** Precomputes static quantities, such as the Q matrix for each element and the volumes of each element */
    void _precomputeQuantities();

    /** Moves the vertices in the absence of constraints.
     * i.e. according to their current velocities and the forces applied to them
     * @param dt : the time step
     * @param g_accel : the acceleration due to gravity
     */
    void _movePositionsIntertially(const double dt, const double g_accel);

    /** Projects the elastic constraints onto the updated positions in a Gauss-Seidel fashion.
     * For XPBD, this is the hydrostatic constraint, C_h and the deviatoric constraint, C_d.
     * See the XPBD paper for more details.
     * This is the direct implementation of the method in the XPBD paper, and mirrors the code provided in the paper.
     * @param dt : the time step
    */
    void _projectConstraintsSequential(const double dt);

    /** Projects the elastic constraints in a RANDOM ORDER in a Gauss-Seidel fashion.
     * Identical to _projectConstraintsSequential but iterates through the elements in a random order.
     * @param dt : the time step
     */
    void _projectConstraintsSequentialRandomized(const double dt);

    /** Projects the elastic constraints by solving the C_h and C_d for a single tetrahedral element SIMULTANEOUSLY.
     * This ultimately involves solving a 2x2 system, resulting in a 2x1 dlambda with one term for the dlambda associated with C_h and the other associated with C_d.
     * @param dt : the time step
     */
    void _projectConstraintsSimultaneous(const double dt);
    
    /** Projects the elastic constraints but updates the x-positions after finding all the dlambdas - i.e. keeps x FIXED during the computation of lambdas.
     * @param dt : the time step
     */
    void _projectConstraintsConstantX(const double dt);

    /** Projects the elastic constraints, initializing the lambdas to a nonzero value depending on the constraint.
     * This is identical to _projectConstraintsSequential but with an initialization of the lambda vector.
     * @param dt : the time step
      */
    void _projectConstraintsSequentialInitLambda(const double dt);

    void _projectConstraintsSimultaneousInitLambda(const double dt);

    void _projectConstraintsRuckerFull(const double dt);

    void _projectConstraintsSplitDeviatoricSequential(const double dt);

    void _projectConstraintsSplitDeviatoricSimultaneous9(const double dt);

    void _projectConstraintsSplitDeviatoricSimultaneous10(const double dt);

    /** Computes the residuals for the equations of motion.
     * See equations 8 and 9 in XPBD (Muller and Macklin 2016)
     * 
     * M * (x^n+1 - x_inertial) - delC(x^n+1)^T * lam^n+1 = "primary residual"
     * C(x^n+1) + alpha * lam^n+1 = "constraint residual"
     */
    void _calculateResiduals(const double dt, const VerticesMat& inertial_positions, const Eigen::VectorXd& lambda_hs, const Eigen::VectorXd& lambda_ds);

    void _calculateResidualsSplitDeviatoric(const double dt, const VerticesMat& inertial_positions, const Eigen::MatrixXd& lambdas);

    void _calculateForces();

    void _calculateForcesSplitDeviatoric();

    /** Projects the collision constraints onto the updated positions. */
    void _projectCollisionConstraints(const double dt);

    /** Update the velocities based on the updated positions.
     * @param dt : the time step
     */
    void _updateVelocities(const double dt);

    inline void _computeF(const unsigned elem_index, Eigen::Matrix3d& X, Eigen::Matrix3d& F);

    inline double _computeDeviatoricConstraint(const Eigen::Matrix3d& F, const Eigen::Matrix3d& Q, Eigen::Matrix<double, 3, 4>& C_d_grads);

    inline double _computeHydrostaticConstraint(const Eigen::Matrix3d& F, const Eigen::Matrix3d& Q, Eigen::Matrix3d& F_cross, Eigen::Matrix<double, 3, 4>& C_h_grads);


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
    Eigen::VectorXd _vols;
    

    /** Per vertex mass
     * Each element contributes 1/4 of its mass to each of its vertices.
     * 
     * Computed once for each vertex, upon initializaation of the mesh.
     */
    Eigen::VectorXd _m;

    /** Loop variables
     * Allocate once at the beginning to avoid constant reallocation of resources each time step
     * each variable has _l prepended to indicate they are loop variables
    */
    Eigen::Matrix3d _lX, _lF, _lF_cross;
    Eigen::Matrix<double, 3, 4> _lC_h_grads, _lC_d_grads;
    // loop variables for the split deviatoric simultaneous method
    Eigen::VectorXd _lB, _lCA_inv, _lA_invB, _ldlam, _lb;
    Eigen::MatrixXd _lM;

    /** Number of Gauss-Seidel iterations
     */
    unsigned _num_iters;

    /** The solve mode of XPBD */
    XPBDSolveMode _solve_mode;

    /** Damping stiffness */
    double _damping_stiffness;

    /** Calculate the residuals every step */
    double _primary_residual;
    double _constraint_residual;
    double _dynamics_residual;
    double _vol_ratio;

    /** For the initializing lambda method */
    Eigen::VectorXd _initial_lambda_ds;
    Eigen::VectorXd _initial_lambda_hs;

    /** For the Rucker method */
    std::vector<std::vector<std::pair<unsigned, unsigned> > > _constraints_per_position;

    /** For the Split Deviatoric Simultaneous method */
    std::vector<Eigen::Matrix3d> _A_inv;





        
};

#endif // __XPBD_MESH_OBJECT_HPP