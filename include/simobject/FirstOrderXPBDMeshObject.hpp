#ifndef __FIRST_ORDER_XPBD_MESH_OBJECT_HPP
#define __FIRST_ORDER_XPBD_MESH_OBJECT_HPP

#include "ElasticMeshObject.hpp"
#include "ElasticMaterial.hpp"
#include "config/FirstOrderXPBDMeshObjectConfig.hpp"

/** A class for solving the dynamics of elastic, highly deformable materials with the XPBD method described in
 *  "A Constraint-based Formulation of Stable Neo-Hookean Materials" by Macklin and Muller (2021).
 *  Refer to the paper and preceding papers for details on the XPBD approach.
 */
class FirstOrderXPBDMeshObject : public ElasticMeshObject
{
    public:
    /** Creates a new XPBDMeshObject from a YAML config node
     * @param name : the name of the new XPBDMeshObject
     * @param config : the YAML node dictionary describing the parameters for the new XPBDMeshObject
     */
    explicit FirstOrderXPBDMeshObject(const FirstOrderXPBDMeshObjectConfig* config);

    virtual std::string toString() const override;
    virtual std::string type() const override { return "FirstOrderXPBDMeshObject"; }

    /** Updates the mesh based on a time step 
     * @param dt : the time delta since the last update
    */
    void update(const double dt, const double g_accel) override;

    double primaryResidual() { return _primary_residual_rms; }
    double dynamicsResidual() { return _dynamics_residual_rms; }
    double constraintResidual() { return _constraint_residual_rms; }
    double volumeRatio() { return _vol_ratio; }

    unsigned numSolverIters() { return _num_iters; }
    std::string solveMode() const;
    
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

    void _projectConstraintsSimultaneous(const double dt);

    void _projectConstraintsSimultaneousJacobi(const double dt);
    
    void _projectConstraintsSimultaneousConvergentJacobi(const double dt);

    /** Computes the residuals for the equations of motion.
     * See equations 8 and 9 in XPBD (Muller and Macklin 2016)
     * 
     * M * (x^n+1 - x_inertial) - delC(x^n+1)^T * lam^n+1 = "primary residual"
     * C(x^n+1) + alpha * lam^n+1 = "constraint residual"
     */
    void _calculateResiduals(const double dt, const VerticesMat& inertial_positions, const Eigen::VectorXd& lambda_hs, const Eigen::VectorXd& lambda_ds);

    void _calculateForces();

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

    /** Per vertex volume
     * The total volume of all elements attached to a given node.
     * 
     * Computed once for each vertex, upon initialization of the mesh.
     */
    Eigen::VectorXd _v_volume;

    /** Loop variables
     * Allocate once at the beginning to avoid constant reallocation of resources each time step
     * each variable has _l prepended to indicate they are loop variables
    */
    Eigen::Matrix3d _lX, _lF, _lF_cross;
    Eigen::Matrix<double, 3, 4> _lC_h_grads, _lC_d_grads;

    /** Number of Gauss-Seidel iterations
     */
    unsigned _num_iters;

    /** The solve mode of XPBD */
    FirstOrderXPBDSolveMode _solve_mode;

    /** Residual policy of XPBD - i.e. when to calculate the residual */
    FirstOrderXPBDResidualPolicy _residual_policy;

    /** Calculate the residuals every step */
    double _primary_residual_rms;
    double _constraint_residual_rms;
    double _dynamics_residual_rms;
    double _vol_ratio;
    VerticesMat _primary_residual;

    double _damping_multiplier;
    double _jacobi_scaling;

    // keeps track of the number of elements that share position i
    Eigen::VectorXi _num_elements_with_position;





        
};

#endif // __FIRST_ORDER_XPBD_MESH_OBJECT_HPP