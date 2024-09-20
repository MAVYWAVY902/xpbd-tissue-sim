#ifndef __FAST_FEM_MESH_OBJECT_HPP
#define __FAST_FEM_MESH_OBJECT_HPP

#include "ElasticMeshObject.hpp"
#include "FastFEMMeshObjectConfig.hpp"

#include <Eigen/Sparse>


class FastFEMMeshObject : public ElasticMeshObject
{
    public:
    
    explicit FastFEMMeshObject(const FastFEMMeshObjectConfig* config);

    virtual std::string toString() const override;
    virtual std::string type() const override { return "FastFEMMeshObject"; }

    void update(const double dt, const double g_accel) override;

    private:
    
    void _init();

    void _precomputeQuantities();

    protected:

    void _movePositionsInertially(const double g_accel);
    void _solveOptimizationProblem();
    void _solveVolumeConstraints();
    void _solveCollisionConstraints();
    void _updateVelocities();

    inline void _computeF(const unsigned elem_index, Eigen::Matrix3d& X, Eigen::Matrix3d& F);
    inline void _computeAPD(const Eigen::Matrix3d& F, Eigen::Vector4d& q);
    inline double _computeHydrostaticConstraint(const Eigen::Matrix3d& F, const Eigen::Matrix3d& Q, Eigen::Matrix3d& F_cross, Eigen::Matrix<double, 3, 4>& C_h_grads);

    inline Eigen::Vector4d _quaternionMultiply(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2);
    inline void _quaternionToRotationMatrix(const Eigen::Vector4d& q, Eigen::Matrix3d& R);

    Eigen::SparseMatrix<double, Eigen::ColMajor> _matL;
    Eigen::SparseMatrix<double, Eigen::ColMajor> _matLT;
    Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, int> _perm;
    Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, int> _perm_inv;

    Eigen::VectorXd _RHS, _RHS_perm;
    Eigen::SparseMatrix<double, Eigen::ColMajor> _LHS;

    std::vector<Eigen::Matrix<double, 3, 4> > _Dt;
    Eigen::VectorXd _K_vec;
    std::vector<Eigen::Vector4d> _quaternions;

    Eigen::VectorXd _masses;
    Eigen::VectorXd _rest_volumes;

    std::vector<Eigen::Matrix3d> _Q;

    // loop variables
    Eigen::Matrix3d _lX, _lF, _lF_cross, _lR;
    Eigen::Matrix<double, 3, 4> _lC_h_grads, _lC_d_grads;


};

#endif // __FAST_FEM_MESH_OBJECT_HPP