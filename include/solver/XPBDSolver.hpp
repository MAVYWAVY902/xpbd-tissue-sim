#ifndef __SOLVER_HPP
#define __SOLVER_HPP

#include "simobject/MeshObject.hpp"
#include "simobject/XPBDMeshObject.hpp"
#include "config/XPBDMeshObjectConfig.hpp"

#include <memory>

namespace Solver
{

class XPBDSolver
{
    public:
    explicit XPBDSolver(XPBDMeshObject const* obj, unsigned num_iter, XPBDResidualPolicy residual_policy);

    virtual ~XPBDSolver() = default;


    virtual void solve();

    Eigen::VectorXd primaryResidual() const { return *_primary_residual; };
    Eigen::VectorXd constraintResidual() const { return *_constraint_residual; };

    unsigned numIterations() const { return _num_iter; }


    protected:
    virtual void _solveConstraints() = 0;
    Eigen::VectorXd _calculatePrimaryResidual() const;
    Eigen::VectorXd _calculateConstraintResidual() const;

    XPBDMeshObject const* _obj;
    unsigned _num_iter;
    MeshObject::VerticesMat _inertial_positions;

    XPBDResidualPolicy _residual_policy;

    bool _constraints_using_primary_residual;

    std::unique_ptr<Eigen::VectorXd> _primary_residual;
    std::unique_ptr<Eigen::VectorXd> _constraint_residual;
    
};

} // namespace Solver

#endif // __SOLVER_HPP