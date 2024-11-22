#include "solver/XPBDSolver.hpp"
#include "solver/Constraint.hpp"
#include "simobject/XPBDMeshObject.hpp"

namespace Solver
{

XPBDSolver::XPBDSolver(XPBDMeshObject const* obj, unsigned num_iter, XPBDResidualPolicy residual_policy)
    : _obj(obj), _num_iter(num_iter), _residual_policy(residual_policy)
{
    _primary_residual = std::make_unique<Eigen::VectorXd>(Eigen::VectorXd::Zero(3*_obj->numVertices()));
    _constraint_residual = std::make_unique<Eigen::VectorXd>(Eigen::VectorXd::Zero(_obj->numConstraints()));
}

void XPBDSolver::solve()
{
    _inertial_positions = _obj->vertices();
    for (unsigned i = 0; i < _num_iter; i++)
    {
        if (_constraints_using_primary_residual)
        {
            *_primary_residual = _calculatePrimaryResidual();
        }

        for (const auto& c : _obj->constraints())
        {
            c->initialize();
        }

        _solveConstraints();
    }

    if (_residual_policy == XPBDResidualPolicy::EVERY_SUBSTEP)
    {
        *_primary_residual = _calculatePrimaryResidual();
        *_constraint_residual = _calculateConstraintResidual();
    }
}

Eigen::VectorXd XPBDSolver::_calculatePrimaryResidual() const
{
    Eigen::VectorXd primary_residual(3*_obj->numVertices());
    // add Mx
    for (unsigned i = 0; i < _obj->numVertices(); i++)
    {
        primary_residual(Eigen::seq(3*i, 3*i+2)) = _obj->vertexMass(i) * (_obj->getVertex(i) - _inertial_positions.row(i).transpose());  
    }

    const std::vector<std::unique_ptr<Constraint>>& constraints = _obj->constraints();
    // subtract delC*lambda
    for (const auto& constraint : constraints)
    {
        const std::vector<PositionReference>& positions = constraint->positions();
        const Eigen::VectorXd grad = constraint->gradient();
        for (unsigned i = 0; i < positions.size(); i++)
        {
            if (_obj != positions[i].obj)
                continue;
            
            const unsigned v_ind = positions[i].index;
            primary_residual(Eigen::seq(3*v_ind, 3*v_ind+2)) -= grad(Eigen::seq(3*i, 3*i+2)) * constraint->lambda();
        }
    }

    return primary_residual;
}

Eigen::VectorXd XPBDSolver::_calculateConstraintResidual() const
{
    Eigen::VectorXd constraint_residual(_obj->numConstraints());
    const std::vector<std::unique_ptr<Constraint>>& constraints = _obj->constraints();
    for (unsigned i = 0; i < _obj->numConstraints(); i++)
    {
        constraint_residual(i) = constraints[i]->evaluate() - constraints[i]->alphaTilde() * constraints[i]->lambda();
    }

    return constraint_residual;
    
}

} // namespace Solver