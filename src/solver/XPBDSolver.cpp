#include "solver/XPBDSolver.hpp"
#include "solver/Constraint.hpp"
#include "solver/ConstraintDecorator.hpp"
#include "simobject/XPBDMeshObject.hpp"

namespace Solver
{

XPBDSolver::XPBDSolver(XPBDMeshObject const* obj, unsigned num_iter, XPBDResidualPolicy residual_policy)
    : _obj(obj), _num_iter(num_iter), _residual_policy(residual_policy)
{
    _primary_residual = std::make_unique<Eigen::VectorXd>(Eigen::VectorXd::Zero(3*_obj->numVertices()));

    _num_constraints = 0;
    for (const auto& proj : _obj->constraintProjectors())
    {
        _num_constraints += proj->numConstraints();
    }
    _constraint_residual = std::make_unique<Eigen::VectorXd>(Eigen::VectorXd::Zero(_num_constraints));

    // for (const auto& constraint : _obj->constraints())
    // {
    //     if (constraint->usesPrimaryResidual())
    //     {
    //         _constraints_using_primary_residual = true;
    //         if (WithPrimaryResidual* wpr = dynamic_cast<WithPrimaryResidual*>(constraint.get()))
    //         {
    //             wpr->setPrimaryResidual(_primary_residual.get());
    //         }
    //     }
    // }
}

void XPBDSolver::solve()
{
    _inertial_positions = _obj->vertices();
    for (const auto& c : _obj->constraintProjectors())
    {
        c->initialize();
    }

    for (unsigned i = 0; i < _num_iter; i++)
    {
        if (_constraints_using_primary_residual)
        {
            *_primary_residual = _calculatePrimaryResidual();
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

    const std::vector<std::unique_ptr<ConstraintProjector>>& projectors = _obj->constraintProjectors();
    // subtract delC*lambda
    for (const auto& proj : projectors)
    {
        const std::vector<PositionReference>& positions = proj->positions();
        const std::vector<Constraint*>& constraints = proj->constraints();
        const Eigen::VectorXd& lambda = proj->lambda();
        // const Eigen::VectorXd grad = constraint->gradient();
        for (unsigned ci = 0; ci < constraints.size(); ci++)
        {
            const Eigen::VectorXd& grad = constraints[ci]->gradient();
            for (unsigned pi = 0; pi < positions.size(); pi++)
            {
                if (_obj != positions[pi].obj)
                    continue;
                
                const unsigned v_ind = positions[pi].index;
                primary_residual(Eigen::seq(3*v_ind, 3*v_ind+2)) -= grad(Eigen::seq(3*pi, 3*pi+2)) * lambda(ci);
            }
        }
        
    }

    // std::cout << "pres:\n" << primary_residual << std::endl;

    return primary_residual;
}

Eigen::VectorXd XPBDSolver::_calculateConstraintResidual() const
{
    Eigen::VectorXd constraint_residual(_num_constraints);
    const std::vector<std::unique_ptr<ConstraintProjector>>& projectors = _obj->constraintProjectors();
    unsigned constraint_index = 0;
    for (unsigned i = 0; i < _obj->numConstraintProjectors(); i++)
    {
        const Eigen::VectorXd& lambda = projectors[i]->lambda();
        const Eigen::MatrixXd& alphaTilde = projectors[i]->alphaTilde();
        const std::vector<Constraint*> constraints = projectors[i]->constraints();
        for (unsigned j = 0; j < constraints.size(); j++)
        {
            constraint_residual(constraint_index) = constraints[j]->evaluate() + alphaTilde(j,j) * lambda(j);
            constraint_index++;
        }
        
    }

    // std::cout << "cres:\n" << constraint_residual << std::endl;

    return constraint_residual;
    
}

} // namespace Solver