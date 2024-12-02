#include "solver/XPBDSolver.hpp"
#include "solver/Constraint.hpp"
#include "solver/ConstraintProjectorDecorator.hpp"
#include "simobject/XPBDMeshObject.hpp"

namespace Solver
{

XPBDSolver::XPBDSolver(XPBDMeshObject const* obj, unsigned num_iter, XPBDResidualPolicy residual_policy)
    : _obj(obj), _num_iter(num_iter), _residual_policy(residual_policy), _constraints_using_primary_residual(false), _num_constraints(0)
{
    // TODO: fix primary and constraint residuals
    _primary_residual = std::make_unique<Eigen::VectorXd>(Eigen::VectorXd::Zero(3*_obj->numVertices()));
    _constraint_residual = std::make_unique<Eigen::VectorXd>(Eigen::VectorXd::Zero(1));
}

void XPBDSolver::addConstraintProjector(std::unique_ptr<ConstraintProjector> projector)
{
    // amount of pre-allocated memory required to perform the constraint(s) projection
    unsigned required_array_size = projector->memoryNeeded() / sizeof(double);
    // make sure that the data buffer of the _data vector is large enough to accomodate the new projector
    if (required_array_size > _data.size())
    {
        _data.resize(required_array_size);
    }

    // make sure that the data buffer of the coordinate updates vector is large enough to accomodate the new projector
    if (projector->numCoordinates() > _coordinate_updates.size())
    {
        _coordinate_updates.resize(projector->numCoordinates());
    }

    // increase the total number of constraints (needed for the constraint residual size)
    _num_constraints += projector->numConstraints();

    // check if primary residual is needed for constraint projection
    if (projector->usesPrimaryResidual())
        _constraints_using_primary_residual = true;
    
    _constraint_projectors.push_back(std::move(projector));
}

void XPBDSolver::solve()
{
    _inertial_positions = _obj->vertices();
    //for (const auto& c : _obj->constraintProjectors())
    for (const auto& c : _constraint_projectors)
    {
        c->initialize();
    }

    for (unsigned i = 0; i < _num_iter; i++)
    {
        // if any of the constraint projections require the primary residual, we need to calculate it
        if (_constraints_using_primary_residual)
        {
            *_primary_residual = _calculatePrimaryResidual();
        }

        // iterate through the constraints and solve them
        _solveConstraints(_data.data());
    }

    // calculate the residual after this step if required by the residual policy
    if (_residual_policy == XPBDResidualPolicy::EVERY_SUBSTEP)
    {
        std::cout << "computing residuals..." << std::endl;
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

    // subtract delC*lambda
    for (const auto& proj : _constraint_projectors)
    {
        const std::vector<PositionReference>& positions = proj->positions();
        const std::vector<Constraint*>& constraints = proj->constraints();
        const std::vector<double>& lambda = proj->lambda();
        for (unsigned ci = 0; ci < constraints.size(); ci++)
        {
            const Eigen::VectorXd& grad = constraints[ci]->gradient();
            for (unsigned pi = 0; pi < positions.size(); pi++)
            {
                if (_obj != positions[pi].obj)
                    continue;
                
                const unsigned v_ind = positions[pi].index;
                primary_residual(Eigen::seq(3*v_ind, 3*v_ind+2)) -= grad(Eigen::seq(3*pi, 3*pi+2)) * lambda[ci];
            }
        }
        
    }

    return primary_residual;
}

Eigen::VectorXd XPBDSolver::_calculateConstraintResidual() const
{
    std::cout << "calculating constraint residual..." << std::endl;
    Eigen::VectorXd constraint_residual(_num_constraints);
    unsigned constraint_index = 0;
    for (unsigned i = 0; i < _constraint_projectors.size(); i++)
    {
        const std::vector<double>& lambda = _constraint_projectors[i]->lambda();
        double* alpha_tilde = _data.data();
        _constraint_projectors[i]->alphaTilde(alpha_tilde);
        const std::vector<Constraint*>& constraints = _constraint_projectors[i]->constraints();
        for (unsigned j = 0; j < constraints.size(); j++)
        {
            constraint_residual(constraint_index) = constraints[j]->evaluate() + alpha_tilde[j] * lambda[j];
            constraint_index++;
        }
        
    }

    // std::cout << "cres:\n" << constraint_residual << std::endl;

    return constraint_residual;
    
}

} // namespace Solver