#include "solver/XPBDSolver.hpp"
#include "solver/Constraint.hpp"
#include "solver/ConstraintProjectorDecorator.hpp"
#include "simobject/XPBDMeshObject.hpp"

#include <chrono>

namespace Solver
{

XPBDSolver::XPBDSolver(XPBDMeshObject const* obj, unsigned num_iter, XPBDResidualPolicy residual_policy)
    : _obj(obj), _num_iter(num_iter), _residual_policy(residual_policy), _constraints_using_primary_residual(false), _num_constraints(0)
{
    _primary_residual.resize(3*_obj->numVertices());
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
    {
        _constraints_using_primary_residual = true;
        if (WithDistributedPrimaryResidual* wpr = dynamic_cast<WithDistributedPrimaryResidual*>(projector.get()))
        {
            wpr->setPrimaryResidual(_primary_residual.data());
        }
    }
        
    
    _constraint_projectors.push_back(std::move(projector));
}

void XPBDSolver::solve()
{
    _inertial_positions = _obj->vertices();

    // initialize all the constraints
    for (const auto& c : _constraint_projectors)
    {
        c->initialize();
    }

    for (unsigned i = 0; i < _num_iter; i++)
    {
        // if any of the constraint projections require the primary residual, we need to calculate it
        if (_constraints_using_primary_residual)
        {
            _calculatePrimaryResidual();
        }

        // iterate through the constraints and solve them
        _solveConstraints(_data.data());
    }

    // calculate the residual after this step if required by the residual policy
    if (_residual_policy == XPBDResidualPolicy::EVERY_SUBSTEP)
    {
        _calculatePrimaryResidual();
        _calculateConstraintResidual();
    }
}

void XPBDSolver::_calculatePrimaryResidual() 
{
    // add Mx
    for (unsigned i = 0; i < _obj->numVertices(); i++)
    {
        Eigen::Map<Eigen::Vector3d> Mx(_primary_residual.data() + 3*i);
        Mx = _obj->vertexMass(i) * (_obj->getVertex(i) - _inertial_positions.row(i).transpose());
    }

    // subtract delC*lambda
    for (const auto& proj : _constraint_projectors)
    {
        const std::vector<PositionReference>& positions = proj->positions();
        const std::vector<Constraint*>& constraints = proj->constraints();
        const std::vector<double>& lambda = proj->lambda();
        for (unsigned ci = 0; ci < constraints.size(); ci++)
        {
            double* delC = _data.data();
            double* additional_memory = delC + proj->numCoordinates();
            constraints[ci]->gradient(delC, additional_memory);
            for (unsigned pi = 0; pi < positions.size(); pi++)
            {
                if (_obj != positions[pi].obj)
                    continue;
                
                const unsigned v_ind = positions[pi].index;
                _primary_residual[3*v_ind]   -= delC[3*pi]   * lambda[ci];
                _primary_residual[3*v_ind+1] -= delC[3*pi+1] * lambda[ci];
                _primary_residual[3*v_ind+2] -= delC[3*pi+2] * lambda[ci];
            }
        }
        
    }
}

void XPBDSolver::_calculateConstraintResidual()
{
    // make sure the constraint residual is large enough (in case the number of constraints has changed)
    // (this will only reallocate memory if the new number of constraints > capacity of vector)
    _constraint_residual.resize(_num_constraints);

    // fill out the constraint residual using the projector constraintEquation() method
    unsigned constraint_index = 0;
    for (const auto& proj : _constraint_projectors)
    {
        proj->constraintEquation(_data.data(), _constraint_residual.data() + constraint_index);     // the result vector starts at the current constraint index
        constraint_index += proj->numConstraints();
    }
    
}

} // namespace Solver