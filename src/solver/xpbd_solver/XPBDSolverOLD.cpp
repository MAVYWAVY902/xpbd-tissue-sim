#include "solver/XPBDSolver.hpp"
#include "solver/Constraint.hpp"
#include "solver/ConstraintProjectorDecorator.hpp"
#include "simobject/XPBDMeshObject.hpp"

#include <chrono>

namespace Solver
{

XPBDSolver::XPBDSolver(Sim::XPBDMeshObject* obj, int num_iter, XPBDResidualPolicy residual_policy)
    : _obj(obj), _num_iter(num_iter), _residual_policy(residual_policy), _constraints_using_primary_residual(false), _num_constraints(0)
{
    _primary_residual.resize(3*_obj->mesh()->numVertices());
    _rigid_body_updates.resize(14); // 14 doubles is enough to store 2 rigid body updates (no more than 2 rigid bodies will be involved in a single constraint projection)
}

/** Overload ConstraintProjector creation explicitly for the combination of a Deviatoric and Hydrostatic constraint.
 * The CombinedNeohookeanConstraintProjector is specifically optimized for this combination.
 */
template <>
std::unique_ptr<ConstraintProjector> XPBDSolver::_createConstraintProjector<DeviatoricConstraint, HydrostaticConstraint>(Real dt, const ConstraintProjectorOptions& options, 
                                                                                                                        DeviatoricConstraint* dev_constraint, HydrostaticConstraint* hyd_constraint)
{
    std::vector<Constraint*> constraints_vec(2);
    constraints_vec[0] = dev_constraint;
    constraints_vec[1] = hyd_constraint;
    std::unique_ptr<CombinedNeohookeanConstraintProjector> projector = std::make_unique<CombinedNeohookeanConstraintProjector>(std::move(constraints_vec), dt);

    std::unique_ptr<ConstraintProjector> decorated_projector = _decorateConstraintProjector(std::move(projector), options);

    return decorated_projector;
}

std::unique_ptr<ConstraintProjector> XPBDSolver::_decorateConstraintProjector(std::unique_ptr<ConstraintProjector> projector, const ConstraintProjectorOptions& options)
{
    if (options.with_damping)
    {
        projector = std::make_unique<WithDamping>(std::move(projector), options.damping_gamma);        // wrap the ConstraintProjector with the WithDamping decorator
    }

    if (options.first_order)
    {
        projector = std::make_unique<FirstOrder>(std::move(projector));                         // wrap the ConstraintProjector with the FirstOrder decorator
    }

    if (options.with_residual)
    {
        projector = std::make_unique<WithDistributedPrimaryResidual>(std::move(projector));     // wrap the ConstraintProjector with the WithDistributedPRimaryResidual decorator
    }

    return projector;
}

void XPBDSolver::removeConstraintProjector(const int index)
{
    _num_constraints -= _constraint_projectors.at(index)->numConstraints();
    _constraint_projectors.at(index) = nullptr;
}

void XPBDSolver::solve()
{
    _inertial_positions = _obj->mesh()->vertices();

    // initialize all the constraints
    for (const auto& c : _constraint_projectors)
    {
        if (c)
            c->initialize();
    }

    for (int i = 0; i < _num_iter; i++)
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
    const Sim::FirstOrderXPBDMeshObject* fo_obj = dynamic_cast<const Sim::FirstOrderXPBDMeshObject*>(_obj);
    // add Mx
    for (int i = 0; i < _obj->mesh()->numVertices(); i++)
    {
        Eigen::Map<Vec3r> Mx(_primary_residual.data() + 3*i);

        // I don't like this - is there some way to not have to explicitly check for FirstOrder?
        if (fo_obj)
        {
            Mx = fo_obj->vertexDamping(i) * (_obj->mesh()->vertex(i) - _inertial_positions.col(i));
        }
        else
        {
            Mx = _obj->vertexMass(i) * (_obj->mesh()->vertex(i) - _inertial_positions.col(i));
        }
        
    }

    // subtract delC*lambda
    for (const auto& proj : _constraint_projectors)
    {
        if (!proj)
            continue;

        const std::vector<PositionReference>& positions = proj->positions();
        const std::vector<Constraint*>& constraints = proj->constraints();
        const std::vector<Real>& lambda = proj->lambda();
        for (size_t ci = 0; ci < constraints.size(); ci++)
        {
            Real* delC = _data.data();
            constraints[ci]->gradient(delC);
            for (size_t pi = 0; pi < positions.size(); pi++)
            {
                if (_obj != positions[pi].obj)
                    continue;
                
                const int v_ind = positions[pi].index;
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
    int constraint_index = 0;
    for (const auto& proj : _constraint_projectors)
    {
        if (!proj)
            continue;

        proj->constraintEquation(_data.data(), _constraint_residual.data() + constraint_index);     // the result vector starts at the current constraint index
        constraint_index += proj->numConstraints();
    }
    
}

} // namespace Solver