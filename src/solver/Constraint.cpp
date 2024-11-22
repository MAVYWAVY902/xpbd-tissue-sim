#include "solver/Constraint.hpp"
#include "simobject/XPBDMeshObject.hpp"

namespace Solver
{

/** Helper function to get the vertex position. */
Eigen::Vector3d PositionReference::position() const
{
    return obj->getVertex(index);
}

Eigen::Vector3d PositionReference::previousPosition() const
{
    return obj->vertexPreviousPosition(index);
}

/** Helper function to get the vertex velocity. */
Eigen::Vector3d PositionReference::velocity() const
{
    return obj->vertexVelocity(index);
}

/** Helper function to get the vertex mass. */
double PositionReference::mass() const
{
    return obj->vertexMass(index);
}

/** Helper function to get number of attached elements to the vertex. */
unsigned PositionReference::attachedElements() const
{
    return obj->vertexAttachedElements(index);
}



/////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////



Constraint::Constraint(const double dt)
    : _dt(dt), _alpha(0), _lambda(0)
{

}

void Constraint::initialize()
{
    _lambda = 0;
}

std::vector<Constraint::PositionUpdate> Constraint::project()
{
    const ValueAndGradient val_and_grad = evaluateWithGradient();
    const double dlam = _RHS(val_and_grad) / _LHS(val_and_grad);

    std::vector<PositionUpdate> position_updates(_positions.size());
    for (unsigned i = 0; i < _positions.size(); i++)
    {
        position_updates.at(i) = std::move(_getPositionUpdate(i, dlam, val_and_grad.second));
    }

    _lambda += dlam;

    return position_updates;
}

double Constraint::_LHS(const ValueAndGradient& val_and_grad) const
{
    const double alpha_tilde = alphaTilde();
    const Eigen::VectorXd& grads = val_and_grad.second;
    double lhs = alpha_tilde;
    for (unsigned i = 0; i < _positions.size(); i++)
    {
        const double inv_m = positionInvMass(i);
        lhs += inv_m * grads(Eigen::seq(3*i, 3*i+2)).squaredNorm();
    }

    return lhs;
}

double Constraint::_RHS(const ValueAndGradient& val_and_grad) const
{
    const double alpha_tilde = alphaTilde();
    const double C = val_and_grad.first;
    double rhs = -C - alpha_tilde * _lambda;

    return rhs;
}

Constraint::PositionUpdate Constraint::_getPositionUpdate(const unsigned position_index, const double dlam, const Eigen::VectorXd& grads) const
{
    PositionUpdate position_update;
    position_update.first = _positions[position_index];
    position_update.second = positionInvMass(position_index) * dlam * grads(Eigen::seq(3*position_index, 3*position_index+2));
    return position_update;
}

} // namespace Solver