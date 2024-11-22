#include "solver/ConstraintDecorator.hpp"
#include "simobject/FirstOrderXPBDMeshObject.hpp"

namespace Solver
{

WithDamping::WithDamping(std::unique_ptr<Constraint> component, double gamma)
    : ConstraintDecorator(std::move(component)), _damping_gamma(gamma)
{
}

double WithDamping::_LHS(const ValueAndGradient& val_and_grad) const
{
    const double alpha_tilde = alphaTilde();
    const double delC_Minv_delC = _componentLHS(val_and_grad) - alpha_tilde;
    return (1 + _damping_gamma) * delC_Minv_delC + alpha_tilde;
}

double WithDamping::_RHS(const ValueAndGradient& val_and_grad) const
{
    const double alpha_tilde = alphaTilde();

    const Eigen::VectorXd& grad = val_and_grad.second;
    double delC_x_prev = 0;
    for (unsigned i = 0; i < _positions.size(); i++)
    {
        delC_x_prev += grad(Eigen::seq(3*i, 3*i+2)).dot(_positions[i].position() - _positions[i].previousPosition());
    }

    return _componentRHS(val_and_grad) - _damping_gamma * delC_x_prev;
}


/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////

WithPrimaryResidual::WithPrimaryResidual(std::unique_ptr<Constraint> component)
    : ConstraintDecorator(std::move(component))
{
}

double WithPrimaryResidual::_RHS(const ValueAndGradient& val_and_grad) const
{
    const Eigen::VectorXd& grad = val_and_grad.second;
    double delC_g = 0;
    for (unsigned i = 0; i < _positions.size(); i++)
    {
        const double inv_m = positionInvMass(i);
        delC_g += inv_m*(grad(Eigen::seq(3*i, 3*i+2)).dot(_scaledResidual(i)));
    }

    return _componentRHS(val_and_grad) + delC_g;
}

Constraint::PositionUpdate WithPrimaryResidual::_getPositionUpdate(const unsigned position_index, const double dlam, const Eigen::VectorXd& grad) const
{
    PositionUpdate position_update = _componentGetPositionUpdate(position_index, dlam, grad);
    position_update.second -= positionInvMass(position_index) * _scaledResidual(position_index);
    return position_update;
}

Eigen::Vector3d WithPrimaryResidual::_scaledResidual(const unsigned position_index) const
{
    assert(_res_ptr);
    unsigned v_ind = _positions[position_index].index;
    const Eigen::Vector3d& g = (*_res_ptr)(Eigen::seq(3*v_ind,3*v_ind+2));
    // TODO: replace number of attached elements with number of "attached" constraints for more generality
    return g / (2*_positions[position_index].attachedElements());
}


/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////

FirstOrder::FirstOrder(std::unique_ptr<Constraint> component)
    : ConstraintDecorator(std::move(component))
{
    // make sure all positions are part of FirstOrderXPBDMeshObject, and store their objs for future use
    const std::vector<PositionReference>& positions = _component->positions();
    _fo_objs.resize(positions.size());
    for (unsigned i = 0; i < positions.size(); i++)
    {
        if (FirstOrderXPBDMeshObject* fo_obj = dynamic_cast<FirstOrderXPBDMeshObject*>(positions[i].obj))
        {
            _fo_objs.at(i) = fo_obj;
        }
        else
        {
            std::cout << positions[i].obj->name() << " of type " << positions[i].obj->type() << " is not a FirstOrderXPBDMeshObject!" << std::endl;
            assert(0);
        }
    }
}

double FirstOrder::alphaTilde() const
{
    return _alpha / _dt;
}

double FirstOrder::positionInvMass(const unsigned position_index) const
{
    return 1.0/_fo_objs[position_index]->vertexDamping(_positions[position_index].index);
}

} // namespace Solver