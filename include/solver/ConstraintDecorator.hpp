#ifndef __CONSTRAINT_DECORATOR_HPP
#define __CONSTRAINT_DECORATOR_HPP

#include "solver/Constraint.hpp"
#include "simobject/FirstOrderXPBDMeshObject.hpp"
#include <memory>
#include <iomanip>

namespace Solver
{

class ConstraintDecorator : public Constraint
{
    public:
    explicit ConstraintDecorator(std::unique_ptr<Constraint> component)
        : Constraint(*component), 
        _component(std::move(component))
    {
    }

    virtual void setLambda(const double new_lambda) override { _component->setLambda(new_lambda); _lambda = new_lambda; }

    /** Evaluates the current value of this constraint.
     * i.e. returns C(x)
     */
    virtual double evaluate() const override { return _component->evaluate(); }

    /** Returns the gradient of this constraint in vector form.
     * i.e. returns delC(x)
     */
    virtual Eigen::VectorXd gradient() const override { return _component->gradient(); }

    /** Returns the value and gradient of this constraint.
     * i.e. returns C(x) and delC(x) together.
     */
    virtual ValueAndGradient evaluateWithGradient() const override { return _component->evaluateWithGradient(); }

    protected:
    double _componentLHS(const ValueAndGradient& vg) const { return _component->_LHS(vg); }
    double _componentRHS(const ValueAndGradient& vg) const { return _component->_RHS(vg); }
    PositionUpdate _componentGetPositionUpdate(const unsigned position_index, const double dlam, const Eigen::VectorXd& grad) const
    {
        return _component->_getPositionUpdate(position_index, dlam, grad);
    }
    std::unique_ptr<Constraint> _component;
};


/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////


class WithDamping : public ConstraintDecorator
{ 
    public:
    WithDamping(std::unique_ptr<Constraint> component, double gamma)
        : ConstraintDecorator(std::move(component)), _damping_gamma(gamma)
    {
    }

    virtual bool usesDamping() const override { return true; }

    protected:
    double _LHS(const ValueAndGradient& val_and_grad) const
    {
        const double alpha_tilde = alphaTilde();
        const double delC_Minv_delC = _componentLHS(val_and_grad) - alpha_tilde;
        return (1 + _damping_gamma) * delC_Minv_delC + alpha_tilde;
    }

    double _RHS(const ValueAndGradient& val_and_grad) const
    {
        const double alpha_tilde = alphaTilde();

        const Eigen::VectorXd& grad = val_and_grad.second;
        double delC_x_prev = 0;
        for (unsigned i = 0; i < _positions.size(); i++)
        {
            delC_x_prev += grad(Eigen::seq(3*i, 3*i+2)).dot(_positions[i].position() - _positions[i].previousPosition());
        }

        double rhs = _componentRHS(val_and_grad) - _damping_gamma * delC_x_prev;
        return rhs;
    }

    private:
    double _damping_gamma;
};


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////


class WithPrimaryResidual : public ConstraintDecorator
{
    
    public:
    WithPrimaryResidual(std::unique_ptr<Constraint> component)
        : ConstraintDecorator(std::move(component))
    {
    }

    void setPrimaryResidual(Eigen::VectorXd const* res_ptr) { _res_ptr = res_ptr; }
    virtual bool usesPrimaryResidual() const override { return true; }

    protected:
    inline virtual double _RHS(const ValueAndGradient& val_and_grad) const
    {
        const Eigen::VectorXd& grad = val_and_grad.second;
        double delC_g = 0;
        for (unsigned i = 0; i < _positions.size(); i++)
        {
            const double inv_m = positionInvMass(i);
            delC_g += inv_m*(grad(Eigen::seq(3*i, 3*i+2)).dot(_scaledResidual(i)));
        }

        const double rhs = _componentRHS(val_and_grad) + delC_g;
        return rhs;
    }

    inline virtual PositionUpdate _getPositionUpdate(const unsigned position_index, const double dlam, const Eigen::VectorXd& grad) const
    {
        PositionUpdate position_update = _componentGetPositionUpdate(position_index, dlam, grad);
        position_update.second -= positionInvMass(position_index) * _scaledResidual(position_index);
        return position_update;
    }

    private:
    inline virtual Eigen::Vector3d _scaledResidual(const unsigned position_index) const
    {
        assert(_res_ptr);
        unsigned v_ind = _positions[position_index].index;
        const Eigen::Vector3d& g = (*_res_ptr)(Eigen::seq(3*v_ind,3*v_ind+2));

        // TODO: replace number of attached elements with number of "attached" constraints for more generality
        return g / (2*_positions[position_index].attachedElements());
    }

    private:
    Eigen::VectorXd const* _res_ptr;
};


////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////


class FirstOrder : public ConstraintDecorator
{
    public:
    FirstOrder(std::unique_ptr<Constraint> component)
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

    inline virtual double alphaTilde() const override
    {
        return _alpha / _dt;
    }

    inline virtual double positionInvMass(const unsigned position_index) const override
    {
        return _fo_objs[position_index]->vertexInvDamping(_positions[position_index].index);
    }

    private:
    std::vector<FirstOrderXPBDMeshObject*> _fo_objs;
};

} // namespace Solver


#endif // __CONSTRAINT_DECORATOR_HPP