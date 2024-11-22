#ifndef __CONSTRAINT_DECORATOR_HPP
#define __CONSTRAINT_DECORATOR_HPP

#include "solver/Constraint.hpp"
#include <memory>

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

class WithDamping : public ConstraintDecorator
{
    public:
    WithDamping(std::unique_ptr<Constraint> component, double gamma); 

    virtual bool usesDamping() const override { return true; }

    protected:
    virtual double _LHS(const ValueAndGradient& val_and_grad) const override;
    virtual double _RHS(const ValueAndGradient& val_and_grad) const override;

    private:
    double _damping_gamma;    
};

class WithPrimaryResidual : public ConstraintDecorator
{
    public:
    WithPrimaryResidual(std::unique_ptr<Constraint> component);

    void setPrimaryResidual(Eigen::VectorXd const* res_ptr) { _res_ptr = res_ptr; }
    virtual bool usesPrimaryResidual() const override { return true; }

    protected:
    virtual double _RHS(const ValueAndGradient& val_and_grad) const override;
    virtual PositionUpdate _getPositionUpdate(const unsigned position_index, const double dlam, const Eigen::VectorXd& grad) const override;

    private:
    Eigen::Vector3d _scaledResidual(const unsigned position_index) const;

    private:
    // std::shared_ptr<Eigen::VectorXd> _res_ptr;
    Eigen::VectorXd const* _res_ptr;
};

class FirstOrder : public ConstraintDecorator
{
    public:
    FirstOrder(std::unique_ptr<Constraint> component);

    virtual double alphaTilde() const override;

    virtual double positionInvMass(const unsigned position_index) const override;

    private:


    std::vector<FirstOrderXPBDMeshObject*> _fo_objs;
};

} // namespace Solver


#endif // __CONSTRAINT_DECORATOR_HPP