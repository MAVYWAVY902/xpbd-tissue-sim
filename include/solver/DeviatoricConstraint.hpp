#ifndef __DEVIATORIC_CONSTRAINT_HPP
#define __DEVIATORIC_CONSTRAINT_HPP

#include "solver/ElementConstraint.hpp"

namespace Solver
{

class DeviatoricConstraint : public ElementConstraint
{
    public:
    explicit DeviatoricConstraint(const double dt, XPBDMeshObject* obj, unsigned v1, unsigned v2, unsigned v3, unsigned v4);

    virtual double evaluate() const override;
    virtual Eigen::VectorXd gradient() const override;
    virtual ValueAndGradient evaluateWithGradient() const override;

    private:
    Eigen::VectorXd _gradient(const Eigen::Matrix3d& F, const double C) const;
    double _evaluate(const Eigen::Matrix3d& F) const;

};

} // namespace Solver

#endif // __DEVIATORIC_CONSTRAINT_HPP