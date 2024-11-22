#include "solver/DeviatoricConstraint.hpp"
#include "simobject/XPBDMeshObject.hpp"

namespace Solver
{

DeviatoricConstraint::DeviatoricConstraint(const double dt, XPBDMeshObject* obj, unsigned v1, unsigned v2, unsigned v3, unsigned v4)
    : ElementConstraint(dt, obj, v1, v2, v3, v4)
{
    _alpha = 1/(obj->material().mu() * _volume);
}

double DeviatoricConstraint::evaluate() const
{
    return _evaluate(_computeF());
}

Eigen::VectorXd DeviatoricConstraint::gradient() const
{
    const Eigen::Matrix3d F = _computeF();
    const double C = _evaluate(F);

    return _gradient(F, C);
    
}

Constraint::ValueAndGradient DeviatoricConstraint::evaluateWithGradient() const
{
    const Eigen::Matrix3d F = _computeF();
    const double C = _evaluate(F);
    return ValueAndGradient(C, _gradient(F, C));
}

double DeviatoricConstraint::_evaluate(const Eigen::Matrix3d& F) const
{
    return std::sqrt(F.col(0).squaredNorm() + F.col(1).squaredNorm() + F.col(2).squaredNorm());
}

Eigen::VectorXd DeviatoricConstraint::_gradient(const Eigen::Matrix3d& F, const double C) const
{
    Eigen::Matrix3d prod = 1/C * F * _Q.transpose();

    Eigen::VectorXd grads(12);
    grads(Eigen::seq(0,2)) = prod.col(0);
    grads(Eigen::seq(3,5)) = prod.col(1);
    grads(Eigen::seq(6,8)) = prod.col(2);
    grads(Eigen::seq(9,11)) = -prod.col(0) - prod.col(1) - prod.col(2);

    return grads;
}

} // namespace Solver