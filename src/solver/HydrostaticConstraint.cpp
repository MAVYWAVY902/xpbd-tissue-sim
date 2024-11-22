#include "solver/HydrostaticConstraint.hpp"
#include "simobject/XPBDMeshObject.hpp"

namespace Solver
{


HydrostaticConstraint::HydrostaticConstraint(const double dt, XPBDMeshObject* obj, unsigned v1, unsigned v2, unsigned v3, unsigned v4)
    : ElementConstraint(dt, obj, v1, v2, v3, v4)
{
    _alpha = 1/(obj->material().lambda() * _volume);
    _gamma = obj->material().mu() / obj->material().lambda();
}

double HydrostaticConstraint::evaluate() const
{
    return _evaluate(_computeF());
}

Eigen::VectorXd HydrostaticConstraint::gradient() const
{
    return _gradient(_computeF());
    
}

Constraint::ValueAndGradient HydrostaticConstraint::evaluateWithGradient() const
{
    const Eigen::Matrix3d F = _computeF();
    return ValueAndGradient(_evaluate(F), _gradient(F));
}

double HydrostaticConstraint::_evaluate(const Eigen::Matrix3d& F) const
{
    return F.determinant() - (1 + _gamma);
}

Eigen::VectorXd HydrostaticConstraint::_gradient(const Eigen::Matrix3d& F) const
{
    Eigen::Matrix3d F_cross, prod;
    F_cross.col(0) = F.col(1).cross(F.col(2));
    F_cross.col(1) = F.col(2).cross(F.col(0));
    F_cross.col(2) = F.col(0).cross(F.col(1));

    prod = F_cross * _Q.transpose();

    Eigen::VectorXd C_h_grads(12);
    C_h_grads(Eigen::seq(0,2)) = prod.col(0);
    C_h_grads(Eigen::seq(3,5)) = prod.col(1);
    C_h_grads(Eigen::seq(6,8)) = prod.col(2);
    C_h_grads(Eigen::seq(9,11)) = -prod.col(0) - prod.col(1) - prod.col(2);

    return C_h_grads;
}

} // namespace Solver