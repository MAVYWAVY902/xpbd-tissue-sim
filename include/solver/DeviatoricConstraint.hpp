#ifndef __DEVIATORIC_CONSTRAINT_HPP
#define __DEVIATORIC_CONSTRAINT_HPP

#include "solver/ElementConstraint.hpp"

namespace Solver
{

class DeviatoricConstraint : public ElementConstraint
{
    // public:
    // explicit DeviatoricConstraint(const double dt, XPBDMeshObject* obj, unsigned v1, unsigned v2, unsigned v3, unsigned v4);

    // inline virtual double evaluate() const override;
    // inline virtual Eigen::VectorXd gradient() const override;
    // inline virtual ValueAndGradient evaluateWithGradient() const override;

    // private:
    // inline Eigen::VectorXd _gradient(const Eigen::Matrix3d& F, const double C) const;
    // inline double _evaluate(const Eigen::Matrix3d& F) const;

    public:
    DeviatoricConstraint(const double dt, XPBDMeshObject* obj, unsigned v1, unsigned v2, unsigned v3, unsigned v4)
        : ElementConstraint(dt, obj, v1, v2, v3, v4)
    {
        _alpha = 1/(obj->material().mu() * _volume);
    }

    inline double evaluate() const
    {
        return _evaluate(_computeF());
    }

    inline Eigen::VectorXd gradient() const
    {
        const Eigen::Matrix3d F = _computeF();
        const double C = _evaluate(F);

        return _gradient(F, C);
        
    }

    inline Constraint::ValueAndGradient evaluateWithGradient() const
    {
        const Eigen::Matrix3d F = _computeF();
        const double C = _evaluate(F);
        return ValueAndGradient(C, _gradient(F, C));
    }

    private:
    inline double _evaluate(const Eigen::Matrix3d& F) const
    {
        return std::sqrt(F.col(0).squaredNorm() + F.col(1).squaredNorm() + F.col(2).squaredNorm());
    }

    inline Eigen::VectorXd _gradient(const Eigen::Matrix3d& F, const double C) const
    {
        Eigen::Matrix3d prod = 1/C * F * _Q.transpose();

        Eigen::VectorXd grads(12);
        grads(Eigen::seq(0,2)) = prod.col(0);
        grads(Eigen::seq(3,5)) = prod.col(1);
        grads(Eigen::seq(6,8)) = prod.col(2);
        grads(Eigen::seq(9,11)) = -prod.col(0) - prod.col(1) - prod.col(2);

        return grads;
    }

};

} // namespace Solver

#endif // __DEVIATORIC_CONSTRAINT_HPP