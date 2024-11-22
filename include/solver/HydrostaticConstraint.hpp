#ifndef __HYDROSTATIC_CONSTRAINT_HPP
#define __HYDROSTATIC_CONSTRAINT_HPP

#include "solver/ElementConstraint.hpp"

namespace Solver
{

class HydrostaticConstraint : public virtual ElementConstraint
{
    // public:
    // explicit HydrostaticConstraint(const double dt, XPBDMeshObject* obj, unsigned v1, unsigned v2, unsigned v3, unsigned v4);

    // inline virtual double evaluate() const override;
    // inline virtual Eigen::VectorXd gradient() const override;
    // inline virtual ValueAndGradient evaluateWithGradient() const override;

    // private:
    // inline Eigen::VectorXd _gradient(const Eigen::Matrix3d& F) const;
    // inline double _evaluate(const Eigen::Matrix3d& F) const;

    // protected:
    // double _gamma;

    public:
    HydrostaticConstraint(const double dt, XPBDMeshObject* obj, unsigned v1, unsigned v2, unsigned v3, unsigned v4)
        : ElementConstraint(dt, obj, v1, v2, v3, v4)
    {
        _alpha = 1/(obj->material().lambda() * _volume);
        _gamma = obj->material().mu() / obj->material().lambda();
    }

    inline double evaluate() const
    {
        return _evaluate(_computeF());
    }

    inline Eigen::VectorXd gradient() const
    {
        return _gradient(_computeF());
        
    }

    inline Constraint::ValueAndGradient evaluateWithGradient() const
    {
        const Eigen::Matrix3d F = _computeF();
        return ValueAndGradient(_evaluate(F), _gradient(F));
    }

    private:
    inline double _evaluate(const Eigen::Matrix3d& F) const
    {
        return F.determinant() - (1 + _gamma);
    }

    inline Eigen::VectorXd _gradient(const Eigen::Matrix3d& F) const
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

    protected:
    double _gamma;
};

} // namespace Solver

#endif // __HYDROSTATIC_CONSTRAINT_HPP
