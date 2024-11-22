#ifndef __HYDROSTATIC_CONSTRAINT_HPP
#define __HYDROSTATIC_CONSTRAINT_HPP

#include "solver/ElementConstraint.hpp"

namespace Solver
{

class HydrostaticConstraint : public virtual ElementConstraint
{
    public:
    explicit HydrostaticConstraint(const double dt, XPBDMeshObject* obj, unsigned v1, unsigned v2, unsigned v3, unsigned v4);

    virtual double evaluate() const override;
    virtual Eigen::VectorXd gradient() const override;
    virtual ValueAndGradient evaluateWithGradient() const override;

    private:
    Eigen::VectorXd _gradient(const Eigen::Matrix3d& F) const;
    double _evaluate(const Eigen::Matrix3d& F) const;

    protected:
    double _gamma;
};

} // namespace Solver

#endif // __HYDROSTATIC_CONSTRAINT_HPP
