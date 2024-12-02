#ifndef __DEVIATORIC_CONSTRAINT_HPP
#define __DEVIATORIC_CONSTRAINT_HPP

#include "solver/ElementConstraint.hpp"

namespace Solver
{

class DeviatoricConstraint : public ElementConstraint
{
    public:
    DeviatoricConstraint(XPBDMeshObject* obj, unsigned v1, unsigned v2, unsigned v3, unsigned v4)
        : ElementConstraint(obj, v1, v2, v3, v4)
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

    inline void evaluate(double* C, double* additional_memory) const
    {
        double* F = additional_memory;
        double* X = F + 9;
        _computeF(F, X);
        _evaluate(C, F);
    }

    inline void gradient(double* grad, double* additional_memory) const
    {
        double* F = additional_memory;
        double* X = F+9;
        _computeF(F, X);
        double C;
        _evaluate(&C, F);
        _gradient(grad, &C, F);
    }

    inline void evaluateWithGradient(double* C, double* grad, double* additional_memory) const
    {
        double* F = additional_memory;
        double* X = F+9;
        _computeF(F, X);
        _evaluate(C, F);
        _gradient(grad, C, F);
    }

    inline unsigned memoryNeeded() const override
    {
        // 9 for F
        // 9 for X
        return 18 * sizeof(double);
    }

    private:
    inline double _evaluate(const Eigen::Matrix3d& F) const
    {
        return std::sqrt(F.col(0).squaredNorm() + F.col(1).squaredNorm() + F.col(2).squaredNorm());
    }

    inline Eigen::VectorXd _gradient(const Eigen::Matrix3d& F, const double C) const
    {
        Eigen::Matrix3d prod = 1/C * F * _Q.transpose();

        Eigen::VectorXd grad = Eigen::VectorXd::Zero(_gradient_vector_size);
        grad(Eigen::seq(_gradient_vector_index[0],_gradient_vector_index[0]+2)) = prod.col(0);
        grad(Eigen::seq(_gradient_vector_index[1],_gradient_vector_index[1]+2)) = prod.col(1);
        grad(Eigen::seq(_gradient_vector_index[2],_gradient_vector_index[2]+2)) = prod.col(2);
        grad(Eigen::seq(_gradient_vector_index[3],_gradient_vector_index[3]+2)) = -prod.col(0) - prod.col(1) - prod.col(2);

        return grad;
    }

    inline void _evaluate(double* C, double* F) const
    {
        *C = std::sqrt(F[0]*F[0] + F[1]*F[1] + F[2]*F[2] + F[3]*F[3] + F[4]*F[4] + F[5]*F[5] + F[6]*F[6] + F[7]*F[7] + F[8]*F[8]);
    }

    inline void _gradient(double* grad, double* C, double* F) const
    {
        // A = 1/C * F * Q^T
        // columns of A are the gradients
        // F is column major
        double inv_C = 1.0/(*C);
        grad[_gradient_vector_index[0]]   = inv_C * (F[0]*_Q(0,0) + F[3]*_Q(0,1) + F[6]*_Q(0,2));
        grad[_gradient_vector_index[1]] = inv_C * (F[1]*_Q(0,0) + F[4]*_Q(0,1) + F[7]*_Q(0,2));
        grad[_gradient_vector_index[2]] = inv_C * (F[2]*_Q(0,0) + F[5]*_Q(0,1) + F[8]*_Q(0,2));

        grad[_gradient_vector_index[3]]   = inv_C * (F[0]*_Q(1,0) + F[3]*_Q(1,1) + F[6]*_Q(1,2));
        grad[_gradient_vector_index[4]] = inv_C * (F[1]*_Q(1,0) + F[4]*_Q(1,1) + F[7]*_Q(1,2));
        grad[_gradient_vector_index[5]] = inv_C * (F[2]*_Q(1,0) + F[5]*_Q(1,1) + F[8]*_Q(1,2));

        grad[_gradient_vector_index[6]]   = inv_C * (F[0]*_Q(2,0) + F[3]*_Q(2,1) + F[6]*_Q(2,2));
        grad[_gradient_vector_index[7]] = inv_C * (F[1]*_Q(2,0) + F[4]*_Q(2,1) + F[7]*_Q(2,2));
        grad[_gradient_vector_index[8]] = inv_C * (F[2]*_Q(2,0) + F[5]*_Q(2,1) + F[8]*_Q(2,2));

        grad[_gradient_vector_index[9]]  = -grad[_gradient_vector_index[0]] - grad[_gradient_vector_index[3]] - grad[_gradient_vector_index[6]];
        grad[_gradient_vector_index[10]] = -grad[_gradient_vector_index[1]] - grad[_gradient_vector_index[4]] - grad[_gradient_vector_index[7]];
        grad[_gradient_vector_index[11]] = -grad[_gradient_vector_index[2]] - grad[_gradient_vector_index[5]] - grad[_gradient_vector_index[8]];
    }

};

} // namespace Solver

#endif // __DEVIATORIC_CONSTRAINT_HPP