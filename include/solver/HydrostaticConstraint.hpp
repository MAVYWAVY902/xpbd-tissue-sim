#ifndef __HYDROSTATIC_CONSTRAINT_HPP
#define __HYDROSTATIC_CONSTRAINT_HPP

#include "solver/ElementConstraint.hpp"

namespace Solver
{

/** Represents the hydrostatic constraint derived from the Stable Neo-hookean strain energy, proposed by Macklin et. al: https://mmacklin.com/neohookean.pdf
 */
class HydrostaticConstraint : public virtual ElementConstraint
{
    public:
    HydrostaticConstraint(XPBDMeshObject* obj, unsigned v1, unsigned v2, unsigned v3, unsigned v4)
        : ElementConstraint(obj, v1, v2, v3, v4)
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
        double* X = F + 9;
        _computeF(F, X);
        _gradient(grad, F, additional_memory + 18);
    }

    void evaluateWithGradient(double* C, double* grad, double* additional_memory) const
    {
        double* F = additional_memory;
        double* X = F+9;
        _computeF(F, X);
        _evaluate(C, F);
        _gradient(grad, F, additional_memory + 18);
    }

    inline unsigned memoryNeeded() const override
    {
        // 9 for F
        // 9 for X
        // 9 for F_cross
        return 27 * sizeof(double);
    }

    private:
    inline double _evaluate(const Eigen::Matrix3d& F) const
    {
        assert(0);

        return F.determinant() - (1 + _gamma);
    }

    inline Eigen::VectorXd _gradient(const Eigen::Matrix3d& F) const
    {

        assert(0);

        Eigen::Matrix3d F_cross, prod;
        F_cross.col(0) = F.col(1).cross(F.col(2));
        F_cross.col(1) = F.col(2).cross(F.col(0));
        F_cross.col(2) = F.col(0).cross(F.col(1));

        prod = F_cross * _Q.transpose();

        // TODO: fix this - I think the gradient vector indices are wrong (though this is not used by the solver loop)
        Eigen::VectorXd grad = Eigen::VectorXd::Zero(_gradient_vector_size);
        grad(Eigen::seq(_gradient_vector_index[0],_gradient_vector_index[0]+2)) = prod.col(0);
        grad(Eigen::seq(_gradient_vector_index[1],_gradient_vector_index[1]+2)) = prod.col(1);
        grad(Eigen::seq(_gradient_vector_index[2],_gradient_vector_index[2]+2)) = prod.col(2);
        grad(Eigen::seq(_gradient_vector_index[3],_gradient_vector_index[3]+2)) = -prod.col(0) - prod.col(1) - prod.col(2);

        return grad;
    }

    inline void _evaluate(double* C, double* F) const
    {
        // C(x) = det(F) - (1 + gamma)
        *C = F[0]*F[4]*F[8] - F[0]*F[7]*F[5] - F[3]*F[1]*F[8] + F[3]*F[7]*F[2] + F[6]*F[1]*F[5] - F[6]*F[4]*F[2] - (1+_gamma);
    }

    inline void _gradient(double* grad, double* F, double* additional_memory) const
    {   
        double* F_cross = additional_memory;
        _cross3(F+3, F+6, F_cross);
        _cross3(F+6, F, F_cross+3);
        _cross3(F, F+3, F_cross+6);

        grad[_gradient_vector_index[0]] = (F_cross[0]*_Q(0,0) + F_cross[3]*_Q(0,1) + F_cross[6]*_Q(0,2));
        grad[_gradient_vector_index[1]] = (F_cross[1]*_Q(0,0) + F_cross[4]*_Q(0,1) + F_cross[7]*_Q(0,2));
        grad[_gradient_vector_index[2]] = (F_cross[2]*_Q(0,0) + F_cross[5]*_Q(0,1) + F_cross[8]*_Q(0,2));

        grad[_gradient_vector_index[3]] = (F_cross[0]*_Q(1,0) + F_cross[3]*_Q(1,1) + F_cross[6]*_Q(1,2));
        grad[_gradient_vector_index[4]] = (F_cross[1]*_Q(1,0) + F_cross[4]*_Q(1,1) + F_cross[7]*_Q(1,2));
        grad[_gradient_vector_index[5]] = (F_cross[2]*_Q(1,0) + F_cross[5]*_Q(1,1) + F_cross[8]*_Q(1,2));

        grad[_gradient_vector_index[6]] = (F_cross[0]*_Q(2,0) + F_cross[3]*_Q(2,1) + F_cross[6]*_Q(2,2));
        grad[_gradient_vector_index[7]] = (F_cross[1]*_Q(2,0) + F_cross[4]*_Q(2,1) + F_cross[7]*_Q(2,2));
        grad[_gradient_vector_index[8]] = (F_cross[2]*_Q(2,0) + F_cross[5]*_Q(2,1) + F_cross[8]*_Q(2,2));

        grad[_gradient_vector_index[9]]  = -grad[_gradient_vector_index[0]] - grad[_gradient_vector_index[3]] - grad[_gradient_vector_index[6]];
        grad[_gradient_vector_index[10]] = -grad[_gradient_vector_index[1]] - grad[_gradient_vector_index[4]] - grad[_gradient_vector_index[7]];
        grad[_gradient_vector_index[11]] = -grad[_gradient_vector_index[2]] - grad[_gradient_vector_index[5]] - grad[_gradient_vector_index[8]];
    }

    inline void _cross3(double* v1, double* v2, double* v3) const
    {
        v3[0] = v1[1]*v2[2] - v1[2]*v2[1];
        v3[1] = v1[2]*v2[0] - v1[0]*v2[2];
        v3[2] = v1[0]*v2[1] - v1[1]*v2[0];
    }

    protected:
    double _gamma;
};

} // namespace Solver

#endif // __HYDROSTATIC_CONSTRAINT_HPP
