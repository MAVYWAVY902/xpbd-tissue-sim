#ifndef __DEVIATORIC_CONSTRAINT_HPP
#define __DEVIATORIC_CONSTRAINT_HPP

#include "solver/ElementConstraint.hpp"

namespace Solver
{

/** Represents the deviatoric constraint derived from the Stable Neo-hookean strain energy, proposed by Macklin et. al: https://mmacklin.com/neohookean.pdf
 */
class DeviatoricConstraint : public ElementConstraint
{
    friend class CombinedNeohookeanConstraintProjector;
    
    public:
    /** Creates the deviatoric constraint from a MeshObject and the 4 vertices that make up the tetrahedral element. */
    DeviatoricConstraint(const Sim::XPBDMeshObject* obj, int v1, int v2, int v3, int v4)
        : ElementConstraint(obj, v1, v2, v3, v4)
    {
        _alpha = 1/(obj->material().mu() * _volume);
    }

    /** Evaluates the current value of this constraint.
     * i.e. returns C(x)
     */
    inline double evaluate() const
    {
        return _evaluate(_computeF());
    }

    /** Returns the gradient of this constraint in vector form.
     * i.e. returns delC(x)
     */
    inline Eigen::VectorXd gradient() const
    {
        const Eigen::Matrix3d F = _computeF();
        const double C = _evaluate(F);

        return _gradient(F, C);
        
    }

    /** Returns the value and gradient of this constraint.
     * i.e. returns C(x) and delC(x) together.
     * 
     * This may be desirable when there would be duplicate work involved to evaluate constraint and its gradient separately.
     */
    inline Constraint::ValueAndGradient evaluateWithGradient() const
    {
        const Eigen::Matrix3d F = _computeF();
        const double C = _evaluate(F);
        return ValueAndGradient(C, _gradient(F, C));
    }


    /** Evaluates the current value of this constraint with pre-allocated memory.
     * i.e. returns C(x)
     * 
     * @param C (OUTPUT) - the pointer to the (currently empty) value of the constraint
     */
    inline void evaluate(double* C) const
    {
        double F[9];
        double X[9];
        _computeF(F, X);
        _evaluate(C, F);
    }

    /** Computes the gradient of this constraint in vector form with pre-allocated memory.
     * i.e. returns delC(x)
     * 
     * @param grad (OUTPUT) - the pointer to the (currently empty) constraint gradient vector. Expects it to be _gradient_vector_size x 1.
     */
    inline void gradient(double* grad) const
    {
        double F[9];
        double X[9];
        _computeF(F, X);
        double C;
        _evaluate(&C, F);                   // we need C(x) since it is used in the gradient calculation
        _gradient(grad, &C, F);
    }

    /** Computes the value and gradient of this constraint with pre-allocated memory.
     * i.e. returns C(x) and delC(x) together.
     * 
     * This may be desirable when there would be duplicate work involved to evaluate constraint and its gradient separately.
     * 
     * @param C (OUTPUT) - the pointer to the (currently empty) value of the constraint
     * @param grad (OUTPUT) - the pointer to the (currently empty) constraint gradient vector. Expects it to be _gradient_vector_size x 1.
     */
    inline void evaluateWithGradient(double* C, double* grad) const
    {
        double F[9];
        double X[9];
        _computeF(F, X);
        _evaluate(C, F);
        _gradient(grad, C, F);
    }

    private:

    /** Helper method to evaluate the constraint given the deformation gradient, F.
     * Avoids the need to recompute F if we already have it.
     */
    inline double _evaluate(const Eigen::Matrix3d& F) const
    {
        assert(0);
        return std::sqrt(F.col(0).squaredNorm() + F.col(1).squaredNorm() + F.col(2).squaredNorm());
    }

    /** Helper method to evaluate the constraint gradient given the deformation gradient, F.
     * Avoids the need to recompute F if we already have it.
     */
    inline Eigen::VectorXd _gradient(const Eigen::Matrix3d& F, const double C) const
    {
        assert(0);

        Eigen::Matrix3d prod = 1/C * F * _Q.transpose();

        Eigen::VectorXd grad = Eigen::VectorXd::Zero(_gradient_vector_size);
        grad(Eigen::seq(_gradient_vector_index[0],_gradient_vector_index[0]+2)) = prod.col(0);
        grad(Eigen::seq(_gradient_vector_index[1],_gradient_vector_index[1]+2)) = prod.col(1);
        grad(Eigen::seq(_gradient_vector_index[2],_gradient_vector_index[2]+2)) = prod.col(2);
        grad(Eigen::seq(_gradient_vector_index[3],_gradient_vector_index[3]+2)) = -prod.col(0) - prod.col(1) - prod.col(2);

        return grad;
    }

    /** Helper method to evaluate the constraint given the deformation gradient, F, using pre-allocated memory.
     * Avoids the need to recompute F if we already have it.
     */
    inline void _evaluate(double* C, double* F) const
    {
        // C = frob(F)
        *C = std::sqrt(F[0]*F[0] + F[1]*F[1] + F[2]*F[2] + F[3]*F[3] + F[4]*F[4] + F[5]*F[5] + F[6]*F[6] + F[7]*F[7] + F[8]*F[8]);
    }

    /** Helper method to evaluate the constraint gradient given the deformation gradient, F, useing pre-allocated memory.
     * Avoids the need to recompute F and C(x) if we already have it.
     */
    inline void _gradient(double* grad, double* C, double* F) const
    {
        // for A = 1/C * F * Q^T,
        // 1st column of A is delC wrt 1st position
        // 2nd column of A is delC wrt 2nd position
        // 3rd column of A is delC wrt 3rd position
        // delC wrt 4th position is (-1st column - 2nd column - 3rd column)
        // see supplementary material of Macklin paper for more details

        double inv_C = 1.0/(*C);

        // F is column major
        // calculation of delC wrt 1st position
        grad[_gradient_vector_index[0]] = inv_C * (F[0]*_Q(0,0) + F[3]*_Q(0,1) + F[6]*_Q(0,2));
        grad[_gradient_vector_index[1]] = inv_C * (F[1]*_Q(0,0) + F[4]*_Q(0,1) + F[7]*_Q(0,2));
        grad[_gradient_vector_index[2]] = inv_C * (F[2]*_Q(0,0) + F[5]*_Q(0,1) + F[8]*_Q(0,2));

        // calculation of delC wrt 2nd position
        grad[_gradient_vector_index[3]] = inv_C * (F[0]*_Q(1,0) + F[3]*_Q(1,1) + F[6]*_Q(1,2));
        grad[_gradient_vector_index[4]] = inv_C * (F[1]*_Q(1,0) + F[4]*_Q(1,1) + F[7]*_Q(1,2));
        grad[_gradient_vector_index[5]] = inv_C * (F[2]*_Q(1,0) + F[5]*_Q(1,1) + F[8]*_Q(1,2));

        // calculation of delC wrt 3rd position
        grad[_gradient_vector_index[6]] = inv_C * (F[0]*_Q(2,0) + F[3]*_Q(2,1) + F[6]*_Q(2,2));
        grad[_gradient_vector_index[7]] = inv_C * (F[1]*_Q(2,0) + F[4]*_Q(2,1) + F[7]*_Q(2,2));
        grad[_gradient_vector_index[8]] = inv_C * (F[2]*_Q(2,0) + F[5]*_Q(2,1) + F[8]*_Q(2,2));

        // calculation of delC wrt 4th position
        grad[_gradient_vector_index[9]]  = -grad[_gradient_vector_index[0]] - grad[_gradient_vector_index[3]] - grad[_gradient_vector_index[6]];
        grad[_gradient_vector_index[10]] = -grad[_gradient_vector_index[1]] - grad[_gradient_vector_index[4]] - grad[_gradient_vector_index[7]];
        grad[_gradient_vector_index[11]] = -grad[_gradient_vector_index[2]] - grad[_gradient_vector_index[5]] - grad[_gradient_vector_index[8]];
    }

};

} // namespace Solver

#endif // __DEVIATORIC_CONSTRAINT_HPP