#ifndef __HYDROSTATIC_CONSTRAINT_HPP
#define __HYDROSTATIC_CONSTRAINT_HPP

#include "solver/ElementConstraint.hpp"

namespace Solver
{

/** Represents the hydrostatic constraint derived from the Stable Neo-hookean strain energy, proposed by Macklin et. al: https://mmacklin.com/neohookean.pdf
 */
class HydrostaticConstraint : public virtual ElementConstraint
{
    friend class CombinedNeohookeanConstraintProjector;
    
    public:
    /** Creates the hydrostatic constraint from a MeshObject and the 4 vertices that make up the tetrahedral element. */
    HydrostaticConstraint(const Sim::XPBDMeshObject* obj, int v1, int v2, int v3, int v4)
        : ElementConstraint(obj, v1, v2, v3, v4)
    {
        _alpha = 1/(obj->material().lambda() * _volume);            // set alpha after the ElementConstraint constructor because we need the element volume
        _gamma = obj->material().mu() / obj->material().lambda();  
    }

    /** Evaluates the current value of this constraint.
     * i.e. returns C(x)
     */
    inline double evaluate() const override
    {
        return _evaluate(_computeF());
    }

    /** Returns the gradient of this constraint in vector form.
     * i.e. returns delC(x)
     */
    inline Eigen::VectorXd gradient() const override
    {
        return _gradient(_computeF());
        
    }

    /** Returns the value and gradient of this constraint.
     * i.e. returns C(x) and delC(x) together.
     * 
     * This may be desirable when there would be duplicate work involved to evaluate constraint and its gradient separately.
     */
    inline Constraint::ValueAndGradient evaluateWithGradient() const override
    {
        const Eigen::Matrix3d F = _computeF();
        return ValueAndGradient(_evaluate(F), _gradient(F));
    }

    Eigen::Vector3d elasticForce(int index) override
    {
        double C;
        double delC[12];
        evaluateWithGradient(&C, delC);

        // compute elastic force for index
        int pos_index = -1;
        for (unsigned i = 0; i < _positions.size(); i++)
        {
            if (index == _positions[i].index)
            {
                pos_index = i;
                break;
            }
        }
        if (pos_index == -1)
            assert(0 && "This constraint does not affect passed position!");
        
        Eigen::Vector3d grad = Eigen::Map<Eigen::Vector3d>(delC + _gradient_vector_index[3*pos_index]);

        return -grad * (1.0/_alpha) * C;
    }


    /** Evaluates the current value of this constraint with pre-allocated memory.
     * i.e. returns C(x)
     * 
     * @param C (OUTPUT) - the pointer to the (currently empty) value of the constraint
     */
    inline void evaluate(double* C) const override
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
    inline void gradient(double* grad) const override
    {
        double F[9];
        double X[9];
        _computeF(F, X);
        _gradient(grad, F);     // additional memory needed for the gradient calculation (after F and X) is provided after X
    }


    /** Computes the value and gradient of this constraint with pre-allocated memory.
     * i.e. returns C(x) and delC(x) together.
     * 
     * This may be desirable when there would be duplicate work involved to evaluate constraint and its gradient separately.
     * 
     * @param C (OUTPUT) - the pointer to the (currently empty) value of the constraint
     * @param grad (OUTPUT) - the pointer to the (currently empty) constraint gradient vector. Expects it to be _gradient_vector_size x 1.
     */
    void evaluateWithGradient(double* C, double* grad) const override
    {
        double F[9];
        double X[9];
        _computeF(F, X);
        _evaluate(C, F);
        _gradient(grad, F);
    }

    private:

    /** Helper method to evaluate the constraint given the deformation gradient, F.
     * Avoids the need to recompute F if we already have it.
     */
    inline double _evaluate(const Eigen::Matrix3d& F) const
    {
        assert(0);

        return F.determinant() - (1 + _gamma);
    }
    
    /** Helper method to evaluate the constraint gradient given the deformation gradient, F.
     * Avoids the need to recompute F if we already have it.
     */
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

    /** Helper method to evaluate the constraint given the deformation gradient, F, using pre-allocated memory.
     * Avoids the need to recompute F if we already have it.
     */
    inline void _evaluate(double* C, double* F) const
    {
        // compute C(x) = det(F) - (1 + gamma)
        *C = F[0]*F[4]*F[8] - F[0]*F[7]*F[5] - F[3]*F[1]*F[8] + F[3]*F[7]*F[2] + F[6]*F[1]*F[5] - F[6]*F[4]*F[2] - (1+_gamma);
    }

    /** Helper method to evaluate the constraint gradient given the deformation gradient, F, useing pre-allocated memory.
     * Avoids the need to recompute F if we already have it.
     * 
     * Requires at least 9 * sizeof(double) bytes of additional memory for intermediate quantities.
     */
    inline void _gradient(double* grad, double* F) const
    {   
        // F_cross = [f2 x f3, f3 x f1, f1 x f2] where f_i are the columns of F
        // F_cross and F are both column-major
        // see supplementary material of Macklin paper for more details
        double F_cross[9];
        _cross3(F+3, F+6, F_cross);             // 2nd column of F crossed with 3rd column
        _cross3(F+6, F, F_cross+3);             // 3rd column of F crossed with 1st column
        _cross3(F, F+3, F_cross+6);             // 1st column of F crossed with 2nd column

        // for A = F_cross * Q^T,
        // 1st column of A is delC wrt 1st position
        // 2nd column of A is delC wrt 2nd position
        // 3rd column of A is delC wrt 3rd position
        // delC wrt 4th position is (-1st column - 2nd column - 3rd column)
        // see supplementary material of Macklin paper for more details

        // calculation of delC wrt 1st position
        grad[_gradient_vector_index[0]] = (F_cross[0]*_Q(0,0) + F_cross[3]*_Q(0,1) + F_cross[6]*_Q(0,2));
        grad[_gradient_vector_index[1]] = (F_cross[1]*_Q(0,0) + F_cross[4]*_Q(0,1) + F_cross[7]*_Q(0,2));
        grad[_gradient_vector_index[2]] = (F_cross[2]*_Q(0,0) + F_cross[5]*_Q(0,1) + F_cross[8]*_Q(0,2));

        // calculation of delC wrt 2nd postion
        grad[_gradient_vector_index[3]] = (F_cross[0]*_Q(1,0) + F_cross[3]*_Q(1,1) + F_cross[6]*_Q(1,2));
        grad[_gradient_vector_index[4]] = (F_cross[1]*_Q(1,0) + F_cross[4]*_Q(1,1) + F_cross[7]*_Q(1,2));
        grad[_gradient_vector_index[5]] = (F_cross[2]*_Q(1,0) + F_cross[5]*_Q(1,1) + F_cross[8]*_Q(1,2));

        // calculation of delC wrt 3rd position
        grad[_gradient_vector_index[6]] = (F_cross[0]*_Q(2,0) + F_cross[3]*_Q(2,1) + F_cross[6]*_Q(2,2));
        grad[_gradient_vector_index[7]] = (F_cross[1]*_Q(2,0) + F_cross[4]*_Q(2,1) + F_cross[7]*_Q(2,2));
        grad[_gradient_vector_index[8]] = (F_cross[2]*_Q(2,0) + F_cross[5]*_Q(2,1) + F_cross[8]*_Q(2,2));

        // calculation of delC wrt 4th position
        grad[_gradient_vector_index[9]]  = -grad[_gradient_vector_index[0]] - grad[_gradient_vector_index[3]] - grad[_gradient_vector_index[6]];
        grad[_gradient_vector_index[10]] = -grad[_gradient_vector_index[1]] - grad[_gradient_vector_index[4]] - grad[_gradient_vector_index[7]];
        grad[_gradient_vector_index[11]] = -grad[_gradient_vector_index[2]] - grad[_gradient_vector_index[5]] - grad[_gradient_vector_index[8]];
    }

    /** Helper method for cross product between two 3-vectors v1 and v2, store the result in v3 */
    inline void _cross3(double* v1, double* v2, double* v3) const
    {
        v3[0] = v1[1]*v2[2] - v1[2]*v2[1];
        v3[1] = v1[2]*v2[0] - v1[0]*v2[2];
        v3[2] = v1[0]*v2[1] - v1[1]*v2[0];
    }

    protected:
    double _gamma;  // the ratio mu/lambda - part of C(x)
};

} // namespace Solver

#endif // __HYDROSTATIC_CONSTRAINT_HPP
