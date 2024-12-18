#ifndef __ELEMENT_CONSTRAINT_HPP
#define __ELEMENT_CONSTRAINT_HPP

#include "solver/Constraint.hpp"

namespace Solver 
{

/** Represents a constraint that is imposed on a tetrahedral mesh element.
 * 
 * Still an abstract class, but provides functionality that computes the rest-state matrix Q, the element volume, and the deformation gradient.
 */
class ElementConstraint : public Constraint
{
    public:
    /** Creates a constraint from a reference to the MeshObject and the four vertices that make up the tetrahedral element that this constraint is applied to.
     * @param obj - a pointer to the XPBDMeshObject that this constraint belongs to
     * @param v1 - the 1st vertex of the tetrahedral element
     * @param v2 - the 2nd vertex of the tetrahedral element
     * @param v3 - the 3rd vertex of the tetrahedral element
     * @param v4 - the 4th vertex of the tetrahedral element
     */
    ElementConstraint(const Sim::XPBDMeshObject* obj, int v1, int v2, int v3, int v4)
        : Constraint(std::vector<PositionReference>({
            PositionReference(obj, v1),
            PositionReference(obj, v2),
            PositionReference(obj, v3),
            PositionReference(obj, v4)}))
    {
        // if this constructor is used, we assumes that this constraint is created when this object is in the rest configuration
        // so we can calculate Q and volume

        // calculate Q
        const Eigen::Vector3d& X0 = Eigen::Map<Eigen::Vector3d>(_positions[0].position_ptr);
        const Eigen::Vector3d& X1 = Eigen::Map<Eigen::Vector3d>(_positions[1].position_ptr);
        const Eigen::Vector3d& X2 = Eigen::Map<Eigen::Vector3d>(_positions[2].position_ptr);
        const Eigen::Vector3d& X3 = Eigen::Map<Eigen::Vector3d>(_positions[3].position_ptr);

        Eigen::Matrix3d X;
        X.col(0) = (X0 - X3);
        X.col(1) = (X1 - X3);
        X.col(2) = (X2 - X3);

        _Q = X.inverse();

        // calculate volume
        _volume = std::abs(X.determinant()/6);
    }

    protected:

    /** Compute the deformation gradient and return it, using the 4 positions referenced by this constraint.
     */
    inline Eigen::Matrix3d _computeF() const
    {
        Eigen::Matrix3d X;
        const Eigen::Vector3d& X3 = Eigen::Map<Eigen::Vector3d>(_positions[3].position_ptr);
        // create the deformed shape matrix from current deformed vertex positions
        X.col(0) = Eigen::Map<Eigen::Vector3d>(_positions[0].position_ptr) - X3;
        X.col(1) = Eigen::Map<Eigen::Vector3d>(_positions[1].position_ptr) - X3;
        X.col(2) = Eigen::Map<Eigen::Vector3d>(_positions[2].position_ptr) - X3;

        // compute and return F
        return X * _Q;
    }

    /** Compute the deformation gradient with pre-allocated memory.
     * @param F (OUTPUT) - the pointer to the (currently empty) 3x3 deformation gradient matrix. Expects that F is COLUMN-MAJOR and 3x3.
     * @param X (OUTPUT) - the pointer to the (currently empty) 3x3 deformed state matrix. Expects that X is COLUMN-MAJOR and 3x3.
     */
    inline void _computeF(double* F, double* X) const
    {
        // Eigen::Map<Eigen::Matrix3d> X_mat(X);
        // Eigen::Map<Eigen::Matrix3d> F_mat(F);
        const double* X0 = _positions[0].position_ptr;
        const double* X1 = _positions[1].position_ptr;
        const double* X2 = _positions[2].position_ptr;
        const double* X3 = _positions[3].position_ptr;
        // create the deformed shape matrix from current deformed vertex positions
        // X_mat.col(0) = _positions[0].position() - X3;
        // X_mat.col(1) = _positions[1].position() - X3;
        // X_mat.col(2) = _positions[2].position() - X3;

        // const Eigen::Vector3d X3 = Eigen::Map<Eigen::Vector3d>(_positions[3].position_ptr);
        // create the deformed shape matrix from current deformed vertex positions
        // X_mat.col(0) = Eigen::Map<Eigen::Vector3d>(_positions[0].position_ptr) - X3;
        // X_mat.col(1) = Eigen::Map<Eigen::Vector3d>(_positions[1].position_ptr) - X3;
        // X_mat.col(2) = Eigen::Map<Eigen::Vector3d>(_positions[2].position_ptr) - X3;

        X[0] = X0[0]-X3[0]; X[1] = X0[1]-X3[1]; X[2] = X0[2]-X3[2];
        X[3] = X1[0]-X3[0]; X[4] = X1[1]-X3[1]; X[5] = X1[2]-X3[2];
        X[6] = X2[0]-X3[0]; X[7] = X2[1]-X3[1]; X[8] = X2[2]-X3[2];
        
        // X_mat = Eigen::Matrix3d::Identity();
        // compute F with Eigen matrix multiplication
        // this should modify the data pointed to by F
        // F_mat = X_mat * _Q;


        F[0] = (X[0]*_Q(0,0) + X[3]*_Q(1,0) + X[6]*_Q(2,0));
        F[1] = (X[1]*_Q(0,0) + X[4]*_Q(1,0) + X[7]*_Q(2,0));
        F[2] = (X[2]*_Q(0,0) + X[5]*_Q(1,0) + X[8]*_Q(2,0));

        F[3] = (X[0]*_Q(0,1) + X[3]*_Q(1,1) + X[6]*_Q(2,1));
        F[4] = (X[1]*_Q(0,1) + X[4]*_Q(1,1) + X[7]*_Q(2,1));
        F[5] = (X[2]*_Q(0,1) + X[5]*_Q(1,1) + X[8]*_Q(2,1));

        F[6] = (X[0]*_Q(0,2) + X[3]*_Q(1,2) + X[6]*_Q(2,2));
        F[7] = (X[1]*_Q(0,2) + X[4]*_Q(1,2) + X[7]*_Q(2,2));
        F[8] = (X[2]*_Q(0,2) + X[5]*_Q(1,2) + X[8]*_Q(2,2));

    }

    protected:
    Eigen::Matrix3d _Q;
    double _volume;

};

} // namespace Solver


#endif // __ELEMENT_CONSTRAINT_HPP