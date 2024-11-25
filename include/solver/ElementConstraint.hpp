#ifndef __ELEMENT_CONSTRAINT_HPP
#define __ELEMENT_CONSTRAINT_HPP

#include "solver/Constraint.hpp"

namespace Solver 
{

class ElementConstraint : public Constraint
{
    public:
    ElementConstraint(const double dt, XPBDMeshObject* obj, unsigned v1, unsigned v2, unsigned v3, unsigned v4)
        : Constraint(dt, std::vector<PositionReference>({
            PositionReference(obj, v1),
            PositionReference(obj, v2),
            PositionReference(obj, v3),
            PositionReference(obj, v4)}))
    {
        // if this constructor is used, we assumes that this constraint is created when this object is in the rest configuration
        // so we can calculate Q and volume
        // calculate Q
        const Eigen::Vector3d X0 = _positions[0].position();
        const Eigen::Vector3d X1 = _positions[1].position();
        const Eigen::Vector3d X2 = _positions[2].position();
        const Eigen::Vector3d X3 = _positions[3].position();

        Eigen::Matrix3d X;
        X.col(0) = (X0 - X3);
        X.col(1) = (X1 - X3);
        X.col(2) = (X2 - X3);

        _Q = X.inverse();

        // calculate volume
        _volume = std::abs(X.determinant()/6);
    }

    protected:
    inline Eigen::Matrix3d _computeF() const
    {
        Eigen::Matrix3d X;
        const Eigen::Vector3d X3 = _positions[3].position();
        // create the deformed shape matrix from current deformed vertex positions
        X.col(0) = _positions[0].position() - X3;
        X.col(1) = _positions[1].position() - X3;
        X.col(2) = _positions[2].position() - X3;

        // compute and return F
        return X * _Q;
    }

    protected:
    Eigen::Matrix3d _Q;
    double _volume;

};

} // namespace Solver


#endif // __ELEMENT_CONSTRAINT_HPP