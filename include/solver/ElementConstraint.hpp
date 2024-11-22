#ifndef __ELEMENT_CONSTRAINT_HPP
#define __ELEMENT_CONSTRAINT_HPP

#include "solver/Constraint.hpp"

namespace Solver 
{

class ElementConstraint : public Constraint
{
    public:
    /** Assumes the object that owns this constraint is at it's rest configuration. */
    explicit ElementConstraint(const double dt, XPBDMeshObject* obj, unsigned v1, unsigned v2, unsigned v3, unsigned v4);

    protected:
    Eigen::Matrix3d _computeF() const;
    
    Eigen::Matrix3d _Q;
    double _volume;

};

} // namespace Solver


#endif // __ELEMENT_CONSTRAINT_HPP