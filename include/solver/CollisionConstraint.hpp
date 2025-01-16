#ifndef __COLLISION_CONSTRAINT_HPP
#define __COLLISION_CONSTRAINT_HPP

#include "solver/Constraint.hpp"

namespace Solver
{

/** Defines a generic collision constraint.
 * 
 */
class CollisionConstraint : public Constraint
{
    public:
    CollisionConstraint(const std::vector<PositionReference>& positions, const Eigen::Vector3d& collision_normal)
        : Constraint(positions, 0), _collision_normal(collision_normal)
    {}

    inline virtual Eigen::Vector3d p1() const = 0;
    inline virtual Eigen::Vector3d p2() const = 0;

    inline virtual Eigen::Vector3d prevP1() const = 0;
    inline virtual Eigen::Vector3d prevP2() const = 0;

    inline Eigen::Vector3d collisionNormal() const { return _collision_normal; }

    protected:
    Eigen::Vector3d _collision_normal;
};

} // namespace Solver

#endif // __COLLISION_CONSTRAINT_HPP