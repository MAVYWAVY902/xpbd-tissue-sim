#ifndef __CONSTRAINT_REFERENCE_HPP
#define __CONSTRAINT_REFERENCE_HPP

#include <iostream>

namespace Solver
{

/** A consistent reference to a Constraint.
 * Constraints are stored in vectors that can change size. If a vector has to allocate more memory, any pointers or references
 * to its contents are invalidated. This becomes a problem for constraints that are dynamically added and removed (like collision constraints).
 * 
 * By storing a pointer to the container and its index, we can ensure that even if the vector changes sizes, we still have a valid
 * reference to the Constraint.
 */
template <typename Constraint>
class ConstraintReference
{
public:
    using constraint_type = Constraint;
    using vector_type = std::vector<constraint_type>;
    
    ConstraintReference(vector_type& vec, int index)
        : _vec(vec), _index(index)
    {
    }

    const constraint_type* operator->() const
    {
        return &_vec.at(_index);
    }

    constraint_type* operator->()
    {
        return &_vec.at(_index);
    }

    const constraint_type& get() const
    {
        return _vec.at(_index);
    }

    constraint_type& get()
    {
        return _vec.at(_index);
    }

    private:
    vector_type& _vec;
    int _index;
};

} // namespace Solver

#endif // __CONSTRAINT_REFERENCE_HPP