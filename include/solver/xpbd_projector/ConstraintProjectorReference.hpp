#ifndef __CONSTRAINT_PROJECTOR_REFERENCE_HPP
#define __CONSTRAINT_PROJECTOR_REFERENCE_HPP

namespace Solver
{
/** A consistent reference to a ConstraintProjector.
 * ConstraintProjectors are stored in vectors that can change size. If a vector has to allocate more memory, any pointers or references
 * to its contents are invalidated. This becomes a problem for constraints that are dynamically added and removed (like collision constraints).
 * 
 * By storing a pointer to the container and its index, we can ensure that even if the vector changes sizes, we still have a valid
 * reference to the ConstraintProjector.
 */
template<typename ConstraintProjectorType>
class ConstraintProjectorReference
{
    public:

    using constraint_projector_type = ConstraintProjectorType;
    using vector_type = std::vector<constraint_projector_type>;
    public:
    ConstraintProjectorReference(vector_type& vec, int index)
        : _vec(vec), _index(index)
    {

    }

    const constraint_projector_type& operator()() const
    {
        return _vec.at(_index);
    } 

    constraint_projector_type& operator()()
    {
        return _vec.at(_index);
    }

    private:
    vector_type& _vec;
    int _index;
};

/** A const-reference version of ConstraintProjectorReference. */
template<typename ConstraintProjectorType>
class ConstraintProjectorConstReference
{
    public:
    using constraint_projector_type = const ConstraintProjectorType;
    using vector_type = const std::vector<constraint_projector_type>;

    public:
    ConstraintProjectorConstReference(vector_type& vec, int index)
        : _vec(vec), _index(index)
    {

    }

    constraint_projector_type& operator()() const
    {
        return _vec.at(_index);
    }

    private:
    vector_type& _vec;
    int _index;
};

} // namespace Solver

#endif // __CONSTRAINT_PROJECTOR_REFERENCE_HPP