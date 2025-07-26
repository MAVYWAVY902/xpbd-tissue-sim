#ifndef __CONSTRAINT_PROJECTOR_REFERENCE_HPP
#define __CONSTRAINT_PROJECTOR_REFERENCE_HPP

#include "solver/xpbd_projector/ConstraintProjectorTraits.hpp"

#include <variant>

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

    const constraint_projector_type* operator->() const
    {
        return &_vec.at(_index);
    } 

    constraint_projector_type operator->()
    {
        return &_vec.at(_index);
    }

    private:
    vector_type& _vec;
    int _index;
};

template<typename ...Constraints>
class ConstraintProjectorReferenceWrapper
{
public:
    using projector_type_1st_order = typename ConstraintProjectorTraits<true, Constraints...>::type;
    using projector_type_2nd_order = typename ConstraintProjectorTraits<false, Constraints...>::type;

    using variant_type = std::variant<ConstraintProjectorReference<projector_type_1st_order>, ConstraintProjectorReference<projector_type_2nd_order>>;

    // template<bool IsFirstOrder>
    // ConstraintProjectorReferenceWrapper(typename ConstraintProjectorReference<ConstraintProjectorTraits<IsFirstOrder, Constraints...>>::type&& proj_reference)
    //     : _variant(std::move(proj_reference))
    // {}

    ConstraintProjectorReferenceWrapper(ConstraintProjectorReference<typename ConstraintProjectorTraits<false, Constraints...>::type>&& proj_reference)
        : _variant(std::move(proj_reference))
    {}

    ConstraintProjectorReferenceWrapper(ConstraintProjectorReference<typename ConstraintProjectorTraits<true, Constraints...>::type>&& proj_reference)
        : _variant(std::move(proj_reference))
    {}
    
    /** TODO: add wrapper methods for interfacing with ConstraintProjector */

private:
    variant_type _variant;
};

} // namespace Solver

#endif // __CONSTRAINT_PROJECTOR_REFERENCE_HPP