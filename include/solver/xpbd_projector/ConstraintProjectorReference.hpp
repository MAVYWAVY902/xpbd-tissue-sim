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

    ConstraintProjectorReference(ConstraintProjectorReference&&) = default;
    ConstraintProjectorReference& operator=(ConstraintProjectorReference&&) = default;
    
    ConstraintProjectorReference(const ConstraintProjectorReference&) = default;
    ConstraintProjectorReference& operator=(const ConstraintProjectorReference&) = default;

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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/** A wrapper around ConstraintProjectorReference that is agnostic to whether the underlying ConstraintProjector is 1st-Order or 2nd-Order.
 * This is useful for treating references to ConstraintProjectors the same way regardless of the underlying algorithm, while maintaining the 
 * strong compile-time typing of the templated classes.
 * 
 * What's nice is that we don't actually need the ConstraintProjector type directly, we can infer it from the Constraints template parameter. This
 * completely removes the need for the IsFirstOrder template parameter, which is exactly what we want.
 */
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

    /** Return the stored object as the specified type, if the type matches.
     * If the type doesn't match, return nullptr
     */
    template<bool IsFirstOrder>
    typename ConstraintProjectorTraits<IsFirstOrder, Constraints...>::type* getAs()
    {
        if (std::holds_alternative<typename ConstraintProjectorTraits<IsFirstOrder, Constraints...>::type>(_variant))
        {
            return std::get<typename ConstraintProjectorTraits<IsFirstOrder, Constraints...>::type>(_variant);
        }
        else
        {
            return nullptr;
        }
    }

    template<bool IsFirstOrder>
    const typename ConstraintProjectorTraits<IsFirstOrder, Constraints...>::type* getAs() const
    {
        if (std::holds_alternative<typename ConstraintProjectorTraits<IsFirstOrder, Constraints...>::type>(_variant))
        {
            return std::get<typename ConstraintProjectorTraits<IsFirstOrder, Constraints...>::type>(_variant);
        }
        else
        {
            return nullptr;
        }
    }
    
    /** TODO: add wrapper methods for interfacing with ConstraintProjector */
    std::vector<Vec3r> constraintForces() const
    {
        return std::visit([](const auto& obj) { return obj->constraintForces(); }, _variant);
    }

    void setValidity(bool valid)
    {
        return std::visit([&](auto& obj) { return obj->setValidity(valid); }, _variant);
    }

    bool isValid() const
    {
        return std::visit([](const auto& obj) { return obj->isValid(); }, _variant);
    }

private:
    variant_type _variant;
};

} // namespace Solver

#endif // __CONSTRAINT_PROJECTOR_REFERENCE_HPP