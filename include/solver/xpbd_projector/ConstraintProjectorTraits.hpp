#ifndef __CONSTRAINT_PROJECTOR_TRAITS_HPP
#define __CONSTRAINT_PROJECTOR_TRAITS_HPP

#include "solver/xpbd_projector/ConstraintProjector.hpp"
#include "solver/xpbd_projector/RigidBodyConstraintProjector.hpp"
#include "solver/xpbd_projector/CombinedConstraintProjector.hpp"

namespace Solver
{


// helper to get the appropriate projector type given IsFirstOrder and a set of constraint types
template<bool IsFirstOrder, class ...Constraints>
struct ConstraintProjectorTraits;

namespace traits_detail
{
    // priority tags for overload resolution
    struct priority_tag_3 {};
    struct priority_tag_2 : priority_tag_3 {};
    struct priority_tag_1 : priority_tag_2 {};
    struct priority_tag_0 : priority_tag_1 {};

    // 1 RigidBodyConstraint = RigidBodyConstraintProjector
    // use the highest priority to override the default single constraint case
    template<bool IsFirstOrder, class Constraint>
    auto get_projector_type(priority_tag_0) -> 
        std::enable_if_t<std::is_base_of_v<RigidBodyConstraint, Constraint>, RigidBodyConstraintProjector<IsFirstOrder, Constraint>>;
    
    // 2 constraints = CombinedConstraintProjector
    template<bool IsFirstOrder, class Constraint1, class Constraint2>
    auto get_projector_type(priority_tag_3) -> CombinedConstraintProjector<IsFirstOrder, Constraint1, Constraint2>;

    // 1 constraint = ConstraintProjector
    template<bool IsFirstOrder, class Constraint>
    auto get_projector_type(priority_tag_3) -> ConstraintProjector<IsFirstOrder, Constraint>;

} // namespace traits_detail

template<bool IsFirstOrder, class ...Constraints>
struct ConstraintProjectorTraits
{
    // use decltype on a templated function to get the return type!
    // pass in priority tag 0 for overload resolution - function templates with priority tag 0 will be matched first, and priority tag 3 will be matched last
    using type = decltype(traits_detail::get_projector_type<IsFirstOrder, Constraints...>(traits_detail::priority_tag_0{}));
};

} // namespace Solver

#endif // __CONSTRAINT_PROJECTOR_REFERENCE_HPP