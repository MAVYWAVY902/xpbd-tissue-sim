#ifndef __XPBD_TYPEDEFS_HPP
#define __XPBD_TYPEDEFS_HPP

#include "solver/xpbd_projector/ConstraintProjector.hpp"
#include "solver/xpbd_projector/RigidBodyConstraintProjector.hpp"
#include "solver/xpbd_projector/CombinedConstraintProjector.hpp"
#include "solver/xpbd_projector/CombinedNeohookeanConstraintProjector.hpp"

#include "solver/constraint/HydrostaticConstraint.hpp"
#include "solver/constraint/DeviatoricConstraint.hpp"
#include "solver/constraint/StaticDeformableCollisionConstraint.hpp"
#include "solver/constraint/RigidDeformableCollisionConstraint.hpp"
#include "solver/constraint/AttachmentConstraint.hpp"

#include "solver/xpbd_solver/XPBDGaussSeidelSolver.hpp"
#include "solver/xpbd_solver/XPBDJacobiSolver.hpp"
#include "solver/xpbd_solver/XPBDParallelJacobiSolver.hpp"

#include "common/TypeList.hpp"

#include <variant>
#include <type_traits>

template<typename ...Projectors>
struct XPBDMeshObjectConstraintConfiguration
{
    using projector_type_list = TypeList<Projectors...>;
    using constraint_type_list = typename ConcatenateTypeLists<typename Projectors::constraint_type_list...>::type;

    // two ConstraintTypes are equal if their Projector types are equal
    template<typename ...OtherProjectors>
    bool operator==(const XPBDMeshObjectConstraintConfiguration<OtherProjectors...>&) const
    {
        return std::is_same_v<TypeList<Projectors...>, TypeList<OtherProjectors...>>;
    }
};

// Declare XPBDMeshObject constraint configurations
struct XPBDMeshObjectConstraintConfigurations
{
    // typedefs for readability
    private:
    using DevProjector = Solver::ConstraintProjector<false, Solver::DeviatoricConstraint>;
    using HydProjector = Solver::ConstraintProjector<false, Solver::HydrostaticConstraint>;
    using DevHydProjector = Solver::CombinedConstraintProjector<false, Solver::DeviatoricConstraint, Solver::HydrostaticConstraint>;
    using StatCollProjector = Solver::ConstraintProjector<false, Solver::StaticDeformableCollisionConstraint>;
    using RigiCollProjector = Solver::RigidBodyConstraintProjector<false, Solver::RigidDeformableCollisionConstraint>;
    using AttProjector = Solver::ConstraintProjector<false, Solver::AttachmentConstraint>;

    // public typedefs represent XPBDMeshObject constraint configurations
    public:
    using StableNeohookean = XPBDMeshObjectConstraintConfiguration<DevProjector, HydProjector, StatCollProjector, RigiCollProjector, AttProjector>;
    using StableNeohookeanCombined = XPBDMeshObjectConstraintConfiguration<DevHydProjector, StatCollProjector, RigiCollProjector, AttProjector>;

    using type_list = TypeList<StableNeohookean, StableNeohookeanCombined>;
    using variant_type = std::variant<StableNeohookean, StableNeohookeanCombined>;

    constexpr static StableNeohookean STABLE_NEOHOOKEAN{};
    constexpr static StableNeohookeanCombined STABLE_NEOHOOKEAN_COMBINED{};
};

// Declare XPBDMeshObject constraint configurations
struct FirstOrderXPBDMeshObjectConstraintConfigurations
{
    // typedefs for readability
    private:
    using DevProjector = Solver::ConstraintProjector<true, Solver::DeviatoricConstraint>;
    using HydProjector = Solver::ConstraintProjector<true, Solver::HydrostaticConstraint>;
    using DevHydProjector = Solver::CombinedConstraintProjector<true, Solver::DeviatoricConstraint, Solver::HydrostaticConstraint>;
    using StatCollProjector = Solver::ConstraintProjector<true, Solver::StaticDeformableCollisionConstraint>;
    using RigiCollProjector = Solver::RigidBodyConstraintProjector<true, Solver::RigidDeformableCollisionConstraint>;
    using AttProjector = Solver::ConstraintProjector<true, Solver::AttachmentConstraint>;

    // public typedefs represent XPBDMeshObject constraint configurations
    public:
    using StableNeohookean = XPBDMeshObjectConstraintConfiguration<DevProjector, HydProjector, StatCollProjector, RigiCollProjector, AttProjector>;
    using StableNeohookeanCombined = XPBDMeshObjectConstraintConfiguration<DevHydProjector, StatCollProjector, RigiCollProjector, AttProjector>;

    using type_list = TypeList<StableNeohookean, StableNeohookeanCombined>;
    using variant_type = std::variant<StableNeohookean, StableNeohookeanCombined>;

    constexpr static StableNeohookean STABLE_NEOHOOKEAN{};
    constexpr static StableNeohookeanCombined STABLE_NEOHOOKEAN_COMBINED{};
};

template<typename ProjectorTypeList> struct XPBDObjectSolverTypes;

template<typename ...Projectors>
struct XPBDObjectSolverTypes<TypeList<Projectors...>>
{
    // check and make sure all projector types are NOT 1st order projectors
    static_assert( (!Projectors::is_first_order && ...) );
    using GaussSeidel = Solver::XPBDGaussSeidelSolver<false, Projectors...>;
    using Jacobi = Solver::XPBDJacobiSolver<false, Projectors...>;
    using ParallelJacobi = Solver::XPBDParallelJacobiSolver<false, Projectors...>;

    using type_list = TypeList<GaussSeidel, Jacobi, ParallelJacobi>;
    using variant_type = std::variant<GaussSeidel, Jacobi, ParallelJacobi>;
};

template<typename ProjectorTypeList> struct FirstOrderXPBDObjectSolverTypes;

template<typename ...Projectors>
struct FirstOrderXPBDObjectSolverTypes<TypeList<Projectors...>>
{
    // check and make sure all projector types are 1st order projectors
    static_assert( (Projectors::is_first_order && ...) );
    using GaussSeidel = Solver::XPBDGaussSeidelSolver<true, Projectors...>;
    using Jacobi = Solver::XPBDJacobiSolver<true, Projectors...>;
    using ParallelJacobi = Solver::XPBDParallelJacobiSolver<true, Projectors...>;

    using type_list = TypeList<GaussSeidel, Jacobi, ParallelJacobi>;
    using variant_type = std::variant<GaussSeidel, Jacobi, ParallelJacobi>;
};



// Declare XPBDMeshObject solver types



#endif // __XPBD_TYPEDEFS_HPP