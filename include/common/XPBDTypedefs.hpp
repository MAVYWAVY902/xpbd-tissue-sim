#ifndef __XPBD_TYPEDEFS_HPP
#define __XPBD_TYPEDEFS_HPP

#include "solver/xpbd_projector/ConstraintProjector.hpp"
#include "solver/xpbd_projector/CombinedConstraintProjector.hpp"

#include "solver/constraint/HydrostaticConstraint.hpp"
#include "solver/constraint/DeviatoricConstraint.hpp"
#include "solver/constraint/StaticDeformableCollisionConstraint.hpp"
#include "solver/constraint/RigidDeformableCollisionConstraint.hpp"

#include "solver/xpbd_solver/XPBDGaussSeidelSolver.hpp"
#include "solver/xpbd_solver/XPBDJacobiSolver.hpp"
#include "solver/xpbd_solver/XPBDParallelJacobiSolver.hpp"

#include "common/TypeList.hpp"

#include <variant>
#include <type_traits>

// namespace Solver
// {

//     template<typename ...Projectors>
//     class XPBDGaussSeidelSolver;

//     template<typename ...Projectors>
//     class XPBDJacobiSolver;

//     template<typename ...Projectors>
//     class XPBDParallelJacobiSolver;
// }

template<typename ...Projectors>
struct XPBDMeshObjectConstraintType
{
    using projector_type_list = TypeList<Projectors...>;
    using constraint_type_list = typename ConcatenateTypeLists<typename Projectors::ConstraintTypes...>::type;

    // two ConstraintTypes are equal if their Projector types are equal
    template<typename ...OtherProjectors>
    bool operator==(const XPBDMeshObjectConstraintType<OtherProjectors...>&) const
    {
        return std::is_same_v<TypeList<Projectors...>, TypeList<OtherProjectors...>>;
    }
};

// Declare XPBDMeshObject constraint configurations
struct XPBDMeshObjectConstraintTypes
{
    // typedefs for readability
    private:
    using DevProjector = Solver::ConstraintProjector<Solver::DeviatoricConstraint>;
    using HydProjector = Solver::ConstraintProjector<Solver::HydrostaticConstraint>;
    using DevHydProjector = Solver::CombinedConstraintProjector<Solver::DeviatoricConstraint, Solver::HydrostaticConstraint>;
    using StatCollProjector = Solver::ConstraintProjector<Solver::StaticDeformableCollisionConstraint>;
    using RigiCollProjector = Solver::ConstraintProjector<Solver::RigidDeformableCollisionConstraint>;

    // public typedefs represent XPBDMeshObject constraint configurations
    public:
    using StableNeohookean = XPBDMeshObjectConstraintType<DevProjector, HydProjector, StatCollProjector, RigiCollProjector>;
    using StableNeohookeanCombined = XPBDMeshObjectConstraintType<DevHydProjector, StatCollProjector, RigiCollProjector>;

    using type_list = TypeList<StableNeohookean, StableNeohookeanCombined>;
    using variant_type = std::variant<StableNeohookean, StableNeohookeanCombined>;

    constexpr static StableNeohookean STABLE_NEOHOOKEAN{};
    constexpr static StableNeohookeanCombined STABLE_NEOHOOKEAN_COMBINED{};
};

template<typename ProjectorTypeList> struct XPBDMeshObjectSolverTypes;

template<typename ...Projectors>
struct XPBDMeshObjectSolverTypes<TypeList<Projectors...>>
{
    using GaussSeidel = Solver::XPBDGaussSeidelSolver<Projectors...>;
    using Jacobi = Solver::XPBDJacobiSolver<Projectors...>;
    using ParallelJacobi = Solver::XPBDParallelJacobiSolver<Projectors...>;

    using types = TypeList<GaussSeidel, Jacobi, ParallelJacobi>;
    using variant_type = std::variant<GaussSeidel, Jacobi, ParallelJacobi>;
};



// Declare XPBDMeshObject solver types



#endif // __XPBD_TYPEDEFS_HPP