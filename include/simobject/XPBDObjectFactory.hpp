#ifndef __XPBD_OBJECT_FACTORY_HPP
#define __XPBD_OBJECT_FACTORY_HPP

#include "simobject/XPBDMeshObject.hpp"
#include "simobject/FirstOrderXPBDMeshObject.hpp"

#include "config/XPBDMeshObjectConfig.hpp"
#include "config/FirstOrderXPBDMeshObjectConfig.hpp"

#include "common/XPBDTypedefs.hpp"

// create a class with static methods to create XPBDMeshObject with appropriate template parameters given config object
class XPBDObjectFactory
{
    public:
    // TODO: using variant type for ConstraintType, and using enum for SolverType. Pick one
    static std::unique_ptr<Sim::XPBDMeshObject_Base> createXPBDMeshObject(const Sim::Simulation* sim, const XPBDMeshObjectConfig* config)
    {
        XPBDMeshObjectConstraintTypes::variant_type constraint_type = config->constraintType();
        if (std::holds_alternative<XPBDMeshObjectConstraintTypes::StableNeohookean>(constraint_type))
        {
            return _createXPBDMeshObject<XPBDMeshObjectConstraintTypes::StableNeohookean>(sim, config);
        }
        else if (std::holds_alternative<XPBDMeshObjectConstraintTypes::StableNeohookeanCombined>(constraint_type))
        {
            return _createXPBDMeshObject<XPBDMeshObjectConstraintTypes::StableNeohookeanCombined>(sim, config);
        }
        else
        {
            assert(0); // something's wrong
            return std::unique_ptr<Sim::XPBDMeshObject_Base>{};
        }
    }

    static std::unique_ptr<Sim::XPBDMeshObject_Base> createFirstOrderXPBDMeshObject(const Sim::Simulation* sim, const FirstOrderXPBDMeshObjectConfig* config)
    {
        XPBDMeshObjectConstraintTypes::variant_type constraint_type = config->constraintType();
        if (std::holds_alternative<XPBDMeshObjectConstraintTypes::StableNeohookean>(constraint_type))
        {
            return _createFirstOrderXPBDMeshObject<XPBDMeshObjectConstraintTypes::StableNeohookean>(sim, config);
        }
        else if (std::holds_alternative<XPBDMeshObjectConstraintTypes::StableNeohookeanCombined>(constraint_type))
        {
            return _createFirstOrderXPBDMeshObject<XPBDMeshObjectConstraintTypes::StableNeohookeanCombined>(sim, config);
        }
        else
        {
            assert(0); // something's wrong
            return std::unique_ptr<Sim::XPBDMeshObject_Base>{};
        }
    }

    private:
    template<typename ConstraintType>
    static std::unique_ptr<Sim::XPBDMeshObject_Base> _createXPBDMeshObject(const Sim::Simulation* sim, const XPBDMeshObjectConfig* config)
    {
        XPBDSolverType solver_type = config->solverType().value();
        if (solver_type == XPBDSolverType::GAUSS_SEIDEL)
        {
            using SolverType = typename XPBDMeshObjectSolverTypes<typename ConstraintType::projector_type_list>::GaussSeidel;
            typename ConstraintType::constraint_type_list constraint_type_list;
            return std::make_unique<Sim::XPBDMeshObject<SolverType, typename ConstraintType::constraint_type_list>>(constraint_type_list, sim, config);
        }
        else if (solver_type == XPBDSolverType::JACOBI)
        {
            using SolverType = typename XPBDMeshObjectSolverTypes<typename ConstraintType::projector_type_list>::Jacobi;
            typename ConstraintType::constraint_type_list constraint_type_list;
            return std::make_unique<Sim::XPBDMeshObject<SolverType, typename ConstraintType::constraint_type_list>>(constraint_type_list, sim, config);
        }
        else if (solver_type == XPBDSolverType::PARALLEL_JACOBI)
        {
            using SolverType = typename XPBDMeshObjectSolverTypes<typename ConstraintType::projector_type_list>::ParallelJacobi;
            typename ConstraintType::constraint_type_list constraint_type_list;
            return std::make_unique<Sim::XPBDMeshObject<SolverType, typename ConstraintType::constraint_type_list>>(constraint_type_list, sim, config);
        }
        else
        {
            assert(0); // something's wrong
            return std::unique_ptr<Sim::XPBDMeshObject_Base>{};
        }
    }

    // TODO: create proper type for First Order (right now just duplicating what's in _createXPBDMeshObject)
    template<typename ConstraintType>
    static std::unique_ptr<Sim::XPBDMeshObject_Base> _createFirstOrderXPBDMeshObject(const Sim::Simulation* sim, const FirstOrderXPBDMeshObjectConfig* config)
    {
        XPBDSolverType solver_type = config->solverType().value();
        if (solver_type == XPBDSolverType::GAUSS_SEIDEL)
        {
            using SolverType = typename XPBDMeshObjectSolverTypes<typename ConstraintType::projector_type_list>::GaussSeidel;
            typename ConstraintType::constraint_type_list constraint_type_list;
            return std::make_unique<Sim::FirstOrderXPBDMeshObject<SolverType, typename ConstraintType::constraint_type_list>>(constraint_type_list, sim, config);
        }
        else if (solver_type == XPBDSolverType::JACOBI)
        {
            using SolverType = typename XPBDMeshObjectSolverTypes<typename ConstraintType::projector_type_list>::Jacobi;
            typename ConstraintType::constraint_type_list constraint_type_list;
            return std::make_unique<Sim::FirstOrderXPBDMeshObject<SolverType, typename ConstraintType::constraint_type_list>>(constraint_type_list, sim, config);
        }
        else if (solver_type == XPBDSolverType::PARALLEL_JACOBI)
        {
            using SolverType = typename XPBDMeshObjectSolverTypes<typename ConstraintType::projector_type_list>::ParallelJacobi;
            typename ConstraintType::constraint_type_list constraint_type_list;
            return std::make_unique<Sim::FirstOrderXPBDMeshObject<SolverType, typename ConstraintType::constraint_type_list>>(constraint_type_list, sim, config);
        }
        else
        {
            assert(0); // something's wrong
            return std::unique_ptr<Sim::XPBDMeshObject_Base>{};
        }
    }
};

#endif // __XPBD_OBJECT_FACTORY_HPP