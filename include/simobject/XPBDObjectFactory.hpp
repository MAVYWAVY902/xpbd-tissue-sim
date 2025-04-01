#ifndef __XPBD_OBJECT_FACTORY_HPP
#define __XPBD_OBJECT_FACTORY_HPP

#include "simobject/XPBDMeshObject.hpp"
#include "simobject/FirstOrderXPBDMeshObject.hpp"

#include "config/XPBDMeshObjectConfig.hpp"
#include "config/FirstOrderXPBDMeshObjectConfig.hpp"

#include "common/XPBDEnumTypes.hpp"
#include "common/XPBDTypedefs.hpp"

// create a class with static methods to create XPBDMeshObject with appropriate template parameters given config object
class XPBDObjectFactory
{
    public:
    static std::unique_ptr<Sim::XPBDMeshObject_Base> createXPBDMeshObject(const Sim::Simulation* sim, const XPBDMeshObjectConfig* config)
    {
        XPBDMeshObjectConstraintConfigurationEnum constraint_type = config->constraintType();
        if (constraint_type == XPBDMeshObjectConstraintConfigurationEnum::STABLE_NEOHOOKEAN)
        {
            return _createXPBDMeshObject<XPBDMeshObjectConstraintConfigurations::StableNeohookean>(sim, config);
        }
        else if (constraint_type == XPBDMeshObjectConstraintConfigurationEnum::STABLE_NEOHOOKEAN_COMBINED)
        {
            return _createXPBDMeshObject<XPBDMeshObjectConstraintConfigurations::StableNeohookeanCombined>(sim, config);
        }
        else
        {
            assert(0); // something's wrong
            return std::unique_ptr<Sim::XPBDMeshObject_Base>{};
        }
    }

    static std::unique_ptr<Sim::XPBDMeshObject_Base> createFirstOrderXPBDMeshObject(const Sim::Simulation* sim, const FirstOrderXPBDMeshObjectConfig* config)
    {
        XPBDMeshObjectConstraintConfigurationEnum constraint_type = config->constraintType();
        if (constraint_type == XPBDMeshObjectConstraintConfigurationEnum::STABLE_NEOHOOKEAN)
        {
            return _createFirstOrderXPBDMeshObject<FirstOrderXPBDMeshObjectConstraintConfigurations::StableNeohookean>(sim, config);
        }
        else if (constraint_type == XPBDMeshObjectConstraintConfigurationEnum::STABLE_NEOHOOKEAN_COMBINED)
        {
            return _createFirstOrderXPBDMeshObject<FirstOrderXPBDMeshObjectConstraintConfigurations::StableNeohookeanCombined>(sim, config);
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
        XPBDObjectSolverTypeEnum solver_type = config->solverType().value();
        if (solver_type == XPBDObjectSolverTypeEnum::GAUSS_SEIDEL)
        {
            using SolverType = typename XPBDObjectSolverTypes<typename ConstraintType::projector_type_list>::GaussSeidel;
            return std::make_unique<Sim::XPBDMeshObject<SolverType, typename ConstraintType::constraint_type_list>>(sim, config);
        }
        else if (solver_type == XPBDObjectSolverTypeEnum::JACOBI)
        {
            using SolverType = typename XPBDObjectSolverTypes<typename ConstraintType::projector_type_list>::Jacobi;
            return std::make_unique<Sim::XPBDMeshObject<SolverType, typename ConstraintType::constraint_type_list>>(sim, config);
        }
        else if (solver_type == XPBDObjectSolverTypeEnum::PARALLEL_JACOBI)
        {
            using SolverType = typename XPBDObjectSolverTypes<typename ConstraintType::projector_type_list>::ParallelJacobi;
            return std::make_unique<Sim::XPBDMeshObject<SolverType, typename ConstraintType::constraint_type_list>>(sim, config);
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
        XPBDObjectSolverTypeEnum solver_type = config->solverType().value();
        if (solver_type == XPBDObjectSolverTypeEnum::GAUSS_SEIDEL)
        {
            using SolverType = typename FirstOrderXPBDObjectSolverTypes<typename ConstraintType::projector_type_list>::GaussSeidel;
            return std::make_unique<Sim::FirstOrderXPBDMeshObject<SolverType, typename ConstraintType::constraint_type_list>>(sim, config);
        }
        else if (solver_type == XPBDObjectSolverTypeEnum::JACOBI)
        {
            using SolverType = typename FirstOrderXPBDObjectSolverTypes<typename ConstraintType::projector_type_list>::Jacobi;
            return std::make_unique<Sim::FirstOrderXPBDMeshObject<SolverType, typename ConstraintType::constraint_type_list>>(sim, config);
        }
        else if (solver_type == XPBDObjectSolverTypeEnum::PARALLEL_JACOBI)
        {
            using SolverType = typename FirstOrderXPBDObjectSolverTypes<typename ConstraintType::projector_type_list>::ParallelJacobi;
            return std::make_unique<Sim::FirstOrderXPBDMeshObject<SolverType, typename ConstraintType::constraint_type_list>>(sim, config);
        }
        else
        {
            assert(0); // something's wrong
            return std::unique_ptr<Sim::XPBDMeshObject_Base>{};
        }
    }
};

#endif // __XPBD_OBJECT_FACTORY_HPP