#ifndef __XPBD_OBJECT_FACTORY_HPP
#define __XPBD_OBJECT_FACTORY_HPP

#include "simobject/XPBDMeshObject.hpp"

#include "config/simobject/XPBDMeshObjectConfig.hpp"
#include "config/simobject/FirstOrderXPBDMeshObjectConfig.hpp"

#include "common/XPBDEnumTypes.hpp"
#include "common/XPBDTypedefs.hpp"

// create a class with static methods to create XPBDMeshObject with appropriate template parameters given config object
class XPBDObjectFactory
{
    public:
    static std::unique_ptr<Sim::XPBDMeshObject_Base> createXPBDMeshObject(const Sim::Simulation* sim, const Config::XPBDMeshObjectConfig* config)
    {
        XPBDMeshObjectConstraintConfigurationEnum constraint_type = config->constraintType();
        if (constraint_type == XPBDMeshObjectConstraintConfigurationEnum::STABLE_NEOHOOKEAN)
        {
            return _createXPBDMeshObject<XPBDMeshObjectConstraintConfigurations<false>::StableNeohookean>(sim, config);
        }
        else if (constraint_type == XPBDMeshObjectConstraintConfigurationEnum::STABLE_NEOHOOKEAN_COMBINED)
        {
            return _createXPBDMeshObject<XPBDMeshObjectConstraintConfigurations<false>::StableNeohookeanCombined>(sim, config);
        }
        else
        {
            assert(0); // something's wrong
            return std::unique_ptr<Sim::XPBDMeshObject_Base>{};
        }
    }

    static std::unique_ptr<Sim::FirstOrderXPBDMeshObject_Base> createFirstOrderXPBDMeshObject(const Sim::Simulation* sim, const Config::FirstOrderXPBDMeshObjectConfig* config)
    {
        XPBDMeshObjectConstraintConfigurationEnum constraint_type = config->constraintType();
        if (constraint_type == XPBDMeshObjectConstraintConfigurationEnum::STABLE_NEOHOOKEAN)
        {
            return _createFirstOrderXPBDMeshObject<XPBDMeshObjectConstraintConfigurations<true>::StableNeohookean>(sim, config);
        }
        else if (constraint_type == XPBDMeshObjectConstraintConfigurationEnum::STABLE_NEOHOOKEAN_COMBINED)
        {
            return _createFirstOrderXPBDMeshObject<XPBDMeshObjectConstraintConfigurations<true>::StableNeohookeanCombined>(sim, config);
        }
        else
        {
            assert(0); // something's wrong
            return std::unique_ptr<Sim::FirstOrderXPBDMeshObject_Base>{};
        }
    }

    private:
    template<typename ConstraintType>
    static std::unique_ptr<Sim::XPBDMeshObject_Base> _createXPBDMeshObject(const Sim::Simulation* sim, const Config::XPBDMeshObjectConfig* config)
    {
        XPBDObjectSolverTypeEnum solver_type = config->solverType();
        if (solver_type == XPBDObjectSolverTypeEnum::GAUSS_SEIDEL)
        {
            using SolverType = typename XPBDObjectSolverTypes<false, typename ConstraintType::projector_type_list>::GaussSeidel;
            return std::make_unique<Sim::XPBDMeshObject<SolverType, typename ConstraintType::constraint_type_list>>(sim, config);
        }
        else if (solver_type == XPBDObjectSolverTypeEnum::JACOBI)
        {
            using SolverType = typename XPBDObjectSolverTypes<false, typename ConstraintType::projector_type_list>::Jacobi;
            return std::make_unique<Sim::XPBDMeshObject<SolverType, typename ConstraintType::constraint_type_list>>(sim, config);
        }
        else if (solver_type == XPBDObjectSolverTypeEnum::PARALLEL_JACOBI)
        {
            using SolverType = typename XPBDObjectSolverTypes<false, typename ConstraintType::projector_type_list>::ParallelJacobi;
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
    static std::unique_ptr<Sim::FirstOrderXPBDMeshObject_Base> _createFirstOrderXPBDMeshObject(const Sim::Simulation* sim, const Config::FirstOrderXPBDMeshObjectConfig* config)
    {
        XPBDObjectSolverTypeEnum solver_type = config->solverType();
        if (solver_type == XPBDObjectSolverTypeEnum::GAUSS_SEIDEL)
        {
            using SolverType = typename XPBDObjectSolverTypes<true, typename ConstraintType::projector_type_list>::GaussSeidel;
            return std::make_unique<Sim::FirstOrderXPBDMeshObject<SolverType, typename ConstraintType::constraint_type_list>>(sim, config);
        }
        else if (solver_type == XPBDObjectSolverTypeEnum::JACOBI)
        {
            using SolverType = typename XPBDObjectSolverTypes<true, typename ConstraintType::projector_type_list>::Jacobi;
            return std::make_unique<Sim::FirstOrderXPBDMeshObject<SolverType, typename ConstraintType::constraint_type_list>>(sim, config);
        }
        else if (solver_type == XPBDObjectSolverTypeEnum::PARALLEL_JACOBI)
        {
            using SolverType = typename XPBDObjectSolverTypes<true, typename ConstraintType::projector_type_list>::ParallelJacobi;
            return std::make_unique<Sim::FirstOrderXPBDMeshObject<SolverType, typename ConstraintType::constraint_type_list>>(sim, config);
        }
        else
        {
            assert(0); // something's wrong
            return std::unique_ptr<Sim::FirstOrderXPBDMeshObject_Base>{};
        }
    }
};

#endif // __XPBD_OBJECT_FACTORY_HPP