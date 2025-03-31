#ifndef __XPBD_ENUM_TYPES_HPP
#define __XPBD_ENUM_TYPES_HPP

// TODO: is this file needed? Introduced to solve some circular dependencies...

enum class XPBDObjectSolverTypeEnum
{
    GAUSS_SEIDEL,
    JACOBI,
    PARALLEL_JACOBI
};

enum class XPBDMeshObjectConstraintConfigurationEnum
{
    STABLE_NEOHOOKEAN,
    STABLE_NEOHOOKEAN_COMBINED
};

// TODO: rename to something slightly more descriptive, i.e. XPBDSolverResidualPolicy
enum class XPBDSolverResidualPolicyEnum
{
    NEVER=0,
    EVERY_SUBSTEP,
    EVERY_ITERATION
};

#endif // __XPBD_ENUM_TYPES_HPP