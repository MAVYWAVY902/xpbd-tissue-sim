#ifndef __XPBD_ENUM_TYPES_HPP
#define __XPBD_ENUM_TYPES_HPP

// TODO: is this file needed? Introduced to solve some circular dependencies...

enum class XPBDSolverType
{
    GAUSS_SEIDEL,
    JACOBI,
    PARALLEL_JACOBI
};

// TODO: rename to something slightly more descriptive, i.e. XPBDSolverResidualPolicy
enum class XPBDResidualPolicy
{
    NEVER=0,
    EVERY_SUBSTEP,
    EVERY_ITERATION
};

#endif // __XPBD_ENUM_TYPES_HPP