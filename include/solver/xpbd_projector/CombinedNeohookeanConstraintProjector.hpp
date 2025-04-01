#ifndef __COMBINED_NEOHOOKEAN_CONSTRAINT_PROJECTOR_HPP
#define __COMBINED_NEOHOOKEAN_CONSTRAINT_PROJECTOR_HPP

#include "solver/xpbd_projector/CombinedConstraintProjector.hpp"
#include "solver/constraint/HydrostaticConstraint.hpp"
#include "solver/constraint/DeviatoricConstraint.hpp"
#include "solver/xpbd_solver/XPBDSolverUpdates.hpp"

namespace Solver
{

template<>
void CombinedConstraintProjector<DeviatoricConstraint, HydrostaticConstraint>::project(CoordinateUpdate* coordinate_updates_ptr);

} // namespace Solver

#endif // __COMBINED_NEOHOOKEAN_CONSTRAINT_PROJECTOR_HPP