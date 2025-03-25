cmake_minimum_required(VERSION 3.13)
project(MyProject)

set(SOLVER_INCLUDE_DIR "../../include/solver")
set(SOLVER_SRC_DIR "./")

target_sources(Solver
    PRIVATE
        ${SOLVER_INCLUDE_DIR}/Constraint.hpp
        ${SOLVER_INCLUDE_DIR}/ElementConstraint.hpp
        ${SOLVER_INCLUDE_DIR}/DeviatoricConstraint.hpp
        ${SOLVER_INCLUDE_DIR}/HydrostaticConstraint.hpp
        ${SOLVER_INCLUDE_DIR}/ConstraintProjector.hpp
        ${SOLVER_INCLUDE_DIR}/ConstraintProjectorDecorator.hpp
        ${SOLVER_INCLUDE_DIR}/CombinedNeohookeanConstraintProjector.hpp
        ${SOLVER_INCLUDE_DIR}/XPBDSolver.hpp
        ${SOLVER_INCLUDE_DIR}/XPBDGaussSeidelSolver.hpp
        ${SOLVER_INCLUDE_DIR}/XPBDJacobiSolver.hpp
        ${SOLVER_INCLUDE_DIR}/XPBDParallelJacobiSolver.hpp
        # ${SOLVER_SRC_DIR}/XPBDSolver.cpp
        # ${SOLVER_SRC_DIR}/XPBDGaussSeidelSolver.cpp
        # ${SOLVER_SRC_DIR}/XPBDJacobiSolver.cpp
        # ${SOLVER_SRC_DIR}/XPBDParallelJacobiSolver.cpp
)