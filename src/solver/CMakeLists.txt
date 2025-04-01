cmake_minimum_required(VERSION 3.13)
project(MyProject)

set(SOLVER_INCLUDE_DIR "../../include/solver")
set(SOLVER_SRC_DIR "./")

target_sources(Solver
    PRIVATE
        ${SOLVER_SRC_DIR}/constraint/DeviatoricConstraint.cpp
        ${SOLVER_SRC_DIR}/constraint/HydrostaticConstraint.cpp
        ${SOLVER_SRC_DIR}/constraint/StaticDeformableCollisionConstraint.cpp
        ${SOLVER_SRC_DIR}/constraint/RigidDeformableCollisionConstraint.cpp

        ${SOLVER_SRC_DIR}/xpbd_projector/CombinedNeohookeanConstraintProjector.cpp

        # ${SOLVER_SRC_DIR}/XPBDSolver.cpp
        # ${SOLVER_SRC_DIR}/XPBDGaussSeidelSolver.cpp
        # ${SOLVER_SRC_DIR}/XPBDJacobiSolver.cpp
        # ${SOLVER_SRC_DIR}/XPBDParallelJacobiSolver.cpp
)