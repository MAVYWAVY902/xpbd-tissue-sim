cmake_minimum_required(VERSION 3.13)
project(XPBDSim)

set(SOLVER_INCLUDE_DIR "../../include/solver")
set(SOLVER_SRC_DIR "./")

set(SOLVER_HEADERS
    ${SOLVER_INCLUDE_DIR}/constraint/AttachmentConstraint.hpp
    ${SOLVER_INCLUDE_DIR}/constraint/CollisionConstraint.hpp
    ${SOLVER_INCLUDE_DIR}/constraint/Constraint.hpp
    ${SOLVER_INCLUDE_DIR}/constraint/DeviatoricConstraint.hpp
    ${SOLVER_INCLUDE_DIR}/constraint/ElementConstraint.hpp
    ${SOLVER_INCLUDE_DIR}/constraint/HydrostaticConstraint.hpp
    ${SOLVER_INCLUDE_DIR}/constraint/RigidBodyConstraint.hpp
    ${SOLVER_INCLUDE_DIR}/constraint/RigidDeformableCollisionConstraint.hpp
    ${SOLVER_INCLUDE_DIR}/constraint/StaticDeformableCollisionConstraint.hpp
    ${SOLVER_INCLUDE_DIR}/constraint/DeformableDeformableCollisionConstraint.hpp
    ${SOLVER_INCLUDE_DIR}/constraint/InterObjectDeformableCollisionConstraint.hpp

    ${SOLVER_INCLUDE_DIR}/xpbd_projector/CombinedConstraintProjector.hpp
    ${SOLVER_INCLUDE_DIR}/xpbd_projector/CombinedNeohookeanConstraintProjector.hpp
    ${SOLVER_INCLUDE_DIR}/xpbd_projector/ConstraintProjector.hpp
    ${SOLVER_INCLUDE_DIR}/xpbd_projector/RigidBodyConstraintProjector.hpp

    ${SOLVER_INCLUDE_DIR}/xpbd_solver/XPBDGaussSeidelSolver.hpp
    ${SOLVER_INCLUDE_DIR}/xpbd_solver/XPBDJacobiSolver.hpp
    ${SOLVER_INCLUDE_DIR}/xpbd_solver/XPBDParallelJacobiSolver.hpp
    ${SOLVER_INCLUDE_DIR}/xpbd_solver/XPBDSolver.hpp
    ${SOLVER_INCLUDE_DIR}/xpbd_solver/XPBDSolverUpdates.hpp
    ${SOLVER_INCLUDE_DIR}/constraint/NerveStretchConstraint.hpp
    ${SOLVER_INCLUDE_DIR}/constraint/NerveBendingConstraint.hpp
    ${SOLVER_INCLUDE_DIR}/constraint/NerveTumorAdhesionConstraint.hpp
    ${SOLVER_INCLUDE_DIR}/constraint/NerveTumorBond.hpp
    ${SOLVER_INCLUDE_DIR}/constraint/InterDeformDeformAdhesionConstraint.hpp
    ${SOLVER_INCLUDE_DIR}/constraint/RigidDeformAdhesionConstraint.hpp

    # Graph Coloring for VBD Gauss-Seidel
    ${SOLVER_INCLUDE_DIR}/GraphColoring.hpp
    ${SOLVER_INCLUDE_DIR}/TetMeshVertexGraph.hpp
)

set(SOLVER_SOURCES
    ${SOLVER_SRC_DIR}/constraint/DeviatoricConstraint.cpp
    ${SOLVER_SRC_DIR}/constraint/HydrostaticConstraint.cpp
    ${SOLVER_SRC_DIR}/constraint/StaticDeformableCollisionConstraint.cpp
    ${SOLVER_SRC_DIR}/constraint/RigidDeformableCollisionConstraint.cpp
    ${SOLVER_SRC_DIR}/constraint/DeformableDeformableCollisionConstraint.cpp
    ${SOLVER_SRC_DIR}/constraint/InterObjectDeformableCollisionConstraint.cpp
    ${SOLVER_SRC_DIR}/constraint/AttachmentConstraint.cpp

    ${SOLVER_SRC_DIR}/xpbd_projector/CombinedNeohookeanConstraintProjector.cpp
    ${SOLVER_SRC_DIR}/constraint/NerveStretchConstraint.cpp
    ${SOLVER_SRC_DIR}/constraint/NerveBendingConstraint.cpp
    ${SOLVER_SRC_DIR}/constraint/NerveTumorAdhesionConstraint.cpp
    ${SOLVER_SRC_DIR}/constraint/InterDeformDeformAdhesionConstraint.cpp
    ${SOLVER_SRC_DIR}/constraint/RigidDeformAdhesionConstraint.cpp

    # Graph Coloring for VBD Gauss-Seidel
    ${SOLVER_SRC_DIR}/GraphColoring.cpp
    ${SOLVER_SRC_DIR}/TetMeshVertexGraph.cpp
)

target_sources(Solver
    PRIVATE
        ${SOLVER_HEADERS}
        ${SOLVER_SOURCES}
)

# Link OpenMP for both Jacobi and VBD solvers (conditionally enabled)
if(OpenMP_CXX_FOUND)
    target_link_libraries(Solver PUBLIC OpenMP::OpenMP_CXX)
    # Define ENABLE_JACOBI_OPENMP to activate OpenMP code in XPBDJacobiSolver
    target_compile_definitions(Solver PUBLIC ENABLE_JACOBI_OPENMP)
    # Define ENABLE_VBD_OPENMP to activate OpenMP code in VBD solver
    target_compile_definitions(Solver PUBLIC ENABLE_VBD_OPENMP)
    message(STATUS "Jacobi solver: OpenMP parallelization ENABLED")
    message(STATUS "VBD solver: OpenMP parallelization ENABLED")
else()
    message(STATUS "Jacobi solver: OpenMP not found, running serially")
    message(STATUS "VBD solver: OpenMP not found, running serially")
endif()

# Enable VBD line search for stability (can be disabled for performance if needed)
# Line search adaptively adjusts step size to prevent overshooting
target_compile_definitions(Solver PUBLIC ENABLE_VBD_LINE_SEARCH)
message(STATUS "VBD solver: Line search ENABLED for improved stability")

install(TARGETS Solver EXPORT Solver-targets
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME})
install(FILES ${SOLVER_HEADERS}
    DESTINATION include/${PROJECT_NAME}/solver)
