cmake_minimum_required(VERSION 3.13)
project(XPBDSim)

set(SIMULATION_INCLUDE_DIR "../../include/simulation")
set(SIMULATION_SRC_DIR "./")


######################################
# Residual Simulation
######################################

set(RESIDUAL_SIMULATION_HEADERS
    "${SIMULATION_INCLUDE_DIR}/Simulation.hpp"
    "${SIMULATION_INCLUDE_DIR}/OutputSimulation.hpp"
    "${SIMULATION_INCLUDE_DIR}/ResidualSimulation.hpp"
)
set(RESIDUAL_SIMULATION_SOURCES
    "${SIMULATION_SRC_DIR}/Simulation.cpp"
    "${SIMULATION_SRC_DIR}/OutputSimulation.cpp"
    "${SIMULATION_SRC_DIR}/ResidualSimulation.cpp"
)

target_sources(ResidualSimulation
    PRIVATE
        ${RESIDUAL_SIMULATION_HEADERS}
        ${RESIDUAL_SIMULATION_SOURCES}
        
)
install(TARGETS ResidualSimulation EXPORT ResidualSimulation-targets
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME})
install(FILES ${RESIDUAL_SIMULATION_HEADERS}
    DESTINATION include/${PROJECT_NAME}/simulation)



######################################
# Beam Simulation
######################################

target_sources(BeamSimulation
    PRIVATE
        ${SIMULATION_INCLUDE_DIR}/BeamSimulation.hpp
        ${SIMULATION_SRC_DIR}/BeamSimulation.cpp
)


######################################
# Virtuoso Simulation
######################################
set(VIRTUOSO_SIMULATION_HEADERS
    "${SIMULATION_INCLUDE_DIR}/VirtuosoSimulation.hpp"
    "${SIMULATION_INCLUDE_DIR}/VirtuosoTissueGraspingSimulation.hpp"
)

set(VIRTUOSO_SIMULATION_SOURCES
    "${SIMULATION_SRC_DIR}/VirtuosoSimulation.cpp"
    "${SIMULATION_SRC_DIR}/VirtuosoTissueGraspingSimulation.cpp"
)

target_sources(VirtuosoSimulation
    PRIVATE
        ${VIRTUOSO_SIMULATION_HEADERS}
        ${VIRTUOSO_SIMULATION_SOURCES}
)
install(TARGETS VirtuosoSimulation EXPORT VirtuosoSimulation-targets
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
)
install(FILES ${VIRTUOSO_SIMULATION_HEADERS}
    DESTINATION include/${PROJECT_NAME}/simulation)


# target_sources(BeamStretchSimulation
#     PRIVATE
#         ${SIMULATION_INCLUDE_DIR}/BeamStretchSimulation.hpp
#         ${SIMULATION_SRC_DIR}/BeamStretchSimulation.cpp
# )

# target_sources(InitialDeformationSimulation
#     PRIVATE
#         ${SIMULATION_INCLUDE_DIR}/InitialDeformationSimulation.hpp
#         ${SIMULATION_SRC_DIR}/InitialDeformationSimulation.cpp
# )

# target_sources(TissueGraspingSimulation
#     PRIVATE
#         ${SIMULATION_INCLUDE_DIR}/TissueGraspingSimulation.hpp
#         ${SIMULATION_SRC_DIR}/TissueGraspingSimulation.cpp
# )