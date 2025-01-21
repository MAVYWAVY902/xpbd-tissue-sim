cmake_minimum_required(VERSION 3.13)
project(MyProject)

set(SIMULATION_INCLUDE_DIR "../../include/simulation")
set(SIMULATION_SRC_DIR "./")

target_sources(ResidualSimulation
    PRIVATE
        ${SIMULATION_INCLUDE_DIR}/Simulation.hpp
        ${SIMULATION_INCLUDE_DIR}/OutputSimulation.hpp
        ${SIMULATION_INCLUDE_DIR}/ResidualSimulation.hpp
        ${SIMULATION_SRC_DIR}/Simulation.cpp
        ${SIMULATION_SRC_DIR}/OutputSimulation.cpp
        ${SIMULATION_SRC_DIR}/ResidualSimulation.cpp
)

target_sources(BeamSimulation
    PRIVATE
        ${SIMULATION_INCLUDE_DIR}/BeamSimulation.hpp
        ${SIMULATION_SRC_DIR}/BeamSimulation.cpp
)

target_sources(VirtuosoSimulation
    PRIVATE
        ${SIMULATION_INCLUDE_DIR}/VirtuosoSimulation.hpp
        ${SIMULATION_SRC_DIR}/VirtuosoSimulation.cpp
)

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