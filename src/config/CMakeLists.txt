cmake_minimum_required(VERSION 3.13)
project(XPBDSim)

set_target_properties(Config PROPERTIES LINKER_LANGUAGE CXX)

set(CONFIG_INCLUDE_DIR "../../include/config")
set(CONFIG_SRC_DIR "./")

set(CONFIG_HEADERS
    ${CONFIG_INCLUDE_DIR}/BeamSimulationConfig.hpp
    ${CONFIG_INCLUDE_DIR}/BeamStretchSimulationConfig.hpp
    ${CONFIG_INCLUDE_DIR}/Config.hpp
    ${CONFIG_INCLUDE_DIR}/ElasticMaterialConfig.hpp
    ${CONFIG_INCLUDE_DIR}/FirstOrderXPBDMeshObjectConfig.hpp
    ${CONFIG_INCLUDE_DIR}/InitialDeformationSimulationConfig.hpp
    ${CONFIG_INCLUDE_DIR}/MeshObjectConfig.hpp
    ${CONFIG_INCLUDE_DIR}/ObjectConfig.hpp
    ${CONFIG_INCLUDE_DIR}/OutputSimulationConfig.hpp
    ${CONFIG_INCLUDE_DIR}/ResidualSimulationConfig.hpp
    ${CONFIG_INCLUDE_DIR}/RigidMeshObjectConfig.hpp
    ${CONFIG_INCLUDE_DIR}/RigidObjectConfig.hpp
    ${CONFIG_INCLUDE_DIR}/RigidPrimitiveConfigs.hpp
    ${CONFIG_INCLUDE_DIR}/SimulationConfig.hpp
    ${CONFIG_INCLUDE_DIR}/StaticLoadingSimulationConfig.hpp
    ${CONFIG_INCLUDE_DIR}/TissueGraspingSimulationConfig.hpp
    ${CONFIG_INCLUDE_DIR}/VirtuosoArmConfig.hpp
    ${CONFIG_INCLUDE_DIR}/VirtuosoRobotConfig.hpp
    ${CONFIG_INCLUDE_DIR}/VirtuosoSimulationConfig.hpp
    ${CONFIG_INCLUDE_DIR}/VirtuosoTissueGraspingSimulationConfig.hpp
    ${CONFIG_INCLUDE_DIR}/XPBDMeshObjectConfig.hpp
)

set(CONFIG_SOURCES
)

target_sources(Config
    PRIVATE
        ${CONFIG_HEADERS}
        ${CONFIG_SOURCES}       
)

install(TARGETS Config EXPORT Config-targets
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME})
install(FILES ${CONFIG_HEADERS}
    DESTINATION include/${PROJECT_NAME}/config)