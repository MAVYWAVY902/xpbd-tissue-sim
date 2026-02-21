cmake_minimum_required(VERSION 3.13)
project(XPBDSim)

set(HAPTICS_INCLUDE_DIR "../../include/haptics")
set(HAPTICS_SRC_DIR "./")

set(HAPTICS_HEADERS
    ${HAPTICS_INCLUDE_DIR}/HapticDeviceManager.hpp
)

set(HAPTICS_SOURCES
    ${HAPTICS_SRC_DIR}/HapticDeviceManager.cpp
)

target_sources(Haptics
    PRIVATE
        ${HAPTICS_HEADERS}
        ${HAPTICS_SOURCES}       
)

install(TARGETS Haptics EXPORT Haptics-targets
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME})
install(FILES ${HAPTICS_HEADERS}
    DESTINATION include/${PROJECT_NAME}/haptics)


######################################
# Haply Haptics (Inverse3 device)
######################################
set(HAPLY_HAPTICS_HEADERS
    ${HAPTICS_INCLUDE_DIR}/HaplyInverse3Device.hpp
)

set(HAPLY_HAPTICS_SOURCES
    ${HAPTICS_SRC_DIR}/HaplyInverse3Device.cpp
)

target_sources(HaplyHaptics
    PRIVATE
        ${HAPLY_HAPTICS_HEADERS}
        ${HAPLY_HAPTICS_SOURCES}
)