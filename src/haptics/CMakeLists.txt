cmake_minimum_required(VERSION 3.13)
project(MyProject)

set(HAPTICS_INCLUDE_DIR "../../include/haptics")
set(HAPTICS_SRC_DIR "./")

target_sources(Haptics
    PRIVATE
        ${HAPTICS_INCLUDE_DIR}/HapticDeviceManager.hpp
        ${HAPTICS_SRC_DIR}/HapticDeviceManager.cpp
)