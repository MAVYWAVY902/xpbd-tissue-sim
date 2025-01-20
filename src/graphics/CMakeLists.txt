cmake_minimum_required(VERSION 3.13)
project(MyProject)

set(GRAPHICS_INCLUDE_DIR "../../include/graphics")
set(GRAPHICS_SRC_DIR "./")

target_sources(Graphics
    PRIVATE
        ${GRAPHICS_INCLUDE_DIR}/Viewer.hpp
        ${GRAPHICS_INCLUDE_DIR}/GraphicsScene.hpp
        ${GRAPHICS_INCLUDE_DIR}/GraphicsObject.hpp
        ${GRAPHICS_INCLUDE_DIR}/MeshGraphicsObject.hpp
        ${GRAPHICS_INCLUDE_DIR}/Easy3DGraphicsScene.hpp
        ${GRAPHICS_INCLUDE_DIR}/Easy3DSphereGraphicsObject.hpp
        ${GRAPHICS_INCLUDE_DIR}/Easy3DBoxGraphicsObject.hpp
        ${GRAPHICS_INCLUDE_DIR}/Easy3DCylinderGraphicsObject.hpp
        ${GRAPHICS_INCLUDE_DIR}/Easy3DMeshGraphicsObject.hpp
        ${GRAPHICS_INCLUDE_DIR}/Easy3DVirtuosoArmGraphicsObject.hpp
        ${GRAPHICS_INCLUDE_DIR}/Easy3DTextRenderingViewer.hpp
        ${GRAPHICS_SRC_DIR}/Viewer.cpp
        ${GRAPHICS_SRC_DIR}/GraphicsScene.cpp
        ${GRAPHICS_SRC_DIR}/MeshGraphicsObject.cpp
        ${GRAPHICS_SRC_DIR}/Easy3DGraphicsScene.cpp
        ${GRAPHICS_SRC_DIR}/Easy3DSphereGraphicsObject.cpp
        ${GRAPHICS_SRC_DIR}/Easy3DBoxGraphicsObject.cpp
        ${GRAPHICS_SRC_DIR}/Easy3DCylinderGraphicsObject.cpp
        ${GRAPHICS_SRC_DIR}/Easy3DMeshGraphicsObject.cpp
        ${GRAPHICS_SRC_DIR}/Easy3DVirtuosoArmGraphicsObject.cpp
        ${GRAPHICS_SRC_DIR}/Easy3DTextRenderingViewer.cpp
)