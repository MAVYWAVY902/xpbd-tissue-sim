#ifndef __GRAPHICS_SCENE
#define __GRAPHICS_SCENE

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <cassert>

#include <graphics/GraphicsObject.hpp>

#include <Eigen/Dense>

class MeshObject;

namespace Graphics {

class GraphicsScene
{
    public:
    explicit GraphicsScene(const std::string& name);

    virtual ~GraphicsScene() {}

    virtual void init() = 0;
    virtual void update() = 0;
    virtual int run() = 0;

    std::string name() const { return _name; }

    virtual size_t addMeshObject(std::shared_ptr<MeshObject> obj) = 0;

    virtual void setCameraOrthographic() = 0;
    virtual void setCameraPerspective() = 0;

    virtual Eigen::Vector3d cameraViewDirection() const = 0;
    virtual void setCameraViewDirection(const Eigen::Vector3d& view_dir) = 0;
    virtual Eigen::Vector3d cameraUpDirection() const = 0;
    virtual Eigen::Vector3d cameraRightDirection() const = 0;
    virtual Eigen::Vector3d cameraPosition() const = 0;
    virtual void setCameraPosition(const Eigen::Vector3d& position) = 0;

    GraphicsObject* getObject(const size_t index);
    GraphicsObject* getObject(const std::string& name);

    protected:
    std::string _name;

    std::vector<std::unique_ptr<GraphicsObject>> _graphics_objects; 

};

} // namespace Graphics

#endif // __GRAPHICS_SCENE