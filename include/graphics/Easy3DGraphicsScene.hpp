#ifndef __EASY3D_GRAPHICS_SCENE_HPP
#define __EASY3D_GRAPHICS_SCENE_HPP

#include "graphics/GraphicsScene.hpp"
#include "graphics/Easy3DTextRenderingViewer.hpp"

namespace Graphics
{

class Easy3DGraphicsScene : public GraphicsScene
{
    public:

    explicit Easy3DGraphicsScene(const std::string& name);

    virtual ~Easy3DGraphicsScene();

    virtual void init() override;
    virtual void update() override;
    virtual int run() override;

    virtual size_t addMeshObject(std::shared_ptr<MeshObject> obj) override;
    
    virtual void setCameraOrthographic() override;
    virtual void setCameraPerspective() override;

    virtual Eigen::Vector3d cameraViewDirection() const override;
    virtual void setCameraViewDirection(const Eigen::Vector3d& view_dir) override;
    virtual Eigen::Vector3d cameraUpDirection() const override;
    virtual Eigen::Vector3d cameraRightDirection() const override;
    virtual Eigen::Vector3d cameraPosition() const override;
    virtual void setCameraPosition(const Eigen::Vector3d& position) override;

    protected:
    /** the Viewer which renders graphics
     * unique_ptr used here for lazy initialization, since easy3d::Viewer must be instantiated 
     * AFTER the easy3d::initialize() call.
     */
    std::unique_ptr<Easy3DTextRenderingViewer> _viewer;


};

} // namespace Graphics

#endif // __EASY3D_GRAPHICS_SCENE_HPP