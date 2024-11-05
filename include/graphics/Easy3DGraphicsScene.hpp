#ifndef __EASY3D_GRAPHICS_SCENE_HPP
#define __EASY3D_GRAPHICS_SCENE_HPP

#include "graphics/GraphicsScene.hpp"
#include "graphics/Easy3DTextRenderingViewer.hpp"

namespace Graphics
{

class Easy3DGraphicsScene : public GraphicsScene
{
    public:

    /** Creates a new Easy3DGraphicsScene with a given name
     * @param name : the name of the new GraphicsScene
     */
    explicit Easy3DGraphicsScene(const std::string& name);

    virtual ~Easy3DGraphicsScene();

    /** Initializes the GraphicsScene
     * Responsible for making any graphics API calls to set up the graphics backend (e.g. easy3d::initialize)
     */
    virtual void init() override;

    /** Updates all graphics objects in the GraphicsScene and then redraws */
    virtual void update() override;

    /** Creates the window and holds it open */
    virtual int run() override;

    /** Returns the name of the GraphicsScene */
    std::string name() const { return _name; }

    /** Creates a MeshGraphicsObject from a supplied MeshObject and adds it to the GraphicsScene
     * @param obj : the MeshObject to add to the GraphicsScene for visualization
     * @returns the index of the provided object in the _graphics_objects array (can be used to fetch it in the future)
     */
    virtual size_t addMeshObject(std::shared_ptr<MeshObject> obj) override;

    /** Sets the camera mode to Orthographic */
    virtual void setCameraOrthographic() override;

    /** Sets the camera mode to Perspective */
    virtual void setCameraPerspective() override;


    /** Gets the camera view direction. */
    virtual Eigen::Vector3d cameraViewDirection() const override;
    /** Sets the camera view direction */
    virtual void setCameraViewDirection(const Eigen::Vector3d& view_dir) override;

    /** Gets the camera up direction. */
    virtual Eigen::Vector3d cameraUpDirection() const override;
    /** Gets the camera right direction. */
    virtual Eigen::Vector3d cameraRightDirection() const override;

    /** Gets the camera position. */
    virtual Eigen::Vector3d cameraPosition() const override;
    /** Sets the camera position. */
    virtual void setCameraPosition(const Eigen::Vector3d& position) override;


    protected:
    /** the easy3d::Viewer which renders graphics
     * unique_ptr used here for lazy initialization, since easy3d::Viewer must be instantiated 
     * AFTER the easy3d::initialize() call.
     */
    std::unique_ptr<Easy3DTextRenderingViewer> _viewer;


};

} // namespace Graphics

#endif // __EASY3D_GRAPHICS_SCENE_HPP