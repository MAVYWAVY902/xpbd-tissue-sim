#ifndef __EASY3D_GRAPHICS_SCENE_HPP
#define __EASY3D_GRAPHICS_SCENE_HPP

#include "graphics/GraphicsScene.hpp"
#include "graphics/Easy3DTextRenderingViewer.hpp"

#include "simobject/Object.hpp"
#include "config/MeshObjectConfig.hpp"
#include "config/ObjectConfig.hpp"

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
     * @param obj_config : the MeshObjectConfig that contains visualization parameters (e.g. coloring, draw points, etc.)
     * @returns the index of the provided object in the _graphics_objects array (can be used to fetch it in the future)
     */
    virtual int addObject(const Sim::Object* obj, const ObjectConfig* obj_config) override;

    /** Sets the camera mode to Orthographic */
    virtual void setCameraOrthographic() override;

    /** Sets the camera mode to Perspective */
    virtual void setCameraPerspective() override;


    /** Gets the camera view direction. */
    virtual Vec3r cameraViewDirection() const override;
    /** Sets the camera view direction */
    virtual void setCameraViewDirection(const Vec3r& view_dir) override;

    /** Gets the camera up direction. */
    virtual Vec3r cameraUpDirection() const override;
    /** Gets the camera right direction. */
    virtual Vec3r cameraRightDirection() const override;

    /** Gets the camera position. */
    virtual Vec3r cameraPosition() const override;
    /** Sets the camera position. */
    virtual void setCameraPosition(const Vec3r& position) override;


    protected:
    /** the easy3d::Viewer which renders graphics, non-owning downcasted pointer for convenience in calling easy3d::Viewer specific methods */
    Easy3DTextRenderingViewer* _easy3d_viewer;


};

} // namespace Graphics

#endif // __EASY3D_GRAPHICS_SCENE_HPP