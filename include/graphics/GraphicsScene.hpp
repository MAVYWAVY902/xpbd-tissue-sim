#ifndef __GRAPHICS_SCENE
#define __GRAPHICS_SCENE

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <cassert>

#include "graphics/Viewer.hpp"
#include "graphics/GraphicsObject.hpp"

#include "common/types.hpp"

namespace Sim
{
    class Object;
}
class ObjectConfig;

namespace Graphics {

/** Abstract base class - responsible for updating/visualizing the graphics for multiple simulation objects.
 * 
 * - Defines many pure virtual functions as a general interface and whose implementation depends on the graphics backend used
 * - Right now, only support visualizing MeshObjects, though in theory can be extended to other types of objects (primitives, rods, etc.)
 */
class GraphicsScene
{
    public:
    /** Creates a new GraphicsScene with a given name
     * @param name : the name of the new GraphicsScene
     */
    explicit GraphicsScene(const std::string& name);

    virtual ~GraphicsScene() {}

    /** Initializes the GraphicsScene
     * Responsible for making any graphics API calls to set up the graphics backend (e.g. easy3d::initialize)
     */
    virtual void init() = 0;

    /** Updates all graphics objects in the GraphicsScene and then redraws */
    virtual void update() = 0;

    /** Creates the window and holds it open */
    virtual int run() = 0;

    /** Returns the name of the GraphicsScene */
    std::string name() const { return _name; }

    Viewer* viewer() { return _viewer.get(); }

    /** Creates a MeshGraphicsObject from a supplied MeshObject and adds it to the GraphicsScene
     * @param obj : the simulation Object to add to the GraphicsScene for visualization
     * @param obj_config : the ObjectConfig that contains any visualization parameters (e.g. coloring, draw points, etc.)
     * @returns the index of the provided object in the _graphics_objects array (can be used to fetch it in the future)
     */
    virtual int addObject(const Sim::Object* obj, const ObjectConfig* obj_config) = 0;

    /** Sets the camera mode to Orthographic */
    virtual void setCameraOrthographic() = 0;

    /** Sets the camera mode to Perspective */
    virtual void setCameraPerspective() = 0;


    /** Gets the camera view direction. */
    virtual Vec3r cameraViewDirection() const = 0;
    /** Sets the camera view direction */
    virtual void setCameraViewDirection(const Vec3r& view_dir) = 0;

    /** Gets the camera up direction. */
    virtual Vec3r cameraUpDirection() const = 0;
    /** Gets the camera right direction. */
    virtual Vec3r cameraRightDirection() const = 0;

    /** Gets the camera position. */
    virtual Vec3r cameraPosition() const = 0;
    /** Sets the camera position. */
    virtual void setCameraPosition(const Vec3r& position) = 0;


    /** Gets a GraphicsObject according to index. (i.e. the index returned by addMeshObject)
     * @param index : the index of the GraphicsObject to get
     * @returns a ptr to the GraphicsObject at the specified index
     */
    GraphicsObject* getObject(const int index);

    /** Gets a GraphicsObject according to name.
     * @param name : the name of the GraphicsObject to get
     * @returns a ptr to the GraphicsObject with the specified name (nullptr if no such object exists)
     */
    GraphicsObject* getObject(const std::string& name);

    protected:
    std::string _name;

    std::vector<std::unique_ptr<GraphicsObject>> _graphics_objects; 

    /** the Viewer which renders graphics in a window */
    std::unique_ptr<Graphics::Viewer> _viewer;

};

} // namespace Graphics

#endif // __GRAPHICS_SCENE