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
class MeshObjectConfig;

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

    /** Creates a MeshGraphicsObject from a supplied MeshObject and adds it to the GraphicsScene
     * @param obj : the MeshObject to add to the GraphicsScene for visualization
     * @param obj_config : the MeshObjectConfig that contains visualization parameters (e.g. coloring, draw points, etc.)
     * @returns the index of the provided object in the _graphics_objects array (can be used to fetch it in the future)
     */
    virtual size_t addMeshObject(std::shared_ptr<MeshObject> obj, MeshObjectConfig* obj_config) = 0;

    /** Sets the camera mode to Orthographic */
    virtual void setCameraOrthographic() = 0;

    /** Sets the camera mode to Perspective */
    virtual void setCameraPerspective() = 0;


    /** Gets the camera view direction. */
    virtual Eigen::Vector3d cameraViewDirection() const = 0;
    /** Sets the camera view direction */
    virtual void setCameraViewDirection(const Eigen::Vector3d& view_dir) = 0;

    /** Gets the camera up direction. */
    virtual Eigen::Vector3d cameraUpDirection() const = 0;
    /** Gets the camera right direction. */
    virtual Eigen::Vector3d cameraRightDirection() const = 0;

    /** Gets the camera position. */
    virtual Eigen::Vector3d cameraPosition() const = 0;
    /** Sets the camera position. */
    virtual void setCameraPosition(const Eigen::Vector3d& position) = 0;


    /** Gets a GraphicsObject according to index. (i.e. the index returned by addMeshObject)
     * @param index : the index of the GraphicsObject to get
     * @returns a ptr to the GraphicsObject at the specified index
     */
    GraphicsObject* getObject(const size_t index);

    /** Gets a GraphicsObject according to name.
     * @param name : the name of the GraphicsObject to get
     * @returns a ptr to the GraphicsObject with the specified name (nullptr if no such object exists)
     */
    GraphicsObject* getObject(const std::string& name);

    protected:
    std::string _name;

    std::vector<std::unique_ptr<GraphicsObject>> _graphics_objects; 

};

} // namespace Graphics

#endif // __GRAPHICS_SCENE