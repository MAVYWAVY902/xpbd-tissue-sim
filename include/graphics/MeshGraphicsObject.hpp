#ifndef __MESH_GRAPHICS_OBJECT_HPP
#define __MESH_GRAPHICS_OBJECT_HPP

#include "graphics/GraphicsObject.hpp"
#include "simobject/MeshObject.hpp"

namespace Graphics
{

/** Handles visualization for mesh-based simulation objects.
 * 
 * - Because the visualization only deals with vertices/edges/faces, this class does not care if the mesh
 *   is deformable or not
 * - Does not implement the update() method - this requires knowing the specific graphics engine used (i.e. Easy3D, OptiX, etc.)
 * - Stores all visual information associated with a mesh (i.e. texture coordinates, vertex coloring)
 * 
 */
class MeshGraphicsObject : public GraphicsObject
{
    public:
    /** Creates a MeshGraphicsObject with a given name and for a given MeshObject
     * @param name : the name of the new MeshGraphicsObject
     * @param mesh_object : the simulation MeshObject to get mesh information from
     */
    explicit MeshGraphicsObject(const std::string& name, std::shared_ptr<MeshObject> mesh_object);

    virtual ~MeshGraphicsObject();

    /** Returns the underlying simulation MeshObject
     * @returns the underlying simulation MeshObject
     */
    MeshObject* meshObject() { return _mesh_object.get(); }

    protected:
    /** The underlying simulation MeshObject */
    std::shared_ptr<MeshObject> _mesh_object;

    /** Whether or not to draw mesh vertices on screen */
    bool _draw_points;
    /** Whether or not to draw mesh edges on screen */
    bool _draw_edges;
    /** Whether or not to draw mesh faces on screen */
    bool _draw_faces;
};

} // namespace Graphics


#endif // __MESH_GRAPHICS_OBJECT_HPP