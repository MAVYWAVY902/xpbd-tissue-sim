#ifndef __MESH_GRAPHICS_OBJECT_HPP
#define __MESH_GRAPHICS_OBJECT_HPP

#include "graphics/GraphicsObject.hpp"
#include "geometry/Mesh.hpp"
#include "config/MeshObjectConfig.hpp"

#include "common/types.hpp"

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
    explicit MeshGraphicsObject(const std::string& name, const Geometry::Mesh* mesh);

    /** Creates a MeshGraphicsObject with a given name and for a given MeshObject, and sets additional parameters
     * using a MeshObjectConfig object.
     * @param name : the name of the new MeshGraphicsObject
     * @param mesh_object : the simulation MeshObject to get mesh information from
     * @param mesh_object_config : the MeshObjectConfig file to get additional parameters from
     */
    explicit MeshGraphicsObject(const std::string& name, const Geometry::Mesh* mesh_object, const MeshObjectConfig* mesh_object_config);

    virtual ~MeshGraphicsObject();

    /** Returns the underlying simulation MeshObject
     * @returns the underlying simulation MeshObject
     */
    const Geometry::Mesh* mesh() { return _mesh; }

    protected:
    /** The underlying simulation MeshObject */
    const Geometry::Mesh* _mesh;

    /** Whether or not to draw mesh vertices on screen */
    bool _draw_points;
    /** Whether or not to draw mesh edges on screen */
    bool _draw_edges;
    /** Whether or not to draw mesh faces on screen */
    bool _draw_faces;

    /** For now, mesh has a constant coloring
     * Use a 4-vector, RGBA format
     */
    Vec4r _color;
};

} // namespace Graphics


#endif // __MESH_GRAPHICS_OBJECT_HPP