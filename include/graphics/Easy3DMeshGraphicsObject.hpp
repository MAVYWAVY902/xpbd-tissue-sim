#ifndef __EASY3D_MESH_GRAPHICS_OBJECT_HPP
#define __EASY3D_MESH_GRAPHICS_OBJECT_HPP

#include "graphics/MeshGraphicsObject.hpp"

#include <easy3d/core/types.h>
#include <easy3d/core/model.h>

namespace Graphics
{

class Easy3DMeshGraphicsObject : public MeshGraphicsObject, public easy3d::Model
{
    public:
    explicit Easy3DMeshGraphicsObject(const std::string& name, std::shared_ptr<MeshObject> mesh_object);

    virtual ~Easy3DMeshGraphicsObject();

    virtual void update() override;

    /** Returns the vec3 vertex cache.
     * Does NOT check if vertices are stale.
     * 
     * A required override for the easy3d::Model class.
     */
    std::vector<easy3d::vec3>& points() override { return _vertex_cache; };

    /** Returns the vec3 vertex cache.
     * Does NOT check if vertices are stale.
     * 
     * A required override for the easy3d::Model class.
     */
    const std::vector<easy3d::vec3>& points() const override { return _vertex_cache; };

    /** Returns the faces of the mesh as a flat vector of vertex indices. Used for moving face information to GPU via Easy3d.
     * As per specified by TrianglesDrawable docs, each 3 consecutive vertices represents a face.
     * @returns a 1d vecor of vertex indices - 3 consecutive entries corresponds to a face to be rendered.
     */
    std::vector<unsigned int> facesAsFlatList() const;

    /** Returns the surface edges of the mesh as a flat vector of vertex indices. Used for moving edge information to GPU via Easy3d.
     * As per specified by LinesDrawable docs, each 2 consecutive vertices represents an edge.
     * @returns a 1d vector of vertex indicies - 2 consecutive entries corresponds to a edge to be rendered.
     */
    std::vector<unsigned int> edgesAsFlatList() const;

    protected:
    /** Updates the vertex cache. Should be called when the _vertices matrix has been changed.
     * Does NOT reallocate storage for the vertices, it has a fixed amount of space.
     * Each vertex is written over the top of the previous version of itself.
     */
    void _updateVertexCache();

    void _init();

    protected:
    /** Vector of vec3 vertices that is continually updated as _vertices changes.
     * Used by easy3d to update the vertex buffers (i.e. the positions) of the geometry on the graphics side.
     */
    std::vector<easy3d::vec3> _vertex_cache;

};

}

#endif // __EASY3D_MESH_GRAPHICS_OBJECT_HPP