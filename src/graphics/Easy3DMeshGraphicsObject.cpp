#include "graphics/Easy3DMeshGraphicsObject.hpp"

#include <easy3d/renderer/drawable_lines.h>
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/renderer/drawable_triangles.h>
#include <easy3d/renderer/renderer.h>

namespace Graphics {

Easy3DMeshGraphicsObject::Easy3DMeshGraphicsObject(const std::string& name, const Geometry::Mesh* mesh)
    : MeshGraphicsObject(name, mesh)
{
    _init();
}

Easy3DMeshGraphicsObject::Easy3DMeshGraphicsObject(const std::string& name, const Geometry::Mesh* mesh, const MeshObjectConfig* mesh_object_config)
    : MeshGraphicsObject(name, mesh, mesh_object_config)
{
    _init();
}

void Easy3DMeshGraphicsObject::_init()
{
    // first ensure that the vertex cache has enough space for each vertex
    _vertex_cache.resize(_mesh->numVertices());
    // then update the vertex cache to populate it initially
    _updateVertexCache();

    // create a new Renderer for this Model so that the Drawables (below) get updated
    set_renderer(std::make_shared<easy3d::Renderer>(this, true));

    if (_draw_faces)
    {
        // create a TrianglesDrawable for the faces of the tetrahedral mesh
        easy3d::TrianglesDrawable* tri_drawable = renderer()->add_triangles_drawable("faces");
        // specify the update function for the faces
        tri_drawable->set_update_func([](easy3d::Model* m, easy3d::Drawable* d) {
            // downcast to MeshObject for access to facesAsFlatList
            Easy3DMeshGraphicsObject* mo = dynamic_cast<Easy3DMeshGraphicsObject*>(m);
            if (mo)
            {
                // update the vertex buffer and element buffer
                d->update_vertex_buffer(mo->points(), true);
                d->update_element_buffer(mo->facesAsFlatList());
            }
            
        });
        // set a uniform color for the mesh
        easy3d::vec4 color(_color(0), _color(1), _color(2), _color(3));
        tri_drawable->set_uniform_coloring(color);
    }

    if (_draw_points)
    {
        // create a PointsDrawable for the points of the tetrahedral mesh
        easy3d::PointsDrawable* points_drawable = renderer()->add_points_drawable("vertices");
        // specify the update function for the points
        points_drawable->set_update_func([](easy3d::Model* m, easy3d::Drawable* d) {
            // update the vertex buffer with the vertices of the mesh
            d->update_vertex_buffer(m->points(), true);
        });
    }

    if (_draw_edges)
    {
        easy3d::LinesDrawable* lines_drawable = renderer()->add_lines_drawable("lines");
        lines_drawable->set_update_func([](easy3d::Model* m, easy3d::Drawable* d) {
            // downcast to MeshObject for access to facesAsFlatList
            Easy3DMeshGraphicsObject* mo = dynamic_cast<Easy3DMeshGraphicsObject*>(m);
            if (mo)
            {
                // update the vertex buffer and element buffer
                d->update_vertex_buffer(mo->points(), true);
                d->update_element_buffer(mo->edgesAsFlatList());
            }
        });
    }
}

Easy3DMeshGraphicsObject::~Easy3DMeshGraphicsObject()
{

}


void Easy3DMeshGraphicsObject::update()
{
    // update the vertex cache, which is what the renderer uses to update the vertex positions
    _updateVertexCache();
    // then call update on the renderer, which will invoke the Drawable update functions
    renderer()->update();
}


std::vector<unsigned int> Easy3DMeshGraphicsObject::facesAsFlatList() const
{
    // each face (triangle) has 3 vertices
    const Geometry::Mesh::FacesMat& faces = _mesh->faces();
    std::vector<unsigned int> faces_flat_list(_mesh->numFaces()*3);

    // iterate through faces and add them to 1D list
    for (const auto& face : faces.colwise())
    {
        faces_flat_list.insert(faces_flat_list.end(), {static_cast<unsigned>(face(0)), static_cast<unsigned>(face(1)), static_cast<unsigned>(face(2))});
    }

    return faces_flat_list;
}


std::vector<unsigned int> Easy3DMeshGraphicsObject::edgesAsFlatList() const
{
    // TODO: filter duplicate edges
    const Geometry::Mesh::FacesMat& faces = _mesh->faces();
    std::vector<unsigned int> edges_flat_list(_mesh->numFaces()*6);

    // iterate through faces and add each edge to 1D list
    for (const auto& face : faces.colwise())
    {
        edges_flat_list.insert(edges_flat_list.end(), {static_cast<unsigned>(face(0)), static_cast<unsigned>(face(1)),
            static_cast<unsigned>(face(1)), static_cast<unsigned>(face(2)),
            static_cast<unsigned>(face(0)), static_cast<unsigned>(face(2))});
    }

    return edges_flat_list;
}


void Easy3DMeshGraphicsObject::_updateVertexCache()
{
    if (!_mesh)
        return;

    // make sure the vertex cache is big enough for all the vertices
    if(_vertex_cache.size() != static_cast<unsigned>(_mesh->numVertices()))
    {
        _vertex_cache.resize(_mesh->numVertices());
    }

    // get vertices from MeshObject
    const Geometry::Mesh::VerticesMat& vertices = _mesh->vertices();

    // loop through and update each vertex in the cache
    for (int i = 0; i < _mesh->numVertices(); i++)
    {
        _vertex_cache.at(i) = (easy3d::vec3(vertices(0,i), vertices(1,i), vertices(2,i)));
    }
}


} // namespace Graphics