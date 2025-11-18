#include "graphics/easy3d/Easy3DMeshGraphicsObject.hpp"

#include <easy3d/renderer/drawable_lines.h>
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/renderer/drawable_triangles.h>
#include <easy3d/renderer/renderer.h>

namespace Graphics {

Easy3DMeshGraphicsObject::Easy3DMeshGraphicsObject(const std::string& name, const Geometry::Mesh* mesh, const Config::ObjectRenderConfig& render_config)
    : MeshGraphicsObject(name, mesh)
{
    std::cout << "[Easy3D] DEBUG: Creating MeshGraphicsObject '" << name << "' with " << mesh->numVertices() << " vertices, " 
              << mesh->numFaces() << " faces\n";
    std::cout << "[Easy3D] DEBUG: Render config - drawFaces=" << render_config.drawFaces() 
              << " drawEdges=" << render_config.drawEdges() << " drawPoints=" << render_config.drawPoints() << "\n";
    
    // TEMPORARY FIX: Force edge rendering for 1D meshes
    bool force_draw_edges = (mesh->numFaces() == 0 && mesh->numVertices() > 1);
    if (force_draw_edges) {
        std::cout << "[Easy3D] DEBUG: 1D mesh detected (0 faces), FORCING drawEdges=true\n";
    }
    
    _init(render_config, force_draw_edges);
}

void Easy3DMeshGraphicsObject::_init(const Config::ObjectRenderConfig& config, bool force_draw_edges)
{
    // first ensure that the vertex cache has enough space for each vertex
    _vertex_cache.resize(_mesh->numVertices());
    // then update the vertex cache to populate it initially
    _updateVertexCache();

    // create a new Renderer for this Model so that the Drawables (below) get updated
    set_renderer(std::make_shared<easy3d::Renderer>(this, true));

    if (config.drawFaces())
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
        if (config.color().has_value())
        {
            easy3d::vec4 color(config.color().value()[0], config.color().value()[1], config.color().value()[2], config.opacity());
            tri_drawable->set_uniform_coloring(color);
        }
    }

    if (config.drawPoints())
    {
        // create a PointsDrawable for the points of the tetrahedral mesh
        easy3d::PointsDrawable* points_drawable = renderer()->add_points_drawable("vertices");
        // specify the update function for the points
        points_drawable->set_update_func([](easy3d::Model* m, easy3d::Drawable* d) {
            Easy3DMeshGraphicsObject* mo = dynamic_cast<Easy3DMeshGraphicsObject*>(m);
            if (mo) {
                std::cout << "[viz] Graphics update called for object with " << mo->_mesh->numVertices() << " vertices\n";
                // update the vertex buffer with the vertices of the mesh
                d->update_vertex_buffer(m->points(), true);
                
                // Check if we have adhesion constraint markers and apply per-vertex coloring
                std::cout << "[viz] Graphics update - checking mesh " << mo->_mesh << " for adhesion property\n";
                if (mo->_mesh->template hasVertexProperty<bool>("has_adhesion_constraint")) {
                    const auto& adhesion_prop = mo->_mesh->template getVertexProperty<bool>("has_adhesion_constraint");
                    std::vector<easy3d::vec3> colors;
                    colors.reserve(mo->_mesh->numVertices());
                    
                    int adhesion_count = 0;
                    for (int i = 0; i < mo->_mesh->numVertices(); ++i) {
                        if (adhesion_prop.get(i)) {
                            // Bright cyan color for vertices with adhesion constraints  
                            colors.emplace_back(0.0f, 1.0f, 1.0f); // Bright cyan
                            adhesion_count++;
                        } else {
                            // Darker color for contrast
                            colors.emplace_back(0.2f, 0.2f, 0.2f); // Dark gray
                        }
                    }
                    std::cout << "[viz] Applied per-vertex coloring: " << adhesion_count << "/" << mo->_mesh->numVertices() << " vertices have adhesion constraints (blue)\n";
                    d->update_color_buffer(colors);
                } else {
                    std::cout << "[viz] No adhesion constraint property found, using default coloring\n";
                }
            }
        });
        
        // Set a much larger point size to make adhesion markers clearly visible
        points_drawable->set_point_size(15.0f);
    }

    if (config.drawEdges() || force_draw_edges)
    {
        std::cout << "[Easy3D] DEBUG: drawEdges=" << (config.drawEdges() ? "true" : "false") 
                  << " force_draw_edges=" << (force_draw_edges ? "true" : "false") << ", creating LinesDrawable\n";
        easy3d::LinesDrawable* lines_drawable = renderer()->add_lines_drawable("lines");
        lines_drawable->set_update_func([](easy3d::Model* m, easy3d::Drawable* d) {
            // downcast to MeshObject for access to facesAsFlatList
            Easy3DMeshGraphicsObject* mo = dynamic_cast<Easy3DMeshGraphicsObject*>(m);
            if (mo)
            {
                // update the vertex buffer and element buffer
                d->update_vertex_buffer(mo->points(), true);
                auto edges = mo->edgesAsFlatList();
                d->update_element_buffer(edges);
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
    std::vector<unsigned int> faces_flat_list;
    faces_flat_list.reserve(_mesh->numFaces()*3);

    bool has_draw_property = _mesh->hasFaceProperty<bool>("draw");

    if (has_draw_property)
    {
        const std::vector<bool>& draw_face = _mesh->getFaceProperty<bool>("draw").properties();
        // iterate through faces and add them to 1D list
        for (int i = 0; i < _mesh->numFaces(); i++)
        {
            if (!draw_face[i])
                continue;
            
            const Vec3i& face = _mesh->face(i); 
            faces_flat_list.insert(faces_flat_list.end(), {static_cast<unsigned>(face(0)), static_cast<unsigned>(face(1)), static_cast<unsigned>(face(2))});
        }
    }
    else
    {
        // iterate through faces and add them to 1D list
        for (const auto& face : faces.colwise())
        {
            faces_flat_list.insert(faces_flat_list.end(), {static_cast<unsigned>(face(0)), static_cast<unsigned>(face(1)), static_cast<unsigned>(face(2))});
        }
    }
    

    return faces_flat_list;
}


std::vector<unsigned int> Easy3DMeshGraphicsObject::edgesAsFlatList() const
{
    // NEW: Check for stored line segments marker first (for 1D meshes)
    if (_mesh->hasVertexProperty<int>("has_line_segments")) {
        int num_segments = _mesh->getVertexProperty<int>("has_line_segments").get(0);
        
        // Only print this message once per object to avoid spam
        static bool printed_1d_mesh_info = false;
        if (!printed_1d_mesh_info) {
            std::cerr << "[viz] Found 1D mesh with " << num_segments << " line segments - using sequential edges\n";
            printed_1d_mesh_info = true;
        }
        
        // For 1D meshes, create sequential line segments connecting consecutive vertices
        std::vector<unsigned int> edges_flat_list;
        edges_flat_list.reserve(num_segments * 2);
        for (int i = 0; i < _mesh->numVertices() - 1; ++i) {
            edges_flat_list.push_back(static_cast<unsigned int>(i));
            edges_flat_list.push_back(static_cast<unsigned int>(i + 1));
        }
        
        return edges_flat_list;
    }
    
    // FALLBACK: Extract edges from faces (for traditional 3D meshes)
    // TODO: filter duplicate edges
    const Geometry::Mesh::FacesMat& faces = _mesh->faces();
    std::vector<unsigned int> edges_flat_list;
    edges_flat_list.reserve(_mesh->numFaces()*6);

    bool has_draw_property = _mesh->hasFaceProperty<bool>("draw");

    if (has_draw_property)
    {
        const std::vector<bool>& draw_face = _mesh->getFaceProperty<bool>("draw").properties();
        // iterate through faces and add them to 1D list
        for (int i = 0; i < _mesh->numFaces(); i++)
        {
            if (!draw_face[i])
                continue;
            
            const Vec3i& face = _mesh->face(i); 
            edges_flat_list.insert(edges_flat_list.end(), {static_cast<unsigned>(face(0)), static_cast<unsigned>(face(1)),
                static_cast<unsigned>(face(1)), static_cast<unsigned>(face(2)),
                static_cast<unsigned>(face(0)), static_cast<unsigned>(face(2))});
        }
    }
    else
    {
        // iterate through faces and add each edge to 1D list
        for (const auto& face : faces.colwise())
        {
            edges_flat_list.insert(edges_flat_list.end(), {static_cast<unsigned>(face(0)), static_cast<unsigned>(face(1)),
                static_cast<unsigned>(face(1)), static_cast<unsigned>(face(2)),
                static_cast<unsigned>(face(0)), static_cast<unsigned>(face(2))});
        }
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