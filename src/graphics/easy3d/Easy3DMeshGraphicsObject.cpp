#include "graphics/easy3d/Easy3DMeshGraphicsObject.hpp"

#include <easy3d/renderer/drawable_lines.h>
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/renderer/drawable_triangles.h>
#include <easy3d/renderer/renderer.h>
#include <easy3d/renderer/texture.h>
#include <easy3d/util/resource.h>

#include "simobject/MeshObject.hpp"
#include "simobject/XPBDMeshObjectBase.hpp"

#include <iostream>

namespace Graphics {

Easy3DMeshGraphicsObject::Easy3DMeshGraphicsObject(const std::string& name, const Geometry::Mesh* mesh, const Config::ObjectRenderConfig& render_config, const Sim::MeshObject* sim_object)
    : MeshGraphicsObject(name, mesh), _sim_object(sim_object)
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
        
        // Set uniform color for the mesh
        // Priority: 1) MTL diffuse color, 2) config color, 3) config colors[0]
        Vec3r final_color(0.8, 0.8, 0.8); // default gray
        bool has_color = false;
        
        std::cout << "[Easy3D] DEBUG: Checking colors for '" << GraphicsObject::name() << "':\n";
        std::cout << "  - diffuseColor has_value: " << config.diffuseColor().has_value() << "\n";
        std::cout << "  - color has_value: " << config.color().has_value() << "\n";
        std::cout << "  - colors has_value: " << config.colors().has_value() << "\n";
        
        // Check for MTL diffuse color (highest priority)
        if (config.diffuseColor().has_value())
        {
            final_color = config.diffuseColor().value();
            has_color = true;
            std::cout << "  -> Using diffuseColor: [" << final_color[0] << " " << final_color[1] << " " << final_color[2] << "]\n";
        }
        // Fall back to config color
        else if (config.color().has_value())
        {
            final_color = config.color().value();
            has_color = true;
            std::cout << "  -> Using color: [" << final_color[0] << " " << final_color[1] << " " << final_color[2] << "]\n";
        }
        // Fall back to first color in colors array
        else if (config.colors().has_value() && config.colors().value().size() > 0)
        {
            final_color = config.colors().value()[0];
            has_color = true;
            std::cout << "  -> Using colors[0]: [" << final_color[0] << " " << final_color[1] << " " << final_color[2] << "]\n";
        }
        else
        {
            std::cout << "  -> Using default gray: [" << final_color[0] << " " << final_color[1] << " " << final_color[2] << "]\n";
        }
        
        if (has_color)
        {
            easy3d::vec4 color(final_color[0], final_color[1], final_color[2], config.opacity());
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
                // std::cout << "[viz] Graphics update called for object with " << mo->_mesh->numVertices() << " vertices\n";
                // update the vertex buffer with the vertices of the mesh
                d->update_vertex_buffer(m->points(), true);
                
                // Check if we have RIGID-DEFORM adhesion markers and apply per-vertex coloring
                
                std::vector<easy3d::vec3> colors;
                colors.reserve(mo->_mesh->numVertices());
                
                if (mo->_mesh->template hasVertexProperty<bool>("has_rigid_adhesion")) {
                    const auto& adhesion_prop = mo->_mesh->template getVertexProperty<bool>("has_rigid_adhesion");
                    
                    int adhesion_count = 0;
                    int first_green_idx = -1;
                    for (int i = 0; i < mo->_mesh->numVertices(); ++i) {
                        if (adhesion_prop.get(i)) {
                            // Bright green color for ACTIVE adhesion vertices  
                            colors.emplace_back(0.0f, 1.0f, 0.0f); // Bright green
                            if (first_green_idx < 0) first_green_idx = i;
                            adhesion_count++;
                        } else {
                            // WHITE for inactive vertices (more visible than black)
                            colors.emplace_back(1.0f, 1.0f, 1.0f); // White
                        }
                    }
                    // Print EVERY frame for debugging
                    std::cout << "[GRAPHICS-VIZ] Active (green): " << adhesion_count << "/" 
                              << mo->_mesh->numVertices() << " | Inactive (white): " << (mo->_mesh->numVertices() - adhesion_count)
                              << " | First green at idx:" << first_green_idx << std::endl;
                } else {
                    std::cout << "[GRAPHICS-VIZ] Object does NOT have has_rigid_adhesion property!" << std::endl;
                    // No adhesion property - use default gray for all vertices
                    for (int i = 0; i < mo->_mesh->numVertices(); ++i) {
                        colors.emplace_back(0.5f, 0.5f, 0.5f); // Gray
                    }
                }
                
                // ALWAYS update color buffer (even with default colors)
                d->update_color_buffer(colors);
                
                // CRITICAL: Disable uniform coloring AFTER updating color buffer
                // to ensure per-vertex colors take effect
                easy3d::PointsDrawable* pts = dynamic_cast<easy3d::PointsDrawable*>(d);
                if (pts) {
                    pts->set_uniform_coloring(easy3d::vec4(-1, -1, -1, -1)); // Disable uniform, use per-vertex
                }
            }
        });
        
        // Configure point rendering for maximum visibility
        points_drawable->set_point_size(10.0f);
        points_drawable->set_impostor_type(easy3d::PointsDrawable::SPHERE);
        
        // DON'T set uniform coloring here - let the update_func handle per-vertex colors
        // points_drawable->set_uniform_coloring(...) is intentionally omitted
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

    // TODO: TEMPORARILY DISABLED adhesion visualization due to Easy3D PropertyArray issues
    // The physics simulation works correctly, but visualization causes crashes
    // Will be re-enabled after investigating Easy3D's internal state management
    
    /*
    // ✅ CRITICAL: Only create adhesion visualization for XPBD mesh objects (deformable)
    if (_sim_object) {
        const bool is_xpbd_mesh = (dynamic_cast<const Sim::XPBDMeshObject_Base*>(_sim_object) != nullptr) ||
                                  (dynamic_cast<const Sim::FirstOrderXPBDMeshObject_Base*>(_sim_object) != nullptr);
        
        if (is_xpbd_mesh) {
            // Adhesion visualization code here...
        }
    }
    */

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

void Easy3DMeshGraphicsObject::setTexture(const std::string& texture_path)
{
    std::cout << "[Easy3D] Loading texture: " << texture_path << std::endl;
    
    // Get the triangles drawable (faces) using renderer
    easy3d::TrianglesDrawable* tri_drawable = renderer()->get_triangles_drawable("faces");
    if (!tri_drawable)
    {
        std::cerr << "[Easy3D] ERROR: No triangles drawable found for texture!" << std::endl;
        return;
    }
    
    // Check if mesh has UV coordinates
    if (!_mesh->hasUVCoords())
    {
        std::cerr << "[Easy3D] ERROR: Mesh has no UV coordinates for texture mapping!" << std::endl;
        return;
    }
    
    // Get UV coordinates from mesh
    const Eigen::Matrix<Real, 2, -1>& uv_coords = _mesh->uvCoords();
    std::cout << "[Easy3D] Found " << uv_coords.cols() << " UV coordinates" << std::endl;
    
    // Convert UV coordinates to Easy3D format
    std::vector<easy3d::vec2> texcoords;
    texcoords.reserve(uv_coords.cols());
    for (int i = 0; i < uv_coords.cols(); i++)
    {
        texcoords.push_back(easy3d::vec2(uv_coords(0, i), uv_coords(1, i)));
    }
    
    // Set texture coordinates on the drawable
    tri_drawable->update_texcoord_buffer(texcoords);
    
    // Load the texture
    easy3d::Texture* texture = easy3d::Texture::create(texture_path);
    if (!texture)
    {
        std::cerr << "[Easy3D] ERROR: Failed to load texture from: " << texture_path << std::endl;
        return;
    }
    
    std::cout << "[Easy3D] Texture loaded successfully: " << texture->width() << "x" << texture->height() << std::endl;
    
    // Apply texture to the drawable
    tri_drawable->set_texture(texture);
    
    // Disable uniform coloring when using texture
    tri_drawable->set_coloring_method(easy3d::State::TEXTURED);
    
    std::cout << "[Easy3D] Texture rendering mode set to TEXTURED" << std::endl;
    std::cout << "[Easy3D] Current texture: " << (tri_drawable->texture() ? "SET" : "NULL") << std::endl;
    std::cout << "[Easy3D] Texture applied successfully!" << std::endl;
}


} // namespace Graphics