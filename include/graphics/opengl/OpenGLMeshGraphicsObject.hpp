#ifndef __OPENGL_MESH_GRAPHICS_OBJECT_HPP
#define __OPENGL_MESH_GRAPHICS_OBJECT_HPP

#include "graphics/MeshGraphicsObject.hpp"
#include "config/render/ObjectRenderConfig.hpp"
#include "common/types.hpp"

#include <vector>
#include <array>
#include <mutex>

namespace Sim {
    class MeshObject;
}

namespace Graphics
{

/**
 * Handles visualization for mesh-based simulation objects using raw OpenGL.
 * Manages its own VAO/VBO/EBO for faces, edges, and points rendering.
 */
class OpenGLMeshGraphicsObject : public MeshGraphicsObject
{
    public:
    explicit OpenGLMeshGraphicsObject(const std::string& name,
                                      const Geometry::Mesh* mesh,
                                      const Config::ObjectRenderConfig& render_config,
                                      const Sim::MeshObject* sim_object = nullptr);
    virtual ~OpenGLMeshGraphicsObject();

    /** Updates vertex positions from the simulation mesh and re-uploads to GPU. */
    virtual void update() override;

    /** Draws this mesh using the given shader program (MVP uniforms must already be set). */
    void draw(unsigned int shader_program) const;

    /** Returns whether this object should draw faces/edges/points. */
    bool drawFaces() const { return _do_draw_faces; }
    bool drawEdges() const { return _do_draw_edges; }
    bool drawPoints() const { return _do_draw_points; }

    /** Returns the uniform color for this object. */
    const std::array<float, 4>& color() const { return _color; }

    /** Set texture for this mesh. */
    void setTexture(const std::string& texture_path);

    private:
    void _initGLBuffers();
    void _ensureGLInitialized();
    void _loadTexture();
    void _updateVertexData();
    void _updateFaceIndices();
    void _updateEdgeIndices();

    std::vector<unsigned int> _facesAsFlatList() const;
    std::vector<unsigned int> _edgesAsFlatList() const;

    const Sim::MeshObject* _sim_object = nullptr;
    bool _gl_initialized = false;
    bool _dirty = true;  // Set by update(), consumed by draw()

    // Rendering flags
    bool _do_draw_faces = true;
    bool _do_draw_edges = false;
    bool _do_draw_points = false;

    // Uniform color (RGBA)
    std::array<float, 4> _color = {0.8f, 0.8f, 0.8f, 1.0f};

    // OpenGL handles - faces
    unsigned int _faces_vao = 0;
    unsigned int _faces_vbo = 0;      // vertex positions
    unsigned int _faces_nbo = 0;      // vertex normals
    unsigned int _faces_ebo = 0;      // element buffer
    int _num_face_indices = 0;

    // OpenGL handles - edges
    unsigned int _edges_vao = 0;
    unsigned int _edges_vbo = 0;
    unsigned int _edges_ebo = 0;
    int _num_edge_indices = 0;

    // OpenGL handles - points
    unsigned int _points_vao = 0;
    unsigned int _points_vbo = 0;
    int _num_points = 0;

    // Texture
    unsigned int _texture_id = 0;
    unsigned int _texcoord_vbo = 0;
    bool _has_texture = false;

    // Deferred texture path (loaded when GL context is ready)
    std::string _pending_texture_path;

    // Thread-safe vertex snapshot: update() copies mesh data here,
    // draw() reads from here — avoids racing with the simulation thread.
    mutable std::mutex _snapshot_mutex;
    std::vector<float> _snapshot_positions;
    std::vector<float> _snapshot_normals;
    bool _snapshot_ready = false;
};

} // namespace Graphics

#endif // __OPENGL_MESH_GRAPHICS_OBJECT_HPP
