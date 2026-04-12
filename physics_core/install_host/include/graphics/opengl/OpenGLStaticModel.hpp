#ifndef __OPENGL_STATIC_MODEL_HPP
#define __OPENGL_STATIC_MODEL_HPP

#include <string>
#include <vector>
#include <Eigen/Dense>

namespace Graphics
{

/**
 * A static 3D model loaded from file (GLB/FBX/OBJ) via Assimp.
 * Purely decorative — not connected to the simulation.
 * Uses lazy GL initialization (no GL calls until first draw).
 */
class OpenGLStaticModel
{
public:
    OpenGLStaticModel(const std::string& filepath, const Eigen::Matrix4f& model_transform);
    ~OpenGLStaticModel();

    /** Draw all sub-meshes using the given shader program. */
    void draw(unsigned int shader_program) const;

private:
    struct SubMesh {
        // CPU-side data (populated on load, before GL context)
        std::vector<float> positions;   // 3 floats per vertex
        std::vector<float> normals;     // 3 floats per vertex
        std::vector<float> tangents;    // 3 floats per vertex (for normal mapping)
        std::vector<float> texcoords;   // 2 floats per vertex
        std::vector<unsigned int> indices;

        // Diffuse texture
        std::vector<unsigned char> texture_pixels;
        int tex_width = 0, tex_height = 0, tex_channels = 0;
        bool has_texture = false;

        // Normal map texture
        std::vector<unsigned char> normalmap_pixels;
        int nmap_width = 0, nmap_height = 0, nmap_channels = 0;
        bool has_normalmap = false;

        // Material color (used when no texture)
        float color_r = 0.7f, color_g = 0.7f, color_b = 0.7f, color_a = 1.0f;
        float roughness = 0.5f;

        // GL handles (created lazily)
        unsigned int vao = 0, vbo = 0, nbo = 0, tbo = 0, tanbo = 0, ebo = 0;
        unsigned int texture_id = 0;
        unsigned int normalmap_id = 0;
        int num_indices = 0;
    };

    void _loadFromFile(const std::string& filepath);
    void _ensureGLInitialized() const;

    mutable std::vector<SubMesh> _submeshes;
    Eigen::Matrix4f _model_transform;
    mutable bool _gl_initialized = false;
};

} // namespace Graphics

#endif // __OPENGL_STATIC_MODEL_HPP
