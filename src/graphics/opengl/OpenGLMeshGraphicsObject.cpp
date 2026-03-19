#include "graphics/opengl/OpenGLMeshGraphicsObject.hpp"

#include <GL/glew.h>

#include "simobject/MeshObject.hpp"
#include "geometry/Mesh.hpp"
#include "graphics/opengl/stb_image.h"

#include <iostream>

namespace Graphics
{

OpenGLMeshGraphicsObject::OpenGLMeshGraphicsObject(
    const std::string& name,
    const Geometry::Mesh* mesh,
    const Config::ObjectRenderConfig& render_config,
    const Sim::MeshObject* sim_object)
    : MeshGraphicsObject(name, mesh), _sim_object(sim_object)
{
    std::cout << "[OpenGL] Creating MeshGraphicsObject '" << name << "' with "
              << mesh->numVertices() << " vertices, " << mesh->numFaces() << " faces\n";

    _do_draw_faces = render_config.drawFaces();
    _do_draw_edges = render_config.drawEdges() || (mesh->numFaces() == 0 && mesh->numVertices() > 1);
    _do_draw_points = render_config.drawPoints();

    // Determine color (no GL calls here)
    Vec3r final_color(0.8, 0.8, 0.8);
    if (render_config.diffuseColor().has_value()) {
        final_color = render_config.diffuseColor().value();
    } else if (render_config.color().has_value()) {
        final_color = render_config.color().value();
    } else if (render_config.colors().has_value() && render_config.colors().value().size() > 0) {
        final_color = render_config.colors().value()[0];
    }
    _color = {static_cast<float>(final_color[0]),
              static_cast<float>(final_color[1]),
              static_cast<float>(final_color[2]),
              static_cast<float>(render_config.opacity())};

    // GL buffers will be created lazily on first draw()
}

OpenGLMeshGraphicsObject::~OpenGLMeshGraphicsObject()
{
    if (_faces_vao) glDeleteVertexArrays(1, &_faces_vao);
    if (_faces_vbo) glDeleteBuffers(1, &_faces_vbo);
    if (_faces_nbo) glDeleteBuffers(1, &_faces_nbo);
    if (_faces_ebo) glDeleteBuffers(1, &_faces_ebo);

    if (_edges_vao) glDeleteVertexArrays(1, &_edges_vao);
    if (_edges_vbo) glDeleteBuffers(1, &_edges_vbo);
    if (_edges_ebo) glDeleteBuffers(1, &_edges_ebo);

    if (_points_vao) glDeleteVertexArrays(1, &_points_vao);
    if (_points_vbo) glDeleteBuffers(1, &_points_vbo);

    if (_texcoord_vbo) glDeleteBuffers(1, &_texcoord_vbo);
    if (_texture_id) glDeleteTextures(1, &_texture_id);
}

void OpenGLMeshGraphicsObject::_ensureGLInitialized()
{
    if (_gl_initialized) return;
    _gl_initialized = true;
    _initGLBuffers();
    _loadTexture();
}

void OpenGLMeshGraphicsObject::_initGLBuffers()
{
    // Get initial vertex data
    const auto& vertices = _mesh->vertices();
    int nv = _mesh->numVertices();

    // Prepare vertex positions as float array (3 floats per vertex)
    std::vector<float> positions(nv * 3);
    for (int i = 0; i < nv; i++) {
        positions[i*3+0] = static_cast<float>(vertices(0, i));
        positions[i*3+1] = static_cast<float>(vertices(1, i));
        positions[i*3+2] = static_cast<float>(vertices(2, i));
    }

    // Compute per-vertex normals (average of adjacent face normals)
    std::vector<float> normals(nv * 3, 0.0f);
    if (_mesh->numFaces() > 0) {
        const auto& faces = _mesh->faces();
        for (int f = 0; f < _mesh->numFaces(); f++) {
            int i0 = faces(0, f), i1 = faces(1, f), i2 = faces(2, f);
            Eigen::Vector3f v0(positions[i0*3], positions[i0*3+1], positions[i0*3+2]);
            Eigen::Vector3f v1(positions[i1*3], positions[i1*3+1], positions[i1*3+2]);
            Eigen::Vector3f v2(positions[i2*3], positions[i2*3+1], positions[i2*3+2]);
            Eigen::Vector3f n = (v1 - v0).cross(v2 - v0);
            for (int vi : {i0, i1, i2}) {
                normals[vi*3+0] += n.x();
                normals[vi*3+1] += n.y();
                normals[vi*3+2] += n.z();
            }
        }
        // Normalize
        for (int i = 0; i < nv; i++) {
            Eigen::Vector3f n(normals[i*3], normals[i*3+1], normals[i*3+2]);
            float len = n.norm();
            if (len > 1e-8f) {
                normals[i*3+0] /= len;
                normals[i*3+1] /= len;
                normals[i*3+2] /= len;
            }
        }
    }

    // ---- Faces VAO ----
    if (_do_draw_faces && _mesh->numFaces() > 0) {
        auto face_indices = _facesAsFlatList();
        _num_face_indices = static_cast<int>(face_indices.size());

        glGenVertexArrays(1, &_faces_vao);
        glGenBuffers(1, &_faces_vbo);
        glGenBuffers(1, &_faces_nbo);
        glGenBuffers(1, &_faces_ebo);

        glBindVertexArray(_faces_vao);

        // Positions
        glBindBuffer(GL_ARRAY_BUFFER, _faces_vbo);
        glBufferData(GL_ARRAY_BUFFER, positions.size() * sizeof(float), positions.data(), GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

        // Normals
        glBindBuffer(GL_ARRAY_BUFFER, _faces_nbo);
        glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(float), normals.data(), GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

        // Indices
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _faces_ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, face_indices.size() * sizeof(unsigned int), face_indices.data(), GL_DYNAMIC_DRAW);

        glBindVertexArray(0);
    }

    // ---- Edges VAO ----
    if (_do_draw_edges) {
        auto edge_indices = _edgesAsFlatList();
        _num_edge_indices = static_cast<int>(edge_indices.size());

        glGenVertexArrays(1, &_edges_vao);
        glGenBuffers(1, &_edges_vbo);
        glGenBuffers(1, &_edges_ebo);

        glBindVertexArray(_edges_vao);

        glBindBuffer(GL_ARRAY_BUFFER, _edges_vbo);
        glBufferData(GL_ARRAY_BUFFER, positions.size() * sizeof(float), positions.data(), GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _edges_ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, edge_indices.size() * sizeof(unsigned int), edge_indices.data(), GL_DYNAMIC_DRAW);

        glBindVertexArray(0);
    }

    // ---- Points VAO ----
    if (_do_draw_points) {
        _num_points = nv;

        glGenVertexArrays(1, &_points_vao);
        glGenBuffers(1, &_points_vbo);

        glBindVertexArray(_points_vao);

        glBindBuffer(GL_ARRAY_BUFFER, _points_vbo);
        glBufferData(GL_ARRAY_BUFFER, positions.size() * sizeof(float), positions.data(), GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

        glBindVertexArray(0);
    }
}

void OpenGLMeshGraphicsObject::update()
{
    if (!_mesh) return;

    const auto& vertices = _mesh->vertices();
    int nv = _mesh->numVertices();

    // Snapshot positions
    std::vector<float> positions(nv * 3);
    for (int i = 0; i < nv; i++) {
        positions[i*3+0] = static_cast<float>(vertices(0, i));
        positions[i*3+1] = static_cast<float>(vertices(1, i));
        positions[i*3+2] = static_cast<float>(vertices(2, i));
    }

    // Compute normals from the snapshot
    std::vector<float> normals(nv * 3, 0.0f);
    if (_mesh->numFaces() > 0) {
        const auto& faces = _mesh->faces();
        for (int f = 0; f < _mesh->numFaces(); f++) {
            int i0 = faces(0, f), i1 = faces(1, f), i2 = faces(2, f);
            Eigen::Vector3f v0(positions[i0*3], positions[i0*3+1], positions[i0*3+2]);
            Eigen::Vector3f v1(positions[i1*3], positions[i1*3+1], positions[i1*3+2]);
            Eigen::Vector3f v2(positions[i2*3], positions[i2*3+1], positions[i2*3+2]);
            Eigen::Vector3f n = (v1 - v0).cross(v2 - v0);
            for (int vi : {i0, i1, i2}) {
                normals[vi*3+0] += n.x();
                normals[vi*3+1] += n.y();
                normals[vi*3+2] += n.z();
            }
        }
        for (int i = 0; i < nv; i++) {
            Eigen::Vector3f n(normals[i*3], normals[i*3+1], normals[i*3+2]);
            float len = n.norm();
            if (len > 1e-8f) {
                normals[i*3+0] /= len;
                normals[i*3+1] /= len;
                normals[i*3+2] /= len;
            }
        }
    }

    // Store snapshot under lock
    {
        std::lock_guard<std::mutex> lock(_snapshot_mutex);
        _snapshot_positions = std::move(positions);
        _snapshot_normals = std::move(normals);
        _snapshot_ready = true;
    }

    _dirty = true;
}

void OpenGLMeshGraphicsObject::_updateVertexData()
{
    // Read from thread-safe snapshot (populated by update() on sim thread)
    std::vector<float> positions;
    std::vector<float> normals;
    {
        std::lock_guard<std::mutex> lock(_snapshot_mutex);
        if (!_snapshot_ready) return;
        positions = _snapshot_positions;
        normals = _snapshot_normals;
        _snapshot_ready = false;
    }

    _num_points = static_cast<int>(positions.size() / 3);

    // Upload to GPU
    if (_faces_vao) {
        glBindBuffer(GL_ARRAY_BUFFER, _faces_vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, positions.size() * sizeof(float), positions.data());
        glBindBuffer(GL_ARRAY_BUFFER, _faces_nbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, normals.size() * sizeof(float), normals.data());
    }

    if (_edges_vao) {
        glBindBuffer(GL_ARRAY_BUFFER, _edges_vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, positions.size() * sizeof(float), positions.data());
    }

    if (_points_vao) {
        glBindBuffer(GL_ARRAY_BUFFER, _points_vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, positions.size() * sizeof(float), positions.data());
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void OpenGLMeshGraphicsObject::_updateFaceIndices()
{
    if (!_faces_vao || !_mesh) return;

    auto face_indices = _facesAsFlatList();
    _num_face_indices = static_cast<int>(face_indices.size());

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _faces_ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, face_indices.size() * sizeof(unsigned int), face_indices.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void OpenGLMeshGraphicsObject::_updateEdgeIndices()
{
    if (!_edges_vao || !_mesh) return;

    auto edge_indices = _edgesAsFlatList();
    _num_edge_indices = static_cast<int>(edge_indices.size());

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _edges_ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, edge_indices.size() * sizeof(unsigned int), edge_indices.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void OpenGLMeshGraphicsObject::draw(unsigned int shader_program) const
{
    // Lazy-init GL buffers on first draw (GL context is guaranteed to exist here)
    auto* self = const_cast<OpenGLMeshGraphicsObject*>(this);
    self->_ensureGLInitialized();

    // Upload new vertex/index data if simulation thread flagged us dirty
    if (_dirty) {
        self->_dirty = false;
        self->_updateVertexData();
        self->_updateFaceIndices();
        self->_updateEdgeIndices();
    }

    // Draw faces (lit)
    if (_do_draw_faces && _faces_vao && _num_face_indices > 0) {
        glUniform4f(glGetUniformLocation(shader_program, "uColor"),
                    _color[0], _color[1], _color[2], _color[3]);
        glUniform1i(glGetUniformLocation(shader_program, "uUseLighting"), 1);

        if (_has_texture && _texture_id) {
            glUniform1i(glGetUniformLocation(shader_program, "uUseTexture"), 1);
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, _texture_id);
            glUniform1i(glGetUniformLocation(shader_program, "uTexture"), 0);
        } else {
            glUniform1i(glGetUniformLocation(shader_program, "uUseTexture"), 0);
        }

        glBindVertexArray(_faces_vao);
        glDrawElements(GL_TRIANGLES, _num_face_indices, GL_UNSIGNED_INT, nullptr);
        glBindVertexArray(0);
    }

    // Draw edges (unlit)
    if (_do_draw_edges && _edges_vao && _num_edge_indices > 0) {
        glUniform4f(glGetUniformLocation(shader_program, "uColor"), 0.0f, 0.0f, 0.0f, 1.0f);
        glUniform1i(glGetUniformLocation(shader_program, "uUseLighting"), 0);
        glUniform1i(glGetUniformLocation(shader_program, "uUseTexture"), 0);

        glBindVertexArray(_edges_vao);
        glDrawElements(GL_LINES, _num_edge_indices, GL_UNSIGNED_INT, nullptr);
        glBindVertexArray(0);
    }

    // Draw points
    if (_do_draw_points && _points_vao && _num_points > 0) {
        glUniform4f(glGetUniformLocation(shader_program, "uColor"), 1.0f, 0.0f, 0.0f, 1.0f);
        glUniform1i(glGetUniformLocation(shader_program, "uUseLighting"), 0);
        glUniform1i(glGetUniformLocation(shader_program, "uUseTexture"), 0);

        glPointSize(6.0f);
        glBindVertexArray(_points_vao);
        glDrawArrays(GL_POINTS, 0, _num_points);
        glBindVertexArray(0);
    }
}

std::vector<unsigned int> OpenGLMeshGraphicsObject::_facesAsFlatList() const
{
    const auto& faces = _mesh->faces();
    std::vector<unsigned int> flat;
    flat.reserve(_mesh->numFaces() * 3);

    bool has_draw_property = _mesh->hasFaceProperty<bool>("draw");

    if (has_draw_property) {
        const auto& draw_face = _mesh->getFaceProperty<bool>("draw").properties();
        for (int i = 0; i < _mesh->numFaces(); i++) {
            if (!draw_face[i]) continue;
            const Vec3i& face = _mesh->face(i);
            flat.push_back(static_cast<unsigned>(face(0)));
            flat.push_back(static_cast<unsigned>(face(1)));
            flat.push_back(static_cast<unsigned>(face(2)));
        }
    } else {
        for (const auto& face : faces.colwise()) {
            flat.push_back(static_cast<unsigned>(face(0)));
            flat.push_back(static_cast<unsigned>(face(1)));
            flat.push_back(static_cast<unsigned>(face(2)));
        }
    }
    return flat;
}

std::vector<unsigned int> OpenGLMeshGraphicsObject::_edgesAsFlatList() const
{
    // Check for 1D mesh (line segments)
    if (_mesh->hasVertexProperty<int>("has_line_segments")) {
        std::vector<unsigned int> edges;
        for (int i = 0; i < _mesh->numVertices() - 1; ++i) {
            edges.push_back(static_cast<unsigned int>(i));
            edges.push_back(static_cast<unsigned int>(i + 1));
        }
        return edges;
    }

    // Extract edges from faces
    const auto& faces = _mesh->faces();
    std::vector<unsigned int> edges;
    edges.reserve(_mesh->numFaces() * 6);

    bool has_draw_property = _mesh->hasFaceProperty<bool>("draw");

    if (has_draw_property) {
        const auto& draw_face = _mesh->getFaceProperty<bool>("draw").properties();
        for (int i = 0; i < _mesh->numFaces(); i++) {
            if (!draw_face[i]) continue;
            const Vec3i& face = _mesh->face(i);
            edges.insert(edges.end(), {
                static_cast<unsigned>(face(0)), static_cast<unsigned>(face(1)),
                static_cast<unsigned>(face(1)), static_cast<unsigned>(face(2)),
                static_cast<unsigned>(face(0)), static_cast<unsigned>(face(2))
            });
        }
    } else {
        for (const auto& face : faces.colwise()) {
            edges.insert(edges.end(), {
                static_cast<unsigned>(face(0)), static_cast<unsigned>(face(1)),
                static_cast<unsigned>(face(1)), static_cast<unsigned>(face(2)),
                static_cast<unsigned>(face(0)), static_cast<unsigned>(face(2))
            });
        }
    }
    return edges;
}

void OpenGLMeshGraphicsObject::setTexture(const std::string& texture_path)
{
    std::cout << "[OpenGL] Loading texture: " << texture_path << std::endl;

    if (!_mesh->hasUVCoords()) {
        std::cerr << "[OpenGL] ERROR: Mesh has no UV coordinates for texture mapping!" << std::endl;
        return;
    }

    // Store path; actual GL texture creation happens lazily in _ensureGLInitialized
    _pending_texture_path = texture_path;
}

void OpenGLMeshGraphicsObject::_loadTexture()
{
    if (_pending_texture_path.empty()) return;

    std::string texture_path = _pending_texture_path;
    _pending_texture_path.clear();

    // Load image with stb_image
    stbi_set_flip_vertically_on_load(true);
    int width, height, channels;
    unsigned char* data = stbi_load(texture_path.c_str(), &width, &height, &channels, 0);
    if (!data) {
        std::cerr << "[OpenGL] ERROR: Failed to load texture: " << texture_path
                  << " (" << stbi_failure_reason() << ")" << std::endl;
        return;
    }

    GLenum format = GL_RGB;
    if (channels == 1) format = GL_RED;
    else if (channels == 3) format = GL_RGB;
    else if (channels == 4) format = GL_RGBA;

    glGenTextures(1, &_texture_id);
    glBindTexture(GL_TEXTURE_2D, _texture_id);
    glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
    glGenerateMipmap(GL_TEXTURE_2D);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glBindTexture(GL_TEXTURE_2D, 0);
    stbi_image_free(data);

    std::cout << "[OpenGL] Texture loaded: " << texture_path
              << " (" << width << "x" << height << ", " << channels << " channels)" << std::endl;

    _has_texture = true;

    // Upload UV coords to faces VAO
    if (_faces_vao && _mesh->hasUVCoords()) {
        const auto& uv_coords = _mesh->uvCoords();
        std::vector<float> texcoords(uv_coords.cols() * 2);
        for (int i = 0; i < uv_coords.cols(); i++) {
            texcoords[i*2+0] = static_cast<float>(uv_coords(0, i));
            texcoords[i*2+1] = static_cast<float>(uv_coords(1, i));
        }

        glBindVertexArray(_faces_vao);
        glGenBuffers(1, &_texcoord_vbo);
        glBindBuffer(GL_ARRAY_BUFFER, _texcoord_vbo);
        glBufferData(GL_ARRAY_BUFFER, texcoords.size() * sizeof(float), texcoords.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(2);  // location 2 for texcoords
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
        glBindVertexArray(0);
    }
}

} // namespace Graphics
