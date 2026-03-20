#include "graphics/opengl/OpenGLGraphicsScene.hpp"
#include "graphics/opengl/OpenGLMeshGraphicsObject.hpp"
#include "graphics/opengl/OpenGLStaticModel.hpp"

#include <GL/glew.h>

#include "simobject/MeshObject.hpp"
#include "simobject/RigidPrimitives.hpp"
#include "graphics/opengl/stb_image.h"

#include <cmath>
#include <iostream>
#include <cassert>

namespace Graphics
{

// ==================== Mesh Shader (Blinn-Phong) ====================

static const char* s_mesh_vert_src = R"(
#version 330 core
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aTexCoord;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;
uniform mat3 uNormalMatrix;

out vec3 vWorldPos;
out vec3 vNormal;
out vec2 vTexCoord;

void main() {
    vec4 worldPos = uModel * vec4(aPosition, 1.0);
    vWorldPos = worldPos.xyz;
    vNormal = normalize(uNormalMatrix * aNormal);
    vTexCoord = aTexCoord;
    gl_Position = uProjection * uView * worldPos;
}
)";

static const char* s_mesh_frag_src = R"(
#version 330 core
in vec3 vWorldPos;
in vec3 vNormal;
in vec2 vTexCoord;

uniform vec4 uColor;
uniform vec3 uLightDir;
uniform vec3 uViewPos;
uniform int uUseLighting;
uniform int uUseTexture;
uniform sampler2D uTexture;
uniform float uMetallic;    // 0 = dielectric, 1 = metal
uniform float uRoughness;   // 0 = mirror, 1 = rough

out vec4 fragColor;

void main() {
    vec4 baseColor = uColor;
    if (uUseTexture == 1) {
        baseColor = texture(uTexture, vTexCoord);
    }

    if (uUseLighting == 1 && length(vNormal) > 0.001) {
        vec3 N = normalize(vNormal);
        vec3 L = normalize(uLightDir);
        vec3 V = normalize(uViewPos - vWorldPos);
        vec3 H = normalize(L + V);
        vec3 R = reflect(-V, N);

        float NdotL = max(dot(N, L), 0.0);
        float NdotH = max(dot(N, H), 0.0);
        float NdotV = max(dot(N, V), 0.001);

        // Two-sided lighting
        float NdotLBack = max(dot(-N, L), 0.0);
        NdotL = max(NdotL, NdotLBack * 0.6);

        if (uMetallic > 0.01) {
            // === Metallic material (procedural) ===
            vec3 metalColor = baseColor.rgb;

            // Fresnel: metals reflect their own color, stronger at grazing angles
            float fresnel = pow(1.0 - NdotV, 5.0);
            vec3 F0 = metalColor * 0.9;  // base reflectivity = metal color
            vec3 F = F0 + (1.0 - F0) * fresnel;

            // Specular: tighter highlight for smoother surfaces
            float shininess = mix(256.0, 16.0, uRoughness);
            float spec = pow(NdotH, shininess);

            // Broader secondary specular for softer fill
            float spec2 = pow(NdotH, shininess * 0.25);

            // Ambient: metallic reflection tint
            vec3 ambient = metalColor * 0.15;

            // Diffuse: metals have very little diffuse
            vec3 diffuse = metalColor * NdotL * 0.15 * uRoughness;

            // Primary specular reflection
            vec3 specular = F * spec * 1.2;

            // Secondary soft specular
            vec3 specular2 = F * spec2 * 0.15;

            // Fake environment reflection (gradient based on reflection direction)
            vec3 envColor = mix(
                vec3(0.15, 0.18, 0.22),  // dark blueish (down/shadow)
                vec3(0.6, 0.65, 0.7),    // light gray-blue (up/sky)
                R.z * 0.5 + 0.5
            );
            vec3 envReflection = envColor * F * mix(0.6, 0.1, uRoughness);

            vec3 result = ambient + diffuse + specular + specular2 + envReflection;

            // Tone mapping to prevent blowout
            result = result / (result + vec3(1.0));

            fragColor = vec4(result, baseColor.a);
        } else {
            // === Non-metallic (original Blinn-Phong) ===
            float ambient = 0.3;
            float diff = NdotL;
            float spec = pow(NdotH, 32.0);

            vec3 lighting = baseColor.rgb * (ambient + diff * 0.7) + vec3(1.0) * spec * 0.3;
            fragColor = vec4(lighting, baseColor.a);
        }
    } else {
        fragColor = baseColor;
    }
}
)";

// ==================== Static Model Shader (Blinn-Phong + Normal Map) ====================

static const char* s_static_vert_src = R"(
#version 330 core
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aTexCoord;
layout(location = 3) in vec3 aTangent;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;
uniform mat3 uNormalMatrix;

out vec3 vWorldPos;
out vec3 vNormal;
out vec2 vTexCoord;
out mat3 vTBN;

void main() {
    vec4 worldPos = uModel * vec4(aPosition, 1.0);
    vWorldPos = worldPos.xyz;
    vNormal = normalize(uNormalMatrix * aNormal);
    vTexCoord = aTexCoord;

    // Build TBN matrix for normal mapping
    vec3 T = normalize(uNormalMatrix * aTangent);
    vec3 N = vNormal;
    T = normalize(T - dot(T, N) * N);  // re-orthogonalize
    vec3 B = cross(N, T);
    vTBN = mat3(T, B, N);

    gl_Position = uProjection * uView * worldPos;
}
)";

static const char* s_static_frag_src = R"(
#version 330 core
in vec3 vWorldPos;
in vec3 vNormal;
in vec2 vTexCoord;
in mat3 vTBN;

uniform vec4 uColor;
uniform vec3 uLightDir;
uniform vec3 uViewPos;
uniform int uUseLighting;
uniform int uUseTexture;
uniform sampler2D uTexture;
uniform int uUseNormalMap;
uniform sampler2D uNormalMap;
uniform float uRoughness;

out vec4 fragColor;

void main() {
    vec4 baseColor = uColor;
    if (uUseTexture == 1) {
        baseColor = texture(uTexture, vTexCoord);
    }

    if (uUseLighting == 1) {
        vec3 N = normalize(vNormal);

        // Apply normal map if available
        if (uUseNormalMap == 1) {
            vec3 mapNormal = texture(uNormalMap, vTexCoord).rgb;
            mapNormal = mapNormal * 2.0 - 1.0;  // [0,1] -> [-1,1]
            N = normalize(vTBN * mapNormal);
        }

        vec3 L = normalize(uLightDir);
        vec3 V = normalize(uViewPos - vWorldPos);
        vec3 H = normalize(L + V);

        // Roughness controls specular: higher roughness = lower shininess
        float shininess = mix(128.0, 4.0, uRoughness);

        float ambient = 0.25;
        float diff = max(dot(N, L), 0.0);
        float spec = pow(max(dot(N, H), 0.0), shininess);

        // Two-sided lighting
        float diffBack = max(dot(-N, L), 0.0);
        diff = max(diff, diffBack * 0.6);

        // Reduce specular for rough surfaces
        float specStrength = mix(0.4, 0.05, uRoughness);

        vec3 lighting = baseColor.rgb * (ambient + diff * 0.75) + vec3(1.0) * spec * specStrength;
        fragColor = vec4(lighting, baseColor.a);
    } else {
        fragColor = baseColor;
    }
}
)";

static GLuint compileShaderSrc(GLenum type, const char* src)
{
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);
    GLint ok = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetShaderInfoLog(shader, 512, nullptr, log);
        std::cerr << "[OpenGL] Shader compile error: " << log << std::endl;
    }
    return shader;
}

static GLuint linkProgram(GLuint vs, GLuint fs)
{
    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glLinkProgram(prog);
    GLint ok = 0;
    glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetProgramInfoLog(prog, 512, nullptr, log);
        std::cerr << "[OpenGL] Shader link error: " << log << std::endl;
    }
    glDeleteShader(vs);
    glDeleteShader(fs);
    return prog;
}

// ==================== Constructor / Destructor ====================

OpenGLGraphicsScene::OpenGLGraphicsScene(const std::string& name, const Config::SimulationRenderConfig& sim_render_config)
    : GraphicsScene(name, sim_render_config)
{
}

OpenGLGraphicsScene::~OpenGLGraphicsScene()
{
    _static_models.clear();  // must destroy before GL context dies
    if (_mesh_shader) glDeleteProgram(_mesh_shader);
    if (_static_model_shader) glDeleteProgram(_static_model_shader);
}

// ==================== Init ====================

void OpenGLGraphicsScene::init()
{
    // Create viewer (GLFW window will be created in run())
    int win_w = _sim_render_config.windowWidth();
    int win_h = _sim_render_config.windowHeight();
    _viewer = std::make_unique<OpenGLViewer>(_name, win_w, win_h);
    _opengl_viewer = dynamic_cast<OpenGLViewer*>(_viewer.get());

    // Pass background UV offsets
    _opengl_viewer->setBackgroundOffset(
        static_cast<float>(_sim_render_config.backgroundUOffset()),
        static_cast<float>(_sim_render_config.backgroundVOffset()));

    // Pass initial camera config
    {
        const auto& cfg = _sim_render_config;
        std::optional<Eigen::Vector3f> pos, view_dir, up_dir;
        std::optional<float> fov;
        if (cfg.cameraPosition().has_value()) {
            const auto& p = cfg.cameraPosition().value();
            pos = Eigen::Vector3f(static_cast<float>(p(0)), static_cast<float>(p(1)), static_cast<float>(p(2)));
        }
        if (cfg.cameraViewDirection().has_value()) {
            const auto& v = cfg.cameraViewDirection().value();
            view_dir = Eigen::Vector3f(static_cast<float>(v(0)), static_cast<float>(v(1)), static_cast<float>(v(2)));
        }
        if (cfg.cameraUpDirection().has_value()) {
            const auto& u = cfg.cameraUpDirection().value();
            up_dir = Eigen::Vector3f(static_cast<float>(u(0)), static_cast<float>(u(1)), static_cast<float>(u(2)));
        }
        if (cfg.cameraFOV().has_value()) {
            fov = static_cast<float>(cfg.cameraFOV().value());
        }
        _opengl_viewer->setInitialCameraConfig(pos, view_dir, up_dir, fov);
    }

    // Set draw callback
    _opengl_viewer->setDrawCallback(&OpenGLGraphicsScene::_drawSceneCallback, this);
}

void OpenGLGraphicsScene::_initShaders()
{
    {
        GLuint vs = compileShaderSrc(GL_VERTEX_SHADER, s_mesh_vert_src);
        GLuint fs = compileShaderSrc(GL_FRAGMENT_SHADER, s_mesh_frag_src);
        _mesh_shader = linkProgram(vs, fs);
        std::cout << "[OpenGL] Mesh shader initialized." << std::endl;
    }
    {
        GLuint vs = compileShaderSrc(GL_VERTEX_SHADER, s_static_vert_src);
        GLuint fs = compileShaderSrc(GL_FRAGMENT_SHADER, s_static_frag_src);
        _static_model_shader = linkProgram(vs, fs);
        std::cout << "[OpenGL] Static model shader initialized." << std::endl;
    }
}

// ==================== Update / Run ====================

void OpenGLGraphicsScene::update()
{
    for (auto& obj : _graphics_objects) {
        obj->update();
    }
    // Note: viewer update (swap buffers) happens in the render loop
}

int OpenGLGraphicsScene::run()
{
    // Ensure window + GL context exist before loading GPU resources
    _opengl_viewer->initWindow();

    // Load background texture now that GL context is ready
    if (_sim_render_config.backgroundImage().has_value())
    {
        const std::string& bg_path = _sim_render_config.backgroundImage().value();
        stbi_set_flip_vertically_on_load(true);
        int w, h, ch;
        unsigned char* data = stbi_load(bg_path.c_str(), &w, &h, &ch, 0);
        if (data) {
            GLenum fmt = (ch == 4) ? GL_RGBA : GL_RGB;
            GLuint tex;
            glGenTextures(1, &tex);
            glBindTexture(GL_TEXTURE_2D, tex);
            glTexImage2D(GL_TEXTURE_2D, 0, fmt, w, h, 0, fmt, GL_UNSIGNED_BYTE, data);
            glGenerateMipmap(GL_TEXTURE_2D);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glBindTexture(GL_TEXTURE_2D, 0);
            stbi_image_free(data);

            _opengl_viewer->setBackgroundTexture(tex);
            std::cout << "[OpenGL] Background image loaded: " << bg_path
                      << " (" << w << "x" << h << ")" << std::endl;
        } else {
            std::cerr << "[OpenGL] WARNING: Failed to load background image: " << bg_path
                      << " (" << stbi_failure_reason() << ")" << std::endl;
        }
    }

    return _opengl_viewer->run();
}

// ==================== Draw ====================

void OpenGLGraphicsScene::_drawSceneCallback(void* user_data)
{
    auto* scene = static_cast<OpenGLGraphicsScene*>(user_data);
    scene->_drawScene();
}

void OpenGLGraphicsScene::_drawScene() const
{
    // Lazy-init shaders (need GL context to be current)
    if (!_mesh_shader) {
        const_cast<OpenGLGraphicsScene*>(this)->_initShaders();
    }

    glUseProgram(_mesh_shader);

    // Set matrices
    Eigen::Matrix4f view = _opengl_viewer->viewMatrix();
    Eigen::Matrix4f proj = _opengl_viewer->projectionMatrix();

    // Visual-only rotation: tilt simulation objects around X axis
    // This does NOT affect physics — only how objects appear on screen
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    {
        float tilt_angle = 0.6491f;  // -20 degrees in radians
        float c = std::cos(tilt_angle), s = std::sin(tilt_angle);
        model(1,1) = c;  model(1,2) = -s;
        model(2,1) = s;  model(2,2) =  c;
    }

    // Normal matrix = transpose(inverse(upper-left 3x3 of model))
    Eigen::Matrix3f normalMat = model.block<3,3>(0,0).inverse().transpose();

    glUniformMatrix4fv(glGetUniformLocation(_mesh_shader, "uModel"), 1, GL_FALSE, model.data());
    glUniformMatrix4fv(glGetUniformLocation(_mesh_shader, "uView"), 1, GL_FALSE, view.data());
    glUniformMatrix4fv(glGetUniformLocation(_mesh_shader, "uProjection"), 1, GL_FALSE, proj.data());
    glUniformMatrix3fv(glGetUniformLocation(_mesh_shader, "uNormalMatrix"), 1, GL_FALSE, normalMat.data());

    // Light direction (from camera)
    Eigen::Vector3f cam_pos = _opengl_viewer->cameraPosition();
    Eigen::Vector3f light_dir = _opengl_viewer->cameraViewDirection().normalized() + Eigen::Vector3f(0.3f, 0.3f, 0.5f);
    light_dir.normalize();

    glUniform3f(glGetUniformLocation(_mesh_shader, "uLightDir"), light_dir.x(), light_dir.y(), light_dir.z());
    glUniform3f(glGetUniformLocation(_mesh_shader, "uViewPos"), cam_pos.x(), cam_pos.y(), cam_pos.z());

    // Enable blending for transparency
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Draw all mesh objects
    for (const auto& obj : _graphics_objects) {
        OpenGLMeshGraphicsObject* mesh_obj = dynamic_cast<OpenGLMeshGraphicsObject*>(obj.get());
        if (mesh_obj) {
            mesh_obj->draw(_mesh_shader);
        }
    }

    // Draw sphere objects (grasp cursor, etc.)
    if (!_sphere_objects.empty()) {
        _ensureSphereGL();
        for (const auto& entry : _sphere_objects) {
            const auto* sphere = entry.sphere;
            auto pos = sphere->position();
            float r = static_cast<float>(sphere->radius());

            // Build model matrix: visual tilt * translate * scale
            Eigen::Matrix4f sphere_model = model;  // inherit visual tilt
            // Apply translation in tilt-space
            Eigen::Matrix4f translate = Eigen::Matrix4f::Identity();
            translate(0, 3) = static_cast<float>(pos[0]);
            translate(1, 3) = static_cast<float>(pos[1]);
            translate(2, 3) = static_cast<float>(pos[2]);
            Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();
            scale(0, 0) = r; scale(1, 1) = r; scale(2, 2) = r;
            sphere_model = model * translate * scale;

            Eigen::Matrix3f sphere_norm = sphere_model.block<3,3>(0,0).inverse().transpose();
            glUniformMatrix4fv(glGetUniformLocation(_mesh_shader, "uModel"), 1, GL_FALSE, sphere_model.data());
            glUniformMatrix3fv(glGetUniformLocation(_mesh_shader, "uNormalMatrix"), 1, GL_FALSE, sphere_norm.data());

            glUniform4f(glGetUniformLocation(_mesh_shader, "uColor"),
                        entry.color[0], entry.color[1], entry.color[2], entry.color[3]);
            glUniform1i(glGetUniformLocation(_mesh_shader, "uUseLighting"), 1);
            glUniform1i(glGetUniformLocation(_mesh_shader, "uUseTexture"), 0);
            glUniform1f(glGetUniformLocation(_mesh_shader, "uMetallic"), 0.0f);
            glUniform1f(glGetUniformLocation(_mesh_shader, "uRoughness"), 0.5f);

            glBindVertexArray(_sphere_vao);
            glDrawElements(GL_TRIANGLES, _sphere_num_indices, GL_UNSIGNED_INT, nullptr);
            glBindVertexArray(0);
        }
        // Restore original model matrix
        glUniformMatrix4fv(glGetUniformLocation(_mesh_shader, "uModel"), 1, GL_FALSE, model.data());
        glUniformMatrix3fv(glGetUniformLocation(_mesh_shader, "uNormalMatrix"), 1, GL_FALSE, normalMat.data());
    }

    // Draw static decorative models with normal-mapped shader
    glUseProgram(_static_model_shader);
    glUniformMatrix4fv(glGetUniformLocation(_static_model_shader, "uView"), 1, GL_FALSE, view.data());
    glUniformMatrix4fv(glGetUniformLocation(_static_model_shader, "uProjection"), 1, GL_FALSE, proj.data());
    glUniform3f(glGetUniformLocation(_static_model_shader, "uLightDir"), light_dir.x(), light_dir.y(), light_dir.z());
    glUniform3f(glGetUniformLocation(_static_model_shader, "uViewPos"), cam_pos.x(), cam_pos.y(), cam_pos.z());

    for (const auto& smodel : _static_models) {
        smodel->draw(_static_model_shader);
    }

    glDisable(GL_BLEND);
    glUseProgram(0);
}

// ==================== Add Object ====================

int OpenGLGraphicsScene::addObject(const Sim::Object* obj, const Config::ObjectRenderConfig& obj_config)
{
    if (getObject(obj->name())) {
        std::cout << "GraphicsObject with name " << obj->name() << " already exists in this GraphicsScene!" << std::endl;
        assert(0);
    }

    std::unique_ptr<GraphicsObject> new_graphics_obj;

    // Check if this is a RigidSphere (e.g. grasp cursor)
    if (const Sim::RigidSphere* sphere = dynamic_cast<const Sim::RigidSphere*>(obj)) {
        std::array<float, 4> color = {0.3f, 0.8f, 0.3f, 0.4f};  // semi-transparent green
        if (obj_config.color().has_value()) {
            auto c = obj_config.color().value();
            color = {static_cast<float>(c[0]), static_cast<float>(c[1]), static_cast<float>(c[2]),
                     static_cast<float>(obj_config.opacity())};
        }
        _sphere_objects.push_back({sphere, color});
        std::cout << "[OpenGL] Registered sphere object: " << obj->name() << std::endl;
        return static_cast<int>(_graphics_objects.size());  // dummy index
    }

    // MeshObject support
    if (const Sim::MeshObject* mo = dynamic_cast<const Sim::MeshObject*>(obj)) {
        auto gl_mgo = std::make_unique<OpenGLMeshGraphicsObject>(obj->name(), mo->mesh(), obj_config, mo);

        if (obj_config.textureFile().has_value()) {
            gl_mgo->setTexture(obj_config.textureFile().value());
        }

        new_graphics_obj = std::move(gl_mgo);
    } else {
        std::cerr << "[OpenGL] WARNING: Object type not yet supported for OpenGL backend: " << obj->name() << std::endl;
        return -1;
    }

    _graphics_objects.push_back(std::move(new_graphics_obj));
    return static_cast<int>(_graphics_objects.size()) - 1;
}

// ==================== Camera Accessors ====================

void OpenGLGraphicsScene::setCameraOrthographic() { _opengl_viewer->setCameraOrthographic(); }
void OpenGLGraphicsScene::setCameraPerspective() { _opengl_viewer->setCameraPerspective(); }
void OpenGLGraphicsScene::setCameraFOV(Real fov) { _opengl_viewer->setCameraFOV(static_cast<float>(fov)); }

Vec3r OpenGLGraphicsScene::cameraViewDirection() const {
    auto d = _opengl_viewer->cameraViewDirection();
    return Vec3r(d.x(), d.y(), d.z());
}
void OpenGLGraphicsScene::setCameraViewDirection(const Vec3r& view_dir) {
    _opengl_viewer->setCameraViewDirection(Eigen::Vector3f(
        static_cast<float>(view_dir(0)), static_cast<float>(view_dir(1)), static_cast<float>(view_dir(2))));
}

Vec3r OpenGLGraphicsScene::cameraUpDirection() const {
    auto u = _opengl_viewer->cameraUpDirection();
    return Vec3r(u.x(), u.y(), u.z());
}
void OpenGLGraphicsScene::setCameraUpDirection(const Vec3r& up_dir) {
    _opengl_viewer->setCameraUpDirection(Eigen::Vector3f(
        static_cast<float>(up_dir(0)), static_cast<float>(up_dir(1)), static_cast<float>(up_dir(2))));
}

Vec3r OpenGLGraphicsScene::cameraRightDirection() const {
    auto r = _opengl_viewer->cameraRightDirection();
    return Vec3r(r.x(), r.y(), r.z());
}

Vec3r OpenGLGraphicsScene::cameraPosition() const {
    auto p = _opengl_viewer->cameraPosition();
    return Vec3r(p.x(), p.y(), p.z());
}
void OpenGLGraphicsScene::setCameraPosition(const Vec3r& position) {
    _opengl_viewer->setCameraPosition(Eigen::Vector3f(
        static_cast<float>(position(0)), static_cast<float>(position(1)), static_cast<float>(position(2))));
}

// ==================== Static Models ====================

void OpenGLGraphicsScene::addStaticModel(const std::string& filepath, const Eigen::Matrix4f& transform)
{
    _static_models.push_back(std::make_unique<OpenGLStaticModel>(filepath, transform));
}

// ==================== Sphere Rendering ====================

void OpenGLGraphicsScene::_ensureSphereGL() const
{
    if (_sphere_vao) return;

    // Generate UV sphere (unit radius, centered at origin)
    const int stacks = 16, slices = 24;
    std::vector<float> positions, normals;
    std::vector<unsigned int> indices;

    for (int i = 0; i <= stacks; ++i) {
        float phi = M_PI * float(i) / float(stacks);
        float sp = std::sin(phi), cp = std::cos(phi);
        for (int j = 0; j <= slices; ++j) {
            float theta = 2.0f * M_PI * float(j) / float(slices);
            float st = std::sin(theta), ct = std::cos(theta);
            float x = sp * ct, y = sp * st, z = cp;
            positions.push_back(x); positions.push_back(y); positions.push_back(z);
            normals.push_back(x);   normals.push_back(y);   normals.push_back(z);
        }
    }
    for (int i = 0; i < stacks; ++i) {
        for (int j = 0; j < slices; ++j) {
            int a = i * (slices + 1) + j;
            int b = a + slices + 1;
            indices.push_back(a); indices.push_back(b); indices.push_back(a + 1);
            indices.push_back(a + 1); indices.push_back(b); indices.push_back(b + 1);
        }
    }
    _sphere_num_indices = static_cast<int>(indices.size());

    glGenVertexArrays(1, &_sphere_vao);
    glBindVertexArray(_sphere_vao);

    glGenBuffers(1, &_sphere_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, _sphere_vbo);
    glBufferData(GL_ARRAY_BUFFER, positions.size() * sizeof(float), positions.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    glGenBuffers(1, &_sphere_nbo);
    glBindBuffer(GL_ARRAY_BUFFER, _sphere_nbo);
    glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(float), normals.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    glGenBuffers(1, &_sphere_ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _sphere_ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    glBindVertexArray(0);
}

} // namespace Graphics
