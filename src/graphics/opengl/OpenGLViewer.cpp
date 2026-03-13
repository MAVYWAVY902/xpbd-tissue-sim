#include "graphics/opengl/OpenGLViewer.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <iostream>
#include <cmath>

namespace Graphics
{

// ==================== GLFW Key Mapping ====================

const std::map<int, SimulationInput::Key> OpenGLViewer::_glfw_key_map =
{
    {GLFW_KEY_UNKNOWN, SimulationInput::Key::UNKNOWN},
    {GLFW_KEY_A, SimulationInput::Key::A},
    {GLFW_KEY_B, SimulationInput::Key::B},
    {GLFW_KEY_C, SimulationInput::Key::C},
    {GLFW_KEY_D, SimulationInput::Key::D},
    {GLFW_KEY_E, SimulationInput::Key::E},
    {GLFW_KEY_F, SimulationInput::Key::F},
    {GLFW_KEY_G, SimulationInput::Key::G},
    {GLFW_KEY_H, SimulationInput::Key::H},
    {GLFW_KEY_I, SimulationInput::Key::I},
    {GLFW_KEY_J, SimulationInput::Key::J},
    {GLFW_KEY_K, SimulationInput::Key::K},
    {GLFW_KEY_L, SimulationInput::Key::L},
    {GLFW_KEY_M, SimulationInput::Key::M},
    {GLFW_KEY_N, SimulationInput::Key::N},
    {GLFW_KEY_O, SimulationInput::Key::O},
    {GLFW_KEY_P, SimulationInput::Key::P},
    {GLFW_KEY_Q, SimulationInput::Key::Q},
    {GLFW_KEY_R, SimulationInput::Key::R},
    {GLFW_KEY_S, SimulationInput::Key::S},
    {GLFW_KEY_T, SimulationInput::Key::T},
    {GLFW_KEY_U, SimulationInput::Key::U},
    {GLFW_KEY_V, SimulationInput::Key::V},
    {GLFW_KEY_W, SimulationInput::Key::W},
    {GLFW_KEY_X, SimulationInput::Key::X},
    {GLFW_KEY_Y, SimulationInput::Key::Y},
    {GLFW_KEY_Z, SimulationInput::Key::Z},
    {GLFW_KEY_1, SimulationInput::Key::ONE},
    {GLFW_KEY_2, SimulationInput::Key::TWO},
    {GLFW_KEY_3, SimulationInput::Key::THREE},
    {GLFW_KEY_4, SimulationInput::Key::FOUR},
    {GLFW_KEY_5, SimulationInput::Key::FIVE},
    {GLFW_KEY_6, SimulationInput::Key::SIX},
    {GLFW_KEY_7, SimulationInput::Key::SEVEN},
    {GLFW_KEY_8, SimulationInput::Key::EIGHT},
    {GLFW_KEY_9, SimulationInput::Key::NINE},
    {GLFW_KEY_0, SimulationInput::Key::ZERO},
    {GLFW_KEY_F1, SimulationInput::Key::F1},
    {GLFW_KEY_F2, SimulationInput::Key::F2},
    {GLFW_KEY_F3, SimulationInput::Key::F3},
    {GLFW_KEY_F4, SimulationInput::Key::F4},
    {GLFW_KEY_F5, SimulationInput::Key::F5},
    {GLFW_KEY_F6, SimulationInput::Key::F6},
    {GLFW_KEY_F7, SimulationInput::Key::F7},
    {GLFW_KEY_F8, SimulationInput::Key::F8},
    {GLFW_KEY_F9, SimulationInput::Key::F9},
    {GLFW_KEY_SPACE, SimulationInput::Key::SPACE},
    {GLFW_KEY_UP, SimulationInput::Key::UP},
    {GLFW_KEY_DOWN, SimulationInput::Key::DOWN},
    {GLFW_KEY_RIGHT, SimulationInput::Key::RIGHT},
    {GLFW_KEY_LEFT, SimulationInput::Key::LEFT},
    {GLFW_KEY_LEFT_BRACKET, SimulationInput::Key::LEFT_BRACKET},
    {GLFW_KEY_RIGHT_BRACKET, SimulationInput::Key::RIGHT_BRACKET},
    {GLFW_KEY_BACKSLASH, SimulationInput::Key::BACKSLASH},
    {GLFW_KEY_SLASH, SimulationInput::Key::SLASH},
    {GLFW_KEY_COMMA, SimulationInput::Key::COMMA},
    {GLFW_KEY_PERIOD, SimulationInput::Key::PERIOD},
    {GLFW_KEY_MINUS, SimulationInput::Key::DASH},
    {GLFW_KEY_EQUAL, SimulationInput::Key::EQUALS},
    {GLFW_KEY_SEMICOLON, SimulationInput::Key::SEMICOLON},
    {GLFW_KEY_LEFT_ALT, SimulationInput::Key::ALT},
    {GLFW_KEY_RIGHT_ALT, SimulationInput::Key::ALT},
    {GLFW_KEY_TAB, SimulationInput::Key::TAB},
    {GLFW_KEY_ENTER, SimulationInput::Key::ENTER},
    {GLFW_KEY_ESCAPE, SimulationInput::Key::ESC},
};

// ==================== Constructor / Destructor ====================

OpenGLViewer::OpenGLViewer(const std::string& title, int width, int height)
    : Viewer(title), _width(width), _height(height)
{
}

OpenGLViewer::~OpenGLViewer()
{
    if (_bg_vao) { glDeleteVertexArrays(1, &_bg_vao); }
    if (_bg_vbo) { glDeleteBuffers(1, &_bg_vbo); }
    if (_bg_ebo) { glDeleteBuffers(1, &_bg_ebo); }
    if (_bg_shader) { glDeleteProgram(_bg_shader); }

    if (_window) {
        glfwDestroyWindow(_window);
    }
    glfwTerminate();
}

// ==================== GLFW Initialization ====================

void OpenGLViewer::_initGLFW()
{
    if (!glfwInit()) {
        std::cerr << "[OpenGL] Failed to initialize GLFW!" << std::endl;
        return;
    }

    // Request OpenGL 3.3 core profile
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4); // 4x MSAA

    _window = glfwCreateWindow(_width, _height, _name.c_str(), nullptr, nullptr);
    if (!_window) {
        std::cerr << "[OpenGL] Failed to create GLFW window!" << std::endl;
        glfwTerminate();
        return;
    }

    glfwMakeContextCurrent(_window);
    glfwSwapInterval(1); // vsync

    // Initialize GLEW
    glewExperimental = GL_TRUE;
    GLenum err = glewInit();
    if (err != GLEW_OK) {
        std::cerr << "[OpenGL] Failed to initialize GLEW: " << glewGetErrorString(err) << std::endl;
        return;
    }

    // Set user pointer so GLFW callbacks can find this object
    glfwSetWindowUserPointer(_window, this);

    // Register GLFW callbacks
    glfwSetKeyCallback(_window, _keyCallback);
    glfwSetMouseButtonCallback(_window, _mouseButtonCallback);
    glfwSetCursorPosCallback(_window, _cursorPosCallback);
    glfwSetScrollCallback(_window, _scrollCallback);
    glfwSetFramebufferSizeCallback(_window, _framebufferSizeCallback);

    // Get actual framebuffer size (may differ from window size on HiDPI)
    glfwGetFramebufferSize(_window, &_width, &_height);

    // OpenGL state
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    glClearColor(0.9f, 0.9f, 0.9f, 1.0f);

    std::cout << "[OpenGL] Viewer initialized: " << glGetString(GL_VERSION) << std::endl;
}

// ==================== Camera ====================

Eigen::Matrix4f OpenGLViewer::viewMatrix() const
{
    if (_view_dirty) {
        Eigen::Vector3f target = _cam_position + _cam_view_dir;
        // lookAt matrix
        Eigen::Vector3f f = (_cam_view_dir).normalized();
        Eigen::Vector3f s = f.cross(_cam_up).normalized();
        Eigen::Vector3f u = s.cross(f);

        Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
        m(0,0) = s.x();  m(0,1) = s.y();  m(0,2) = s.z();
        m(1,0) = u.x();  m(1,1) = u.y();  m(1,2) = u.z();
        m(2,0) = -f.x(); m(2,1) = -f.y(); m(2,2) = -f.z();
        m(0,3) = -s.dot(_cam_position);
        m(1,3) = -u.dot(_cam_position);
        m(2,3) = f.dot(_cam_position);

        _cached_view = m;
        _view_dirty = false;
    }
    return _cached_view;
}

Eigen::Matrix4f OpenGLViewer::projectionMatrix() const
{
    if (_proj_dirty) {
        float aspect = static_cast<float>(_width) / static_cast<float>(_height);
        if (_perspective) {
            float fov_rad = _fov_degrees * static_cast<float>(M_PI) / 180.0f;
            float tanHalfFov = std::tan(fov_rad / 2.0f);
            float near = 0.01f;
            float far = 100.0f;

            Eigen::Matrix4f p = Eigen::Matrix4f::Zero();
            p(0,0) = 1.0f / (aspect * tanHalfFov);
            p(1,1) = 1.0f / tanHalfFov;
            p(2,2) = -(far + near) / (far - near);
            p(2,3) = -(2.0f * far * near) / (far - near);
            p(3,2) = -1.0f;
            _cached_proj = p;
        } else {
            float ortho_size = 2.0f;
            Eigen::Matrix4f p = Eigen::Matrix4f::Zero();
            p(0,0) = 1.0f / (aspect * ortho_size);
            p(1,1) = 1.0f / ortho_size;
            p(2,2) = -2.0f / 100.0f;
            p(2,3) = -1.0f;
            p(3,3) = 1.0f;
            _cached_proj = p;
        }
        _proj_dirty = false;
    }
    return _cached_proj;
}

void OpenGLViewer::setInitialCameraConfig(
    const std::optional<Eigen::Vector3f>& pos,
    const std::optional<Eigen::Vector3f>& view_dir,
    const std::optional<Eigen::Vector3f>& up_dir,
    const std::optional<float>& fov)
{
    _pending_camera_position = pos;
    _pending_camera_view_dir = view_dir;
    _pending_camera_up_dir = up_dir;
    _pending_camera_fov = fov;
    _pending_camera_applied = false;
}

void OpenGLViewer::_applyPendingCamera()
{
    if (_pending_camera_applied) return;
    _pending_camera_applied = true;

    if (_pending_camera_position.has_value()) {
        _cam_position = _pending_camera_position.value();
        _view_dirty = true;
        std::cout << "[Camera] Position applied" << std::endl;
    }
    if (_pending_camera_view_dir.has_value()) {
        _cam_view_dir = _pending_camera_view_dir.value().normalized();
        _view_dirty = true;
        std::cout << "[Camera] View direction applied" << std::endl;
    }
    if (_pending_camera_up_dir.has_value()) {
        _cam_up = _pending_camera_up_dir.value().normalized();
        _view_dirty = true;
        std::cout << "[Camera] Up direction applied" << std::endl;
    }
    if (_pending_camera_fov.has_value()) {
        _fov_degrees = _pending_camera_fov.value();
        _proj_dirty = true;
        std::cout << "[Camera] FOV applied: " << _fov_degrees << " degrees" << std::endl;
    }
}

// ==================== Mouse Interaction ====================

void OpenGLViewer::_handleMouseOrbit(double dx, double dy)
{
    float sensitivity = 0.002f;
    float yaw = static_cast<float>(-dx) * sensitivity;
    float pitch = static_cast<float>(-dy) * sensitivity;

    // Compute rotation around up axis (yaw)
    Eigen::AngleAxisf yaw_rot(yaw, _cam_up);
    _cam_view_dir = yaw_rot * _cam_view_dir;

    // Compute rotation around right axis (pitch)
    Eigen::Vector3f right = _cam_view_dir.cross(_cam_up).normalized();
    Eigen::AngleAxisf pitch_rot(pitch, right);
    Eigen::Vector3f new_view_dir = pitch_rot * _cam_view_dir;

    // Prevent flipping past poles
    if (std::abs(new_view_dir.normalized().dot(_cam_up)) < 0.99f) {
        _cam_view_dir = new_view_dir.normalized();
    }

    _view_dirty = true;
}

void OpenGLViewer::_handleMousePan(double dx, double dy)
{
    float sensitivity = 0.0008f;
    Eigen::Vector3f right = _cam_view_dir.cross(_cam_up).normalized();
    Eigen::Vector3f up = right.cross(_cam_view_dir).normalized();

    _cam_position -= right * static_cast<float>(dx) * sensitivity;
    _cam_position += up * static_cast<float>(dy) * sensitivity;
    _view_dirty = true;
}

void OpenGLViewer::_handleMouseZoom(double dy)
{
    float sensitivity = 0.05f;
    _cam_position += _cam_view_dir.normalized() * static_cast<float>(dy) * sensitivity;
    _view_dirty = true;
}

// ==================== Background Rendering ====================

static const char* s_bg_vert_src = R"(
#version 330 core
layout(location = 0) in vec2 vtx_position;
out vec2 vNDC;
void main() {
    vNDC = vtx_position;
    gl_Position = vec4(vtx_position, 0.999, 1.0);
}
)";

static const char* s_bg_frag_src = R"(
#version 330 core
in vec2 vNDC;
out vec4 fragOutput;
uniform sampler2D uTexture;
uniform mat4 uInverseVPRot;
uniform float uUOffset;
uniform float uVOffset;

const float PI = 3.14159265359;

void main() {
    vec4 worldDir = uInverseVPRot * vec4(vNDC, -1.0, 1.0);
    vec3 dir = normalize(worldDir.xyz / worldDir.w);

    float u = atan(-dir.y, dir.x) / (2.0 * PI) + 0.5 + uUOffset;
    float v = asin(clamp(dir.z, -1.0, 1.0)) / PI + 0.5 + uVOffset;

    fragOutput = texture(uTexture, vec2(u, v));
}
)";

static GLuint compileShader(GLenum type, const char* src)
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

void OpenGLViewer::_initBackground()
{
    GLuint vs = compileShader(GL_VERTEX_SHADER, s_bg_vert_src);
    GLuint fs = compileShader(GL_FRAGMENT_SHADER, s_bg_frag_src);
    _bg_shader = glCreateProgram();
    glAttachShader(_bg_shader, vs);
    glAttachShader(_bg_shader, fs);
    glLinkProgram(_bg_shader);
    GLint ok = 0;
    glGetProgramiv(_bg_shader, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetProgramInfoLog(_bg_shader, 512, nullptr, log);
        std::cerr << "[OpenGL] Background shader link error: " << log << std::endl;
    }
    glDeleteShader(vs);
    glDeleteShader(fs);

    static const float verts[] = {
        -1.0f, -1.0f,
         1.0f, -1.0f,
         1.0f,  1.0f,
        -1.0f,  1.0f
    };
    static const unsigned int indices[] = { 0, 1, 2, 0, 2, 3 };

    glGenVertexArrays(1, &_bg_vao);
    glGenBuffers(1, &_bg_vbo);
    glGenBuffers(1, &_bg_ebo);

    glBindVertexArray(_bg_vao);
    glBindBuffer(GL_ARRAY_BUFFER, _bg_vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _bg_ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
    glBindVertexArray(0);

    _bg_shader_initialized = true;
    std::cout << "[OpenGL] Equirectangular background shader initialized." << std::endl;
}

void OpenGLViewer::_drawBackground() const
{
    if (!_bg_shader_initialized)
        const_cast<OpenGLViewer*>(this)->_initBackground();

    // Strip translation from view matrix, keep only rotation
    Eigen::Matrix4f view = viewMatrix();
    view(0, 3) = 0.0f;
    view(1, 3) = 0.0f;
    view(2, 3) = 0.0f;

    Eigen::Matrix4f vpRot = projectionMatrix() * view;
    Eigen::Matrix4f invVPRot = vpRot.inverse();

    glDepthMask(GL_FALSE);
    glUseProgram(_bg_shader);

    glUniform1i(glGetUniformLocation(_bg_shader, "uTexture"), 0);
    glUniformMatrix4fv(glGetUniformLocation(_bg_shader, "uInverseVPRot"), 1, GL_FALSE, invVPRot.data());
    glUniform1f(glGetUniformLocation(_bg_shader, "uUOffset"), _bg_u_offset);
    glUniform1f(glGetUniformLocation(_bg_shader, "uVOffset"), _bg_v_offset);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, _background_texture_id);

    glBindVertexArray(_bg_vao);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);

    glUseProgram(0);
    glDepthMask(GL_TRUE);
}

// ==================== Main Loop ====================

void OpenGLViewer::update()
{
    if (_window) {
        glfwSwapBuffers(_window);
        glfwPollEvents();
    }
}

int OpenGLViewer::width() const { return _width; }
int OpenGLViewer::height() const { return _height; }

void OpenGLViewer::initWindow()
{
    if (!_window) {
        _initGLFW();
        _applyPendingCamera();
    }
}

int OpenGLViewer::run()
{
    initWindow();
    if (!_window) return -1;

    while (!glfwWindowShouldClose(_window))
    {
        glfwPollEvents();

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Draw background
        if (_background_texture_id != 0) {
            _drawBackground();
        }

        // Call external draw callback (scene rendering)
        if (_draw_callback) {
            _draw_callback(_draw_user_data);
        }

        glfwSwapBuffers(_window);
    }

    return 0;
}

// ==================== GLFW Callbacks ====================

void OpenGLViewer::_keyCallback(GLFWwindow* window, int key, int /*scancode*/, int action, int mods)
{
    OpenGLViewer* viewer = static_cast<OpenGLViewer*>(glfwGetWindowUserPointer(window));
    if (!viewer) return;

    SimulationInput::Key sim_key = SimulationInput::Key::UNKNOWN;
    auto it = _glfw_key_map.find(key);
    if (it != _glfw_key_map.end())
        sim_key = it->second;

    // Use GLFW polling for actual key state to work around X11 auto-repeat
    // which sends rapid PRESS/RELEASE pairs when holding a key
    bool actually_pressed = (glfwGetKey(window, key) == GLFW_PRESS);
    SimulationInput::KeyAction sim_action = actually_pressed
        ? SimulationInput::KeyAction::PRESS
        : SimulationInput::KeyAction::RELEASE;

    int modifiers = SimulationInput::ActionModifier::NONE;
    if (mods & GLFW_MOD_SHIFT) modifiers |= SimulationInput::ActionModifier::SHIFT;
    if (mods & GLFW_MOD_CONTROL) modifiers |= SimulationInput::ActionModifier::CTRL;
    if (mods & GLFW_MOD_ALT) modifiers |= SimulationInput::ActionModifier::ALT;

    // Track space key for tool interaction mode
    if (sim_key == SimulationInput::Key::SPACE) {
        viewer->_space_held = actually_pressed;
    }

    viewer->_processKeyboardEvent(sim_key, sim_action, modifiers);
}

void OpenGLViewer::_mouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
{
    OpenGLViewer* viewer = static_cast<OpenGLViewer*>(glfwGetWindowUserPointer(window));
    if (!viewer) return;

    // Track button state for camera interaction
    if (button == GLFW_MOUSE_BUTTON_LEFT) viewer->_left_button_down = (action == GLFW_PRESS);
    if (button == GLFW_MOUSE_BUTTON_RIGHT) viewer->_right_button_down = (action == GLFW_PRESS);
    if (button == GLFW_MOUSE_BUTTON_MIDDLE) viewer->_middle_button_down = (action == GLFW_PRESS);

    // Map to SimulationInput
    SimulationInput::MouseButton sim_button;
    switch (button) {
        case GLFW_MOUSE_BUTTON_LEFT:   sim_button = SimulationInput::MouseButton::LEFT; break;
        case GLFW_MOUSE_BUTTON_RIGHT:  sim_button = SimulationInput::MouseButton::RIGHT; break;
        case GLFW_MOUSE_BUTTON_MIDDLE: sim_button = SimulationInput::MouseButton::MIDDLE; break;
        default: return;
    }

    SimulationInput::MouseAction sim_action = (action == GLFW_PRESS)
        ? SimulationInput::MouseAction::PRESS
        : SimulationInput::MouseAction::RELEASE;

    int modifiers = SimulationInput::ActionModifier::NONE;
    if (mods & GLFW_MOD_SHIFT) modifiers |= SimulationInput::ActionModifier::SHIFT;
    if (mods & GLFW_MOD_CONTROL) modifiers |= SimulationInput::ActionModifier::CTRL;
    if (mods & GLFW_MOD_ALT) modifiers |= SimulationInput::ActionModifier::ALT;

    viewer->_processMouseButtonEvent(sim_button, sim_action, modifiers);

    // Camera interaction (only if mouse interaction is enabled)
    if (!viewer->_enable_mouse_interaction) return;
    // (interaction is handled in cursor_pos callback)
}

void OpenGLViewer::_cursorPosCallback(GLFWwindow* window, double xpos, double ypos)
{
    OpenGLViewer* viewer = static_cast<OpenGLViewer*>(glfwGetWindowUserPointer(window));
    if (!viewer) return;

    double dx = xpos - viewer->_last_mouse_x;
    double dy = ypos - viewer->_last_mouse_y;
    viewer->_last_mouse_x = xpos;
    viewer->_last_mouse_y = ypos;

    // Notify simulation (y is flipped to match Easy3D convention)
    viewer->_processCursorMoveEvent(xpos, viewer->_height - ypos);

    if (!viewer->_enable_mouse_interaction) return;
    if (viewer->_space_held) return;  // Space held = tool mode, skip camera

    // Camera interaction
    if (viewer->_left_button_down) {
        viewer->_handleMouseOrbit(dx, dy);
    }
    if (viewer->_right_button_down) {
        viewer->_handleMousePan(dx, dy);
    }
    if (viewer->_middle_button_down) {
        viewer->_handleMouseZoom(-dy);
    }
}

void OpenGLViewer::_scrollCallback(GLFWwindow* window, double xoffset, double yoffset)
{
    OpenGLViewer* viewer = static_cast<OpenGLViewer*>(glfwGetWindowUserPointer(window));
    if (!viewer) return;

    viewer->_processScrollEvent(xoffset, yoffset);

    if (!viewer->_enable_mouse_interaction) return;
    if (viewer->_space_held) return;  // Space held = tool mode, skip camera
    viewer->_handleMouseZoom(yoffset);
}

void OpenGLViewer::_framebufferSizeCallback(GLFWwindow* window, int width, int height)
{
    OpenGLViewer* viewer = static_cast<OpenGLViewer*>(glfwGetWindowUserPointer(window));
    if (!viewer) return;

    viewer->_width = width;
    viewer->_height = height;
    viewer->_proj_dirty = true;
    glViewport(0, 0, width, height);
}

} // namespace Graphics
