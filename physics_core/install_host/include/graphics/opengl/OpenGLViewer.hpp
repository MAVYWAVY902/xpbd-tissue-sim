#ifndef __OPENGL_VIEWER_HPP
#define __OPENGL_VIEWER_HPP

#include "graphics/Viewer.hpp"
#include "common/SimulationInput.hpp"

#include <string>
#include <map>
#include <optional>
#include <array>
#include <Eigen/Dense>

// Forward declare GLFW types
struct GLFWwindow;

namespace Graphics
{

/**
 * Raw OpenGL + GLFW viewer implementation.
 * Replaces Easy3DTextRenderingViewer with direct OpenGL rendering.
 */
class OpenGLViewer : public Viewer
{
    public:

    explicit OpenGLViewer(const std::string& title, int width = 1024, int height = 768);
    ~OpenGLViewer();

    /** Swaps buffers and polls events. */
    virtual void update() override;

    /** Width/height of the viewer window. */
    virtual int width() const override;
    virtual int height() const override;

    /** Creates the GLFW window and OpenGL context. Must be called before addObject(). */
    void initWindow();

    /** Enters the main render loop. Returns 0 on normal exit. */
    int run();

    /** Sets the background texture ID for rendering a full-screen background image. */
    void setBackgroundTexture(unsigned int id) { _background_texture_id = id; }

    /** Sets UV offsets for the equirectangular background. */
    void setBackgroundOffset(float u_offset, float v_offset) {
        _bg_u_offset = u_offset;
        _bg_v_offset = v_offset;
    }

    /** Set initial camera config to be applied on first draw. */
    void setInitialCameraConfig(
        const std::optional<Eigen::Vector3f>& pos,
        const std::optional<Eigen::Vector3f>& view_dir,
        const std::optional<Eigen::Vector3f>& up_dir,
        const std::optional<float>& fov);

    // ---- Camera accessors ----
    Eigen::Vector3f cameraPosition() const { return _cam_position; }
    void setCameraPosition(const Eigen::Vector3f& pos) { _cam_position = pos; _view_dirty = true; }

    Eigen::Vector3f cameraViewDirection() const { return _cam_view_dir; }
    void setCameraViewDirection(const Eigen::Vector3f& dir) { _cam_view_dir = dir.normalized(); _view_dirty = true; }

    Eigen::Vector3f cameraUpDirection() const { return _cam_up; }
    void setCameraUpDirection(const Eigen::Vector3f& up) { _cam_up = up.normalized(); _view_dirty = true; }

    Eigen::Vector3f cameraRightDirection() const { return _cam_view_dir.cross(_cam_up).normalized(); }

    void setCameraFOV(float fov_degrees) { _fov_degrees = fov_degrees; _proj_dirty = true; }
    float cameraFOV() const { return _fov_degrees; }

    void setCameraOrthographic() { _perspective = false; _proj_dirty = true; }
    void setCameraPerspective() { _perspective = true; _proj_dirty = true; }

    /** Get the current view matrix (4x4, column-major). */
    Eigen::Matrix4f viewMatrix() const;
    /** Get the current projection matrix (4x4, column-major). */
    Eigen::Matrix4f projectionMatrix() const;

    /** Get the GLFW window pointer (for sharing context, etc). */
    GLFWwindow* glfwWindow() const { return _window; }

    /** Set a draw callback that will be called each frame. */
    using DrawCallback = void(*)(void* user_data);
    void setDrawCallback(DrawCallback cb, void* user_data) { _draw_callback = cb; _draw_user_data = user_data; }

    protected:
    // GLFW callbacks (static trampolines)
    static void _keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
    static void _mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
    static void _cursorPosCallback(GLFWwindow* window, double xpos, double ypos);
    static void _scrollCallback(GLFWwindow* window, double xoffset, double yoffset);
    static void _framebufferSizeCallback(GLFWwindow* window, int width, int height);

    private:
    void _initGLFW();
    void _initBackground();
    void _drawBackground() const;
    void _applyPendingCamera();

    /** Arcball-style camera interaction */
    void _handleMouseOrbit(double dx, double dy);
    void _handleMousePan(double dx, double dy);
    void _handleMouseZoom(double dy);

    GLFWwindow* _window = nullptr;
    int _width, _height;

    // Camera state
    Eigen::Vector3f _cam_position{0.0f, -2.0f, 1.0f};
    Eigen::Vector3f _cam_view_dir{0.0f, 1.0f, -0.3f};
    Eigen::Vector3f _cam_up{0.0f, 0.0f, 1.0f};
    float _fov_degrees = 45.0f;
    bool _perspective = true;
    mutable bool _view_dirty = true;
    mutable bool _proj_dirty = true;
    mutable Eigen::Matrix4f _cached_view;
    mutable Eigen::Matrix4f _cached_proj;

    // Mouse interaction state
    bool _left_button_down = false;
    bool _right_button_down = false;
    bool _middle_button_down = false;
    double _last_mouse_x = 0.0;
    double _last_mouse_y = 0.0;

    // Space key held (disables camera interaction, enables tool movement)
    bool _space_held = false;

    // Pending initial camera config
    bool _pending_camera_applied = false;
    std::optional<Eigen::Vector3f> _pending_camera_position;
    std::optional<Eigen::Vector3f> _pending_camera_view_dir;
    std::optional<Eigen::Vector3f> _pending_camera_up_dir;
    std::optional<float> _pending_camera_fov;

    // Background rendering
    unsigned int _background_texture_id = 0;
    float _bg_u_offset = 0.0f;
    float _bg_v_offset = 0.0f;
    mutable unsigned int _bg_shader = 0;
    mutable unsigned int _bg_vao = 0;
    mutable unsigned int _bg_vbo = 0;
    mutable unsigned int _bg_ebo = 0;
    mutable bool _bg_shader_initialized = false;
    mutable bool _initial_cam_pos_set = false;
    float _initial_cam_pos[3] = {0, 0, 0};

    // External draw callback
    DrawCallback _draw_callback = nullptr;
    void* _draw_user_data = nullptr;

    // GLFW key mapping
    static const std::map<int, SimulationInput::Key> _glfw_key_map;
};

} // namespace Graphics

#endif // __OPENGL_VIEWER_HPP
