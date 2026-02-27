#include "graphics/easy3d/Easy3DTextRenderingViewer.hpp"
#include <easy3d/renderer/opengl.h>
#include <easy3d/renderer/camera.h>

namespace Graphics
{

const std::map<int, SimulationInput::Key> Easy3DTextRenderingViewer::_easy3d_key_map =
{
    {easy3d::Viewer::KEY_UNKNOWN, SimulationInput::Key::UNKNOWN},
    {easy3d::Viewer::KEY_A, SimulationInput::Key::A},
    {easy3d::Viewer::KEY_B, SimulationInput::Key::B},
    {easy3d::Viewer::KEY_C, SimulationInput::Key::C},
    {easy3d::Viewer::KEY_D, SimulationInput::Key::D},
    {easy3d::Viewer::KEY_E, SimulationInput::Key::E},
    {easy3d::Viewer::KEY_F, SimulationInput::Key::F},
    {easy3d::Viewer::KEY_G, SimulationInput::Key::G},
    {easy3d::Viewer::KEY_H, SimulationInput::Key::H},
    {easy3d::Viewer::KEY_I, SimulationInput::Key::I},
    {easy3d::Viewer::KEY_J, SimulationInput::Key::J},
    {easy3d::Viewer::KEY_K, SimulationInput::Key::K},
    {easy3d::Viewer::KEY_L, SimulationInput::Key::L},
    {easy3d::Viewer::KEY_M, SimulationInput::Key::M},
    {easy3d::Viewer::KEY_N, SimulationInput::Key::N},
    {easy3d::Viewer::KEY_O, SimulationInput::Key::O},
    {easy3d::Viewer::KEY_P, SimulationInput::Key::P},
    {easy3d::Viewer::KEY_Q, SimulationInput::Key::Q},
    {easy3d::Viewer::KEY_R, SimulationInput::Key::R},
    {easy3d::Viewer::KEY_S, SimulationInput::Key::S},
    {easy3d::Viewer::KEY_T, SimulationInput::Key::T},
    {easy3d::Viewer::KEY_U, SimulationInput::Key::U},
    {easy3d::Viewer::KEY_V, SimulationInput::Key::V},
    {easy3d::Viewer::KEY_W, SimulationInput::Key::W},
    {easy3d::Viewer::KEY_X, SimulationInput::Key::X},
    {easy3d::Viewer::KEY_Y, SimulationInput::Key::Y},
    {easy3d::Viewer::KEY_Z, SimulationInput::Key::Z},
    {easy3d::Viewer::KEY_1, SimulationInput::Key::ONE},
    {easy3d::Viewer::KEY_2, SimulationInput::Key::TWO},
    {easy3d::Viewer::KEY_3, SimulationInput::Key::THREE},
    {easy3d::Viewer::KEY_4, SimulationInput::Key::FOUR},
    {easy3d::Viewer::KEY_5, SimulationInput::Key::FIVE},
    {easy3d::Viewer::KEY_6, SimulationInput::Key::SIX},
    {easy3d::Viewer::KEY_7, SimulationInput::Key::SEVEN},
    {easy3d::Viewer::KEY_8, SimulationInput::Key::EIGHT},
    {easy3d::Viewer::KEY_9, SimulationInput::Key::NINE},
    {easy3d::Viewer::KEY_0, SimulationInput::Key::ZERO},
    {easy3d::Viewer::KEY_F1, SimulationInput::Key::F1},
    {easy3d::Viewer::KEY_F2, SimulationInput::Key::F2},
    {easy3d::Viewer::KEY_F3, SimulationInput::Key::F3},
    {easy3d::Viewer::KEY_F4, SimulationInput::Key::F4},
    {easy3d::Viewer::KEY_F5, SimulationInput::Key::F5},
    {easy3d::Viewer::KEY_F6, SimulationInput::Key::F6},
    {easy3d::Viewer::KEY_F7, SimulationInput::Key::F7},
    {easy3d::Viewer::KEY_F8, SimulationInput::Key::F8},
    {easy3d::Viewer::KEY_F9, SimulationInput::Key::F9},
    {easy3d::Viewer::KEY_SPACE, SimulationInput::Key::SPACE},
    {easy3d::Viewer::KEY_UP, SimulationInput::Key::UP},
    {easy3d::Viewer::KEY_DOWN, SimulationInput::Key::DOWN},
    {easy3d::Viewer::KEY_RIGHT, SimulationInput::Key::RIGHT},
    {easy3d::Viewer::KEY_LEFT, SimulationInput::Key::LEFT},
    {easy3d::Viewer::KEY_LEFT_BRACKET, SimulationInput::Key::LEFT_BRACKET},
    {easy3d::Viewer::KEY_RIGHT_BRACKET, SimulationInput::Key::RIGHT_BRACKET},
    {easy3d::Viewer::KEY_BACKSLASH, SimulationInput::Key::BACKSLASH},
    {easy3d::Viewer::KEY_SLASH, SimulationInput::Key::SLASH},
    {easy3d::Viewer::KEY_COMMA, SimulationInput::Key::COMMA},
    {easy3d::Viewer::KEY_PERIOD, SimulationInput::Key::PERIOD},
    {easy3d::Viewer::KEY_MINUS, SimulationInput::Key::DASH},
    {easy3d::Viewer::KEY_EQUAL, SimulationInput::Key::EQUALS},
    {easy3d::Viewer::KEY_SEMICOLON, SimulationInput::Key::SEMICOLON},
    {342, SimulationInput::Key::ALT},
    {258, SimulationInput::Key::TAB}
    // TODO: add mappings for keys not explicitly named
};

const std::map<int, SimulationInput::KeyAction> Easy3DTextRenderingViewer::_easy3d_key_action_map =
{
    {0, SimulationInput::KeyAction::RELEASE},   // easy3d uses "0" for key up events
    {1, SimulationInput::KeyAction::PRESS},     // "1" for key press events
    {2, SimulationInput::KeyAction::PRESS}      // "2" for key held events (which we'll just process as key press events)
};

const std::map<int, SimulationInput::MouseButton> Easy3DTextRenderingViewer::_easy3d_mouse_button_map =
{
    {easy3d::Viewer::BUTTON_LEFT, SimulationInput::MouseButton::LEFT},
    {easy3d::Viewer::BUTTON_RIGHT, SimulationInput::MouseButton::RIGHT},
    {easy3d::Viewer::BUTTON_MIDDLE, SimulationInput::MouseButton::MIDDLE}
};

const std::map<int, SimulationInput::MouseAction> Easy3DTextRenderingViewer::_easy3d_mouse_action_map =
{
    {0, SimulationInput::MouseAction::PRESS},   // "0" = mouse press
    {1, SimulationInput::MouseAction::RELEASE}  // "1" = mouse release
};


Easy3DTextRenderingViewer::Easy3DTextRenderingViewer(const std::string& title) 
    : easy3d::Viewer(title),
      Graphics::Viewer(title),
      _text_renderer(nullptr)
{
}

void Easy3DTextRenderingViewer::update()
{
    easy3d::Viewer::update();
}

void Easy3DTextRenderingViewer::drawText() const
{
    // iterate through each TextSpec
    for (auto const& [name, t_spec] : _text_map)
    {
        // draw the TextSpec with the TextRenderer
        _text_renderer->draw(t_spec.text,
                             t_spec.x * dpi_scaling(),
                             t_spec.y * dpi_scaling(),
                             t_spec.font_size,
                             _getEasy3dAlignment(t_spec.alignment),
                             _getEasy3dFontIndex(t_spec.font),
                             _getEasy3dColor(t_spec.color),
                             t_spec.line_spacing,
                             t_spec.upper_left);
    }
}

// Equirectangular environment map shaders (inline GLSL)
static const char* s_bg_vert_src = R"(
#version 150
in vec2 vtx_position;
out vec2 vNDC;
void main() {
    vNDC = vtx_position;
    gl_Position = vec4(vtx_position, 0.999, 1.0);
}
)";

static const char* s_bg_frag_src = R"(
#version 150
in vec2 vNDC;
out vec4 fragOutput;
uniform sampler2D uTexture;
uniform mat4 uInverseVPRot;

const float PI = 3.14159265359;

void main() {
    // Unproject NDC to world-space direction (rotation only, no translation)
    vec4 worldDir = uInverseVPRot * vec4(vNDC, -1.0, 1.0);
    vec3 dir = normalize(worldDir.xyz / worldDir.w);

    // Equirectangular mapping: direction -> UV
    float u = atan(dir.z, dir.x) / (2.0 * PI) + 0.5;
    float v = asin(clamp(dir.y, -1.0, 1.0)) / PI + 0.5;

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
        std::cerr << "[Easy3D Background] Shader compile error: " << log << std::endl;
    }
    return shader;
}

void Easy3DTextRenderingViewer::_initBackgroundShader() const
{
    // Compile and link the equirectangular shader program
    GLuint vs = compileShader(GL_VERTEX_SHADER, s_bg_vert_src);
    GLuint fs = compileShader(GL_FRAGMENT_SHADER, s_bg_frag_src);
    _bg_shader = glCreateProgram();
    glAttachShader(_bg_shader, vs);
    glAttachShader(_bg_shader, fs);
    // Bind attribute location before linking
    glBindAttribLocation(_bg_shader, 0, "vtx_position");
    glLinkProgram(_bg_shader);
    GLint ok = 0;
    glGetProgramiv(_bg_shader, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetProgramInfoLog(_bg_shader, 512, nullptr, log);
        std::cerr << "[Easy3D Background] Shader link error: " << log << std::endl;
    }
    glDeleteShader(vs);
    glDeleteShader(fs);

    // Create full-screen quad VAO
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
    std::cout << "[Easy3D] Equirectangular background shader initialized." << std::endl;
}

void Easy3DTextRenderingViewer::_drawBackground() const
{
    if (!_bg_shader_initialized)
        _initBackgroundShader();

    // Get camera matrices
    const easy3d::mat4& mv = camera()->modelViewMatrix();
    const easy3d::mat4& proj = camera()->projectionMatrix();

    // Strip translation from modelView matrix (keep only rotation)
    // The view matrix is [R | t; 0 0 0 1], we zero out the translation column
    easy3d::mat4 viewRot = mv;
    viewRot(0, 3) = 0.0f;
    viewRot(1, 3) = 0.0f;
    viewRot(2, 3) = 0.0f;

    // Compute inverse(Projection * ViewRotation)
    easy3d::mat4 vpRot = proj * viewRot;
    easy3d::mat4 invVPRot = easy3d::inverse(vpRot);

    // Draw with depth test disabled (background behind everything)
    glDepthMask(GL_FALSE);

    glUseProgram(_bg_shader);

    // Set uniforms
    GLint loc_tex = glGetUniformLocation(_bg_shader, "uTexture");
    GLint loc_mat = glGetUniformLocation(_bg_shader, "uInverseVPRot");
    glUniform1i(loc_tex, 0);
    // Easy3D mat4 is column-major, same as OpenGL expects
    glUniformMatrix4fv(loc_mat, 1, GL_FALSE, &invVPRot(0, 0));

    // Bind texture
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, _background_texture_id);

    // Draw full-screen quad
    glBindVertexArray(_bg_vao);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);

    glUseProgram(0);
    glDepthMask(GL_TRUE);
}

void Easy3DTextRenderingViewer::draw() const
{
    // draw equirectangular environment map background
    if (_background_texture_id != 0)
    {
        _drawBackground();
    }

    // call original easy3d Viewer draw call
    easy3d::Viewer::draw();

    // and then draw the text
    drawText();
}

bool Easy3DTextRenderingViewer::callback_event_keyboard(int e3d_key, int e3d_action, int e3d_modifiers)
{
    // map easy3d key to SimulationInput::Key
    SimulationInput::Key key = SimulationInput::Key::UNKNOWN;
    auto it = Easy3DTextRenderingViewer::_easy3d_key_map.find(e3d_key);
    if (it != Easy3DTextRenderingViewer::_easy3d_key_map.end())
        key = it->second;

    // maps easy3d action to SimulationInput::KeyAction
    SimulationInput::KeyAction action = Easy3DTextRenderingViewer::_easy3d_key_action_map.at(e3d_action);
    
    // deconstruct modifiers and map them to SimulationInput::ActionModifier
    bool shift_modifier = (e3d_modifiers & easy3d::Viewer::MODIF_SHIFT) == easy3d::Viewer::MODIF_SHIFT;
    bool ctrl_modifier = (e3d_modifiers & easy3d::Viewer::MODIF_CTRL) == easy3d::Viewer::MODIF_CTRL;
    bool alt_modifier = (e3d_modifiers & easy3d::Viewer::MODIF_ALT) == easy3d::Viewer::MODIF_ALT;
    
    int modifiers = SimulationInput::ActionModifier::NONE;
    if (shift_modifier) modifiers |= SimulationInput::ActionModifier::SHIFT;
    if (ctrl_modifier) modifiers |= SimulationInput::ActionModifier::CTRL;
    if (alt_modifier) modifiers |= SimulationInput::ActionModifier::ALT; 

    _processKeyboardEvent(key, action, modifiers);

    return true;
    // return easy3d::Viewer::callback_event_keyboard(key, action, modifiers);
}

bool Easy3DTextRenderingViewer::callback_event_mouse_button(int e3d_button, int e3d_action, int e3d_modifiers)
{
    // maps easy3d mouse button to SimulationInput::MouseButton
    SimulationInput::MouseButton button = Easy3DTextRenderingViewer::_easy3d_mouse_button_map.at(e3d_button);

    // maps easy3d action to SimulationInput::MouseAction
    SimulationInput::MouseAction action = Easy3DTextRenderingViewer::_easy3d_mouse_action_map.at(e3d_action);

    // deconstruct modifiers and map them to SimulationInput::ActionModifier
    bool shift_modifier = (e3d_modifiers & easy3d::Viewer::MODIF_SHIFT) == easy3d::Viewer::MODIF_SHIFT;
    bool ctrl_modifier = (e3d_modifiers & easy3d::Viewer::MODIF_CTRL) == easy3d::Viewer::MODIF_CTRL;
    bool alt_modifier = (e3d_modifiers & easy3d::Viewer::MODIF_ALT) == easy3d::Viewer::MODIF_ALT;
    
    int modifiers = SimulationInput::ActionModifier::NONE;
    if (shift_modifier) modifiers |= SimulationInput::ActionModifier::SHIFT;
    if (ctrl_modifier) modifiers |= SimulationInput::ActionModifier::CTRL;
    if (alt_modifier) modifiers |= SimulationInput::ActionModifier::ALT; 

    _processMouseButtonEvent(button, action, modifiers);

    // if mouse interaction is enabled, pass on the event to easy3d::Viewer
    if (_enable_mouse_interaction)
        return easy3d::Viewer::callback_event_mouse_button(e3d_button, e3d_action, e3d_modifiers);
    else
        return true;
    
}

bool Easy3DTextRenderingViewer::callback_event_cursor_pos(double x, double y)
{
    _processCursorMoveEvent(x, height()-y);

    // if mouse interaction is enabled, pass onthe event to easy3d::Viewer
    if (_enable_mouse_interaction)
        return easy3d::Viewer::callback_event_cursor_pos(x, y);
    else
        return true;
}

bool Easy3DTextRenderingViewer::callback_event_scroll(double dx, double dy)
{
    _processScrollEvent(dx, dy);

    if (_enable_mouse_interaction)
        return easy3d::Viewer::callback_event_scroll(dx, dy);
    else
        return true;
}

void Easy3DTextRenderingViewer::init()
{
    easy3d::Viewer::init();

    // create the TextRenderer
    _text_renderer = std::make_unique<easy3d::TextRenderer>(dpi_scaling());
    // and add fonts to it
    _text_renderer->add_font(easy3d::resource::directory() + "/fonts/cn_Mao.ttf");
    _text_renderer->add_font(easy3d::resource::directory() + "/fonts/en_Cousine-Regular.ttf");
    _text_renderer->add_font(easy3d::resource::directory() + "/fonts/en_Earth-Normal.ttf");
    _text_renderer->add_font(easy3d::resource::directory() + "/fonts/en_G-Unit.TTF");
    _text_renderer->add_font(easy3d::resource::directory() + "/fonts/en_Roboto-Bold.ttf");
    _text_renderer->add_font(easy3d::resource::directory() + "/fonts/en_Roboto-Regular.ttf");
    _text_renderer->add_font(easy3d::resource::directory() + "/fonts/en_Vera.ttf");
}

easy3d::TextRenderer::Align Easy3DTextRenderingViewer::_getEasy3dAlignment(const TextAlignment& alignment) const
{
    switch(alignment)
    {
        case TextAlignment::LEFT:
            return easy3d::TextRenderer::Align::ALIGN_LEFT;
        case TextAlignment::RIGHT:
            return easy3d::TextRenderer::Align::ALIGN_RIGHT;
        case TextAlignment::CENTER:
            return easy3d::TextRenderer::Align::ALIGN_CENTER;
        default:
            std::cout << "Easy3D does not have requested text alignment!" << std::endl;
            return easy3d::TextRenderer::Align::ALIGN_LEFT;
    }
}

int Easy3DTextRenderingViewer::_getEasy3dFontIndex(const Font& font) const
{
    // hard code these for now, depends on order that they are added to the Easy3D text renderer
    switch(font)
    {
        case Font::MAO:
            return 0;
        case Font::COUSINE:
            return 1;
        case Font::EARTH:
            return 2;
        case Font::GUNIT:
            return 3;
        case Font::ROBOTO_BOLD:
            return 4;
        case Font::ROBOTO_REGULAR:
            return 5;
        case Font::VERA:
            return 6;
        default:
            std::cout << "Easy3D does not have requested font!" << std::endl;
            return Font::MAO;
    }

}

easy3d::vec3 Easy3DTextRenderingViewer::_getEasy3dColor(const std::array<float,3>& color) const
{
    return easy3d::vec3(color[0], color[1], color[2]);
}


} // namespace Graphics

