#include "graphics/easy3d/Easy3DTextRenderingViewer.hpp"

namespace Graphics
{

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

void Easy3DTextRenderingViewer::draw() const
{
    // call original easy3d Viewer draw call
    easy3d::Viewer::draw();

    // and then draw the text
    drawText();
}

bool Easy3DTextRenderingViewer::callback_event_keyboard(int key, int action, int modifiers)
{
    _processKeyboardEvent(key, action, modifiers);

    return true;
    // return easy3d::Viewer::callback_event_keyboard(key, action, modifiers);
}

bool Easy3DTextRenderingViewer::callback_event_mouse_button(int button, int action, int modifiers)
{
    _processMouseButtonEvent(button, action, modifiers);

    // if mouse interaction is enabled, pass on the event to easy3d::Viewer
    if (_enable_mouse_interaction)
        return easy3d::Viewer::callback_event_mouse_button(button, action, modifiers);
    else
        return true;
    
}

bool Easy3DTextRenderingViewer::callback_event_cursor_pos(double x, double y)
{
    _processCursorMoveEvent(x, y);

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

