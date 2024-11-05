#include "graphics/Easy3DTextRenderingViewer.hpp"

namespace Graphics
{

Easy3DTextRenderingViewer::Easy3DTextRenderingViewer(const std::string& title) : easy3d::Viewer(title)
        , _text_renderer(nullptr)
{

}

void Easy3DTextRenderingViewer::addText(const std::string& name,
                                  const std::string& text,
                                  const float& x,
                                  const float& y,
                                  const float& font_size,
                                  const easy3d::TextRenderer::Align& alignment,
                                  const Font& font,
                                  const easy3d::vec3& color,
                                  const float& line_spacing,
                                  const bool& upper_left)
{
    // try to add the TextSpec to the map, and ensure that it was successful
    auto [it,b] = _text_map.try_emplace(name, TextSpec(name, text, x, y, font_size, alignment, font, color, line_spacing, upper_left));
    assert(b);
}

void Easy3DTextRenderingViewer::removeText(const std::string& name)
{
    // erase the TextSpec with the specified name from the map
    _text_map.erase(name);
}

void Easy3DTextRenderingViewer::editText(const std::string& name, const std::string& new_text)
{
    // make sure a TextSpec with the name exists
    assert(_text_map.find(name) != _text_map.end());
    // edit the text of the TextSpec
    _text_map.at(name).text = new_text;
}

void Easy3DTextRenderingViewer::editText( const std::string& name,
                                    const std::string& new_text,
                                    const float& new_x,
                                    const float& new_y,
                                    const float& new_font_size)
{
    // make sure a TextSpec with the name exists
    assert(_text_map.find(name) != _text_map.end());
    // edit the properties of the TextSpec
    _text_map.at(name).text = new_text;
    _text_map.at(name).x = new_x;
    _text_map.at(name).y = new_y;
    _text_map.at(name).font_size = new_font_size;
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
                             t_spec.alignment,
                             t_spec.font,
                             t_spec.color,
                             t_spec.line_spacing,
                             t_spec.upper_left);
    }
}

void Easy3DTextRenderingViewer::draw() const
{
    // call original Viewer draw call
    Viewer::draw();

    // and then draw the text
    drawText();
}

bool Easy3DTextRenderingViewer::callback_event_keyboard(int key, int action, int modifiers)
{
    if (_simulation)
    {
        // notify the simulation that a key was pressed
        _simulation->notifyKeyPressed(key, action, modifiers);
    }

    return Viewer::callback_event_keyboard(key, action, modifiers);
}

bool Easy3DTextRenderingViewer::callback_event_mouse_button(int button, int action, int modifiers)
{
    if (_simulation)
    {
        // notify the simulation that a mouse button was pressed
        _simulation->notifyMouseButtonPressed(button, action, modifiers);
    }

    // if mouse interaction is enabled, pass on the event to easy3d::Viewer
    if (_enable_mouse_interaction)
        return Viewer::callback_event_mouse_button(button, action, modifiers);
    else
        return true;
    
}

bool Easy3DTextRenderingViewer::callback_event_cursor_pos(double x, double y)
{
    if (_simulation)
    {
        // notify the simulation that the mouse was moved
        _simulation->notifyMouseMoved(x, y);
    }

    // if mouse interaction is enabled, pass onthe event to easy3d::Viewer
    if (_enable_mouse_interaction)
        return Viewer::callback_event_cursor_pos(x, y);
    else
        return true;
}

void Easy3DTextRenderingViewer::init()
{
    Viewer::init();

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


} // namespace Graphics

