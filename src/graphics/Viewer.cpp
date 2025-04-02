#include "graphics/Viewer.hpp"
#include "simulation/Simulation.hpp"

namespace Graphics
{

Viewer::Viewer(const std::string& name)
    : _name(name), _text_map(), _simulation(0), _enable_mouse_interaction(true)
{

}

void Viewer::addText(   const std::string& name,
                        const std::string& text,
                        const float& x,
                        const float& y,
                        const float& font_size,
                        const TextAlignment& alignment,
                        const Font& font,
                        const std::array<float,3>& color,
                        const float& line_spacing,
                        const bool& upper_left )
{
    // try to add the TextSpec to the map, and ensure that it was successful - it is unsuccessful if a TextSpec with the same name already exists in the map
    auto [it,b] = _text_map.try_emplace(name, TextSpec(name, text, x, y, font_size, alignment, font, color, line_spacing, upper_left));
    assert(b);
}

void Viewer::removeText(const std::string& name)
{
    // erase the TextSpec with the specified name from the map
    _text_map.erase(name);
}

void Viewer::editText(const std::string& name, const std::string& new_text)
{
    // make sure a TextSpec with the name exists
    assert(_text_map.find(name) != _text_map.end());
    // edit the text of the TextSpec
    _text_map.at(name).text = new_text;
}

void Viewer::editText(  const std::string& name,
                        const std::string& new_text,
                        const float& new_x,
                        const float& new_y,
                        const float& new_font_size )
{
    // make sure a TextSpec with the name exists
    assert(_text_map.find(name) != _text_map.end());
    // edit the properties of the TextSpec
    _text_map.at(name).text = new_text;
    _text_map.at(name).x = new_x;
    _text_map.at(name).y = new_y;
    _text_map.at(name).font_size = new_font_size;
}

void Viewer::_processKeyboardEvent(int key, int action, int modifiers)
{
    if (_simulation)
    {
        // notify the simulation that a key was pressed
        _simulation->notifyKeyPressed(key, action, modifiers);
    }
}

void Viewer::_processMouseButtonEvent(int button, int action, int modifiers)
{
    if (_simulation)
    {
        // notify the simulation that a mouse button was pressed
        _simulation->notifyMouseButtonPressed(button, action, modifiers);
    }
}

void Viewer::_processCursorMoveEvent(double x, double y)
{
    if (_simulation)
    {
        // notify the simulation that the mouse was moved
        _simulation->notifyMouseMoved(x, y);
    }
}

void Viewer::_processScrollEvent(double dx, double dy)
{
    if (_simulation)
    {
        // notify the simulation that the mouse wheel was scrolled
        _simulation->notifyMouseScrolled(dx, dy);
    }
}

} // namespace Graphics