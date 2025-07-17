#include "graphics/vtk/CustomVTKInteractorStyle.hpp"
#include "graphics/vtk/VTKViewer.hpp"

#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

#include <string>
#include <iostream>

vtkStandardNewMacro(Graphics::CustomVTKInteractorStyle);

namespace Graphics
{

const std::map<std::string, SimulationInput::Key> CustomVTKInteractorStyle::_vtk_key_map =
{
    {"a", SimulationInput::Key::A},
    {"b", SimulationInput::Key::B},
    {"c", SimulationInput::Key::C},
    {"d", SimulationInput::Key::D},
    {"e", SimulationInput::Key::E},
    {"f", SimulationInput::Key::F},
    {"g", SimulationInput::Key::G},
    {"h", SimulationInput::Key::H},
    {"i", SimulationInput::Key::I},
    {"j", SimulationInput::Key::J},
    {"k", SimulationInput::Key::K},
    {"l", SimulationInput::Key::L},
    {"m", SimulationInput::Key::M},
    {"n", SimulationInput::Key::N},
    {"o", SimulationInput::Key::O},
    {"p", SimulationInput::Key::P},
    {"q", SimulationInput::Key::Q},
    {"r", SimulationInput::Key::R},
    {"s", SimulationInput::Key::S},
    {"t", SimulationInput::Key::T},
    {"u", SimulationInput::Key::U},
    {"v", SimulationInput::Key::V},
    {"w", SimulationInput::Key::W},
    {"x", SimulationInput::Key::X},
    {"y", SimulationInput::Key::Y},
    {"z", SimulationInput::Key::Z},
    {"A", SimulationInput::Key::A},
    {"B", SimulationInput::Key::B},
    {"C", SimulationInput::Key::C},
    {"D", SimulationInput::Key::D},
    {"E", SimulationInput::Key::E},
    {"F", SimulationInput::Key::F},
    {"G", SimulationInput::Key::G},
    {"H", SimulationInput::Key::H},
    {"I", SimulationInput::Key::I},
    {"J", SimulationInput::Key::J},
    {"K", SimulationInput::Key::K},
    {"L", SimulationInput::Key::L},
    {"M", SimulationInput::Key::M},
    {"N", SimulationInput::Key::N},
    {"O", SimulationInput::Key::O},
    {"P", SimulationInput::Key::P},
    {"Q", SimulationInput::Key::Q},
    {"R", SimulationInput::Key::R},
    {"S", SimulationInput::Key::S},
    {"T", SimulationInput::Key::T},
    {"U", SimulationInput::Key::U},
    {"V", SimulationInput::Key::V},
    {"W", SimulationInput::Key::W},
    {"X", SimulationInput::Key::X},
    {"Y", SimulationInput::Key::Y},
    {"Z", SimulationInput::Key::Z},
    {"1", SimulationInput::Key::ONE},
    {"2", SimulationInput::Key::TWO},
    {"3", SimulationInput::Key::THREE},
    {"4", SimulationInput::Key::FOUR},
    {"5", SimulationInput::Key::FIVE},
    {"6", SimulationInput::Key::SIX},
    {"7", SimulationInput::Key::SEVEN},
    {"8", SimulationInput::Key::EIGHT},
    {"9", SimulationInput::Key::NINE},
    {"0", SimulationInput::Key::ZERO},
    {"exclam", SimulationInput::Key::ONE},
    {"at", SimulationInput::Key::TWO},
    {"numbersign", SimulationInput::Key::THREE},
    {"dollar", SimulationInput::Key::FOUR},
    {"percent", SimulationInput::Key::FIVE},
    {"asciicircum", SimulationInput::Key::SIX},
    {"ampersand", SimulationInput::Key::SEVEN},
    {"asterisk", SimulationInput::Key::EIGHT},
    {"parenleft", SimulationInput::Key::NINE},
    {"parenright", SimulationInput::Key::ZERO},
    {"F1", SimulationInput::Key::F1},
    {"F2", SimulationInput::Key::F2},
    {"F3", SimulationInput::Key::F3},
    {"F4", SimulationInput::Key::F4},
    {"F5", SimulationInput::Key::F5},
    {"F6", SimulationInput::Key::F6},
    {"F7", SimulationInput::Key::F7},
    {"F8", SimulationInput::Key::F8},
    {"F9", SimulationInput::Key::F9},
    {"Shift_R", SimulationInput::Key::SHIFT},
    {"Shift_L", SimulationInput::Key::SHIFT},
    {"Control_R", SimulationInput::Key::CONTROL},
    {"Control_L", SimulationInput::Key::CONTROL},
    {"Alt_R", SimulationInput::Key::ALT},
    {"Alt_L", SimulationInput::Key::ALT},
    {"space", SimulationInput::Key::SPACE},
    {"BackSpace", SimulationInput::Key::BACKSPACE},
    {"Tab", SimulationInput::Key::TAB},
    {"Escape", SimulationInput::Key::ESC},
    {"Return", SimulationInput::Key::ENTER},
    {"Up", SimulationInput::Key::UP},
    {"Down", SimulationInput::Key::DOWN},
    {"Right", SimulationInput::Key::RIGHT},
    {"Left", SimulationInput::Key::LEFT}
};


void CustomVTKInteractorStyle::registerViewer(Graphics::VTKViewer* viewer)
{
    _viewer = viewer;
}

void CustomVTKInteractorStyle::OnKeyPress()
{
    // Get the keypress.
    vtkRenderWindowInteractor* rwi = this->Interactor;
    std::string key_str = rwi->GetKeySym();

    // convert to SimulationInput::Key
    SimulationInput::Key key = SimulationInput::Key::UNKNOWN;
    auto it = _vtk_key_map.find(key_str);
    if (it != _vtk_key_map.end())
    {
        key = it->second;
    }
    else
    {
        std::cout << "Unknown VTK key str: " << key_str << std::endl;
    }

    if (_viewer)
        _viewer->_processKeyboardEvent(key, SimulationInput::KeyAction::PRESS, _getModifiers());

    // vtkInteractorStyleTrackballCamera::OnKeyPress();
}

void CustomVTKInteractorStyle::OnKeyRelease()
{
    // Get the key release.
    vtkRenderWindowInteractor* rwi = this->Interactor;
    std::string key_str = rwi->GetKeySym();

    // convert to SimulationInput::Key
    SimulationInput::Key key = SimulationInput::Key::UNKNOWN;
    auto it = _vtk_key_map.find(key_str);
    if (it != _vtk_key_map.end())
    {
        key = it->second;
    }
    else
    {
        std::cout << "Unknown VTK key str: " << key_str << std::endl;
    }

    if (_viewer)
        _viewer->_processKeyboardEvent(key, SimulationInput::KeyAction::RELEASE, _getModifiers());

    // vtkInteractorStyleTrackballCamera::OnKeyRelease();
}

void CustomVTKInteractorStyle::OnMouseMove()
{
    vtkRenderWindowInteractor* rwi = this->Interactor;
    int mx, my;
    rwi->GetMousePosition(&mx, &my);

    if (_viewer)
        _viewer->_processCursorMoveEvent(mx, my);
    
    vtkInteractorStyleTrackballCamera::OnMouseMove();

    this->Interactor->SetEventPosition(mx, my, 0);  // this stops the trackball camera interactor from continuing to spin even though the mouse isn't moving
}

void CustomVTKInteractorStyle::OnLeftButtonDown()
{
    if (_viewer)
        _viewer->_processMouseButtonEvent(SimulationInput::MouseButton::LEFT, SimulationInput::MouseAction::PRESS, _getModifiers());

    if (_viewer && _viewer->_enable_mouse_interaction)
        vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
}

void CustomVTKInteractorStyle::OnLeftButtonUp()
{
    if (_viewer)
        _viewer->_processMouseButtonEvent(SimulationInput::MouseButton::LEFT, SimulationInput::MouseAction::RELEASE, _getModifiers());

    if (_viewer && _viewer->_enable_mouse_interaction)
        vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
}

void CustomVTKInteractorStyle::OnRightButtonDown()
{
    if (_viewer)
        _viewer->_processMouseButtonEvent(SimulationInput::MouseButton::RIGHT, SimulationInput::MouseAction::PRESS, _getModifiers());

    if (_viewer && _viewer->_enable_mouse_interaction)
        vtkInteractorStyleTrackballCamera::OnRightButtonDown();
}

void CustomVTKInteractorStyle::OnRightButtonUp()
{
    if (_viewer)
        _viewer->_processMouseButtonEvent(SimulationInput::MouseButton::RIGHT, SimulationInput::MouseAction::RELEASE, _getModifiers());
    
    if (_viewer && _viewer->_enable_mouse_interaction)
        vtkInteractorStyleTrackballCamera::OnRightButtonUp();
}

void CustomVTKInteractorStyle::OnMiddleButtonDown()
{
    if (_viewer)
        _viewer->_processMouseButtonEvent(SimulationInput::MouseButton::MIDDLE, SimulationInput::MouseAction::PRESS, _getModifiers());
    
    if (_viewer && _viewer->_enable_mouse_interaction)
        vtkInteractorStyleTrackballCamera::OnMiddleButtonDown();
}

void CustomVTKInteractorStyle::OnMiddleButtonUp()
{
    if (_viewer)
        _viewer->_processMouseButtonEvent(SimulationInput::MouseButton::MIDDLE, SimulationInput::MouseAction::RELEASE, _getModifiers());
    
    if (_viewer && _viewer->_enable_mouse_interaction)
        vtkInteractorStyleTrackballCamera::OnMiddleButtonUp();
}

void CustomVTKInteractorStyle::OnMouseWheelForward()
{
    if (_viewer)
        _viewer->_processScrollEvent(0, 1);

    if (_viewer && _viewer->_enable_mouse_interaction)
        vtkInteractorStyleTrackballCamera::OnMouseWheelForward();
}

void CustomVTKInteractorStyle::OnMouseWheelBackward()
{
    if (_viewer)
        _viewer->_processScrollEvent(0, -1);
    
    if (_viewer && _viewer->_enable_mouse_interaction)
        vtkInteractorStyleTrackballCamera::OnMouseWheelBackward();
}

int CustomVTKInteractorStyle::_getModifiers()
{
    vtkRenderWindowInteractor* rwi = this->Interactor;
    int modifiers = 0;
    if (rwi->GetShiftKey())     modifiers |= SimulationInput::ActionModifier::SHIFT;
    if (rwi->GetControlKey())   modifiers |= SimulationInput::ActionModifier::CTRL;
    if (rwi->GetAltKey())       modifiers |= SimulationInput::ActionModifier::ALT;
    
    return modifiers;
}


} // namespace Graphics