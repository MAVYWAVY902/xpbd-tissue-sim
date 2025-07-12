#include "graphics/CustomVTKInteractorStyle.hpp"
#include "graphics/vtk/VTKViewer.hpp"

#include "common/SimulationInput.hpp"

#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

#include <string>
#include <iostream>

vtkStandardNewMacro(Graphics::CustomVTKInteractorStyle);

namespace Graphics
{

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
    int modifiers;
    if (rwi->GetShiftKey())     modifiers |= SimulationInput::ActionModifier::SHIFT;
    if (rwi->GetControlKey())   modifiers |= SimulationInput::ActionModifier::CTRL;
    if (rwi->GetAltKey())       modifiers |= SimulationInput::ActionModifier::ALT;
    
    return modifiers;
}


} // namespace Graphics