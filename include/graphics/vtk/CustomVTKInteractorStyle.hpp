#ifndef __CUSTOM_VTK_INTERACTOR_HPP
#define __CUSTOM_VTK_INTERACTOR_HPP

#include "common/SimulationInput.hpp"

#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObjectFactory.h>

namespace Graphics
{
    class VTKViewer;
}

namespace Graphics
{

class CustomVTKInteractorStyle : public vtkInteractorStyleTrackballCamera
{
    public:
    static CustomVTKInteractorStyle* New();
    vtkTypeMacro(CustomVTKInteractorStyle, vtkInteractorStyleTrackballCamera);

    void registerViewer(Graphics::VTKViewer* viewer);

    // keyboard events
    virtual void OnKeyPress() override;
    virtual void OnKeyRelease() override;

    // mouse events
    virtual void OnMouseMove() override;
    virtual void OnLeftButtonDown() override;
    virtual void OnLeftButtonUp() override;
    virtual void OnRightButtonDown() override;
    virtual void OnRightButtonUp() override;
    virtual void OnMiddleButtonDown() override;
    virtual void OnMiddleButtonUp() override;
    virtual void OnMouseWheelForward() override;
    virtual void OnMouseWheelBackward() override;

    private:
    int _getModifiers();

    private:
    Graphics::VTKViewer* _viewer;

    static const std::map<std::string, SimulationInput::Key> _vtk_key_map;

};

} // namespace Graphics

#endif // __CUSTOM_VTK_INTERACTOR_HPP