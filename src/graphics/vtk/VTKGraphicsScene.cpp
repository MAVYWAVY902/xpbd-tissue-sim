#include "graphics/vtk/VTKGraphicsScene.hpp"

#include "simobject/Object.hpp"

#include <vtkOpenGLRenderer.h>
#include <vtkCamera.h>

namespace Graphics
{

VTKGraphicsScene::VTKGraphicsScene(const std::string& name)
    : GraphicsScene(name)
{

}

void VTKGraphicsScene::init()
{
    _viewer = std::make_unique<VTKViewer>(_name);
    _vtk_viewer = dynamic_cast<VTKViewer*>(_viewer.get());
}

void VTKGraphicsScene::update()
{
    // make sure graphics are up to date for all the objects in the scene
    for (auto& obj : _graphics_objects)
    {
        obj->update();
    }

    // update the viewer (signal a redraw)
    _viewer->update();
}

int VTKGraphicsScene::run()
{
    // display the window initially
    _vtk_viewer->displayWindow();

    // then start the interactor 
    _vtk_viewer->interactorStart();

    return 0;
}

int VTKGraphicsScene::addObject(const Sim::Object* obj, const Config::ObjectConfig* obj_config)
{
    // make sure object with name doesn't already exist in the scene
    if (getObject(obj->name()))
    {
        std::cout << "GraphicsObject with name " << obj->name() << " already exists in this GraphicsScene!" << std::endl;
        assert(0);
    }

    /** TODO: create new graphics object depending on type */
}

void VTKGraphicsScene::setCameraOrthographic()
{
    _vtk_viewer->renderer()->GetActiveCamera()->SetParallelProjection(true);
}

void VTKGraphicsScene::setCameraPerspective()
{
    _vtk_viewer->renderer()->GetActiveCamera()->SetParallelProjection(false);
}

void VTKGraphicsScene::setCameraFOV(Real fov)
{
    _vtk_viewer->renderer()->GetActiveCamera()->UseHorizontalViewAngleOn();
    _vtk_viewer->renderer()->GetActiveCamera()->SetViewAngle(fov);
}

Vec3r VTKGraphicsScene::cameraViewDirection() const
{
    double* vec = _vtk_viewer->renderer()->GetActiveCamera()->GetDirectionOfProjection();
    return Vec3r(vec[0], vec[1], vec[2]);
}

void VTKGraphicsScene::setCameraViewDirection(const Vec3r& view_dir)
{
    double dist = _vtk_viewer->renderer()->GetActiveCamera()->GetDistance();
    Vec3r pos = cameraPosition();
    Vec3r new_focal_point = pos + view_dir*dist;
    _vtk_viewer->renderer()->GetActiveCamera()->SetFocalPoint(new_focal_point[0], new_focal_point[1], new_focal_point[2]);
}

Vec3r VTKGraphicsScene::cameraUpDirection() const
{
    double* vec = _vtk_viewer->renderer()->GetActiveCamera()->GetViewUp();
    return Vec3r(vec[0], vec[1], vec[2]);
}

void VTKGraphicsScene::setCameraUpDirection(const Vec3r& up_dir)
{
    _vtk_viewer->renderer()->GetActiveCamera()->SetViewUp(up_dir[0], up_dir[1], up_dir[2]);
}

Vec3r VTKGraphicsScene::cameraRightDirection() const
{
    return cameraViewDirection().cross(cameraUpDirection());
}

Vec3r VTKGraphicsScene::cameraPosition() const
{
    double* vec = _vtk_viewer->renderer()->GetActiveCamera()->GetPosition();
    return Vec3r(vec[0], vec[1], vec[2]);
}

void VTKGraphicsScene::setCameraPosition(const Vec3r& pos)
{
    _vtk_viewer->renderer()->GetActiveCamera()->SetPosition(pos[0], pos[1], pos[2]);
}



} // namespace Graphics