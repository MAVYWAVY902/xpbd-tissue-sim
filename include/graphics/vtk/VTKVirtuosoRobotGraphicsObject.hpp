#ifndef __VTK_VIRTUOSO_ROBOT_GRAPHICS_OBJECT_HPP
#define __VTK_VIRTUOSO_ROBOT_GRAPHICS_OBJECT_HPP

#include "graphics/VirtuosoRobotGraphicsObject.hpp"

#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkActor.h>

namespace Graphics
{

class VTKVirtuosoRobotGraphicsObject : public VirtuosoRobotGraphicsObject
{
    public:
    explicit VTKVirtuosoRobotGraphicsObject(const std::string& name, const Sim::VirtuosoRobot* robot);

    virtual void update() override;

    vtkSmartPointer<vtkActor> actor() { return _vtk_actor; }

    private:
    vtkSmartPointer<vtkActor> _vtk_actor;           // VTK actor for the robot
    vtkSmartPointer<vtkTransform> _vtk_transform;   // transform for the actor (updated according to position and orientation of robot)
};

} // namespace Graphics

#endif // __VTK_VIRTUOSO_ROBOT_GRAPHICS_OBJECT_HPP