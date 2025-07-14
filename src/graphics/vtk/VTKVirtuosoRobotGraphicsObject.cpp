#include "graphics/vtk/VTKVirtuosoRobotGraphicsObject.hpp"

#include <vtkCylinderSource.h>
#include <vtkPolyDataMapper.h>

namespace Graphics
{

VTKVirtuosoRobotGraphicsObject::VTKVirtuosoRobotGraphicsObject(const std::string& name, const Sim::VirtuosoRobot* robot)
    : VirtuosoRobotGraphicsObject(name, robot)
{
    // create the vtkActor from a cylinder source
    vtkNew<vtkCylinderSource> cyl_source;;
    cyl_source->SetHeight(robot->endoscopeLength());
    cyl_source->SetRadius(0.5*robot->endoscopeDiameter());

    vtkNew<vtkPolyDataMapper> data_mapper;
    data_mapper->SetInputConnection(cyl_source->GetOutputPort());
    
    _vtk_actor = vtkSmartPointer<vtkActor>::New();
    _vtk_actor->SetMapper(data_mapper);

    _vtk_transform = vtkSmartPointer<vtkTransform>::New();

    Geometry::TransformationMatrix endoscope_transform = _virtuoso_robot->endoscopeFrame().transform();
    const Vec3r cyl_frame_pos = endoscope_transform.translation() - endoscope_transform.rotMat().col(2) * _virtuoso_robot->endoscopeLength();

    Geometry::TransformationMatrix cyl_transform(endoscope_transform.rotMat(), cyl_frame_pos);
    Mat4r transform_mat = cyl_transform.asMatrix();
    _vtk_transform->SetMatrix(transform_mat.data());

    _vtk_actor->SetUserTransform(_vtk_transform);
}

void VTKVirtuosoRobotGraphicsObject::update()
{
    Geometry::TransformationMatrix endoscope_transform = _virtuoso_robot->endoscopeFrame().transform();
    const Vec3r cyl_frame_pos = endoscope_transform.translation() - endoscope_transform.rotMat().col(2) * _virtuoso_robot->endoscopeLength();

    Geometry::TransformationMatrix cyl_transform(endoscope_transform.rotMat(), cyl_frame_pos);
    Mat4r transform_mat = cyl_transform.asMatrix();
    _vtk_transform->SetMatrix(transform_mat.data());
}

} // namespace Graphics