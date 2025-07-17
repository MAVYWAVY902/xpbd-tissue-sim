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
    const Vec3r cyl_frame_pos = endoscope_transform.translation() - endoscope_transform.rotMat().col(2) * _virtuoso_robot->endoscopeLength()/2.0;

    Geometry::TransformationMatrix cyl_transform(endoscope_transform.rotMat(), cyl_frame_pos);
     // IMPORTANT: use row-major ordering since that is what VTKTransform expects (default for Eigen is col-major)
    Eigen::Matrix<Real, 4, 4, Eigen::RowMajor> transform_mat = cyl_transform.asMatrix();
    _vtk_transform->SetMatrix(transform_mat.data());

    // vtkCylinderSource creates a cylinder along the y-axis, but we expect the endoscope to be along the z-axis
    // hence we need to first rotate the cylinder provided by vtkCylinderSource by -90 deg about the x-axis
    _vtk_transform->PreMultiply();
    _vtk_transform->RotateX(-90);

    _vtk_actor->SetUserTransform(_vtk_transform);
}

void VTKVirtuosoRobotGraphicsObject::update()
{
    Geometry::TransformationMatrix endoscope_transform = _virtuoso_robot->endoscopeFrame().transform();
    const Vec3r cyl_frame_pos = endoscope_transform.translation() - endoscope_transform.rotMat().col(2) * _virtuoso_robot->endoscopeLength()/2.0;

    Geometry::TransformationMatrix cyl_transform(endoscope_transform.rotMat(), cyl_frame_pos);
     // IMPORTANT: use row-major ordering since that is what VTKTransform expects (default for Eigen is col-major)
    Eigen::Matrix<Real, 4, 4, Eigen::RowMajor> transform_mat = cyl_transform.asMatrix();
    _vtk_transform->SetMatrix(transform_mat.data());

    // vtkCylinderSource creates a cylinder along the y-axis, but we expect the endoscope to be along the z-axis
    // hence we need to first rotate the cylinder provided by vtkCylinderSource by -90 deg about the x-axis
    _vtk_transform->PreMultiply();
    _vtk_transform->RotateX(-90);
}

} // namespace Graphics