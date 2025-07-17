#include "graphics/vtk/VTKSphereGraphicsObject.hpp"

#include <vtkPolyDataMapper.h>
#include <vtkMatrix4x4.h>

namespace Graphics
{

VTKSphereGraphicsObject::VTKSphereGraphicsObject(const std::string& name, const Sim::RigidSphere* sphere)
    : SphereGraphicsObject(name, sphere)
{
    // create the vtkActor from a sphere source
    _sphere_source = vtkSmartPointer<vtkSphereSource>::New();
    _sphere_source->SetRadius(sphere->radius());

    vtkNew<vtkPolyDataMapper> data_mapper;
    data_mapper->SetInputConnection(_sphere_source->GetOutputPort());
    
    _sphere_actor = vtkSmartPointer<vtkActor>::New();
    _sphere_actor->SetMapper(data_mapper);

    _vtk_transform = vtkSmartPointer<vtkTransform>::New();

    // IMPORTANT: use row-major ordering since that is what VTKTransform expects (default for Eigen is col-major)
    Eigen::Matrix<Real, 4, 4, Eigen::RowMajor> sphere_transform_mat = sphere->transform().asMatrix();
    _vtk_transform->SetMatrix(sphere_transform_mat.data());

    _sphere_actor->SetUserTransform(_vtk_transform);
}

void VTKSphereGraphicsObject::update()
{
    // IMPORTANT: use row-major ordering since that is what VTKTransform expects (default for Eigen is col-major)
    Eigen::Matrix<Real, 4, 4, Eigen::RowMajor> sphere_transform_mat = _sphere->transform().asMatrix();
    _vtk_transform->SetMatrix(sphere_transform_mat.data());
}

} // namespace Graphics