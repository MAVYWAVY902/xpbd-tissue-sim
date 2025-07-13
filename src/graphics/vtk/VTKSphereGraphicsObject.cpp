#include "graphics/vtk/VTKSphereGraphicsObject.hpp"

#include <vtkPolyDataMapper.h>

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

    Mat4r sphere_transform_mat = sphere->transform().asMatrix();
    _vtk_transform->SetMatrix(sphere_transform_mat.data());

    _sphere_actor->SetUserTransform(_vtk_transform);
}

void VTKSphereGraphicsObject::update()
{
    Mat4r sphere_transform_mat = _sphere->transform().asMatrix();
    _vtk_transform->SetMatrix(sphere_transform_mat.data());
}

} // namespace Graphics