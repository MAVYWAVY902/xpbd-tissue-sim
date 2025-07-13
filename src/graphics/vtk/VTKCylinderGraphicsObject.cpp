#include "graphics/vtk/VTKCylinderGraphicsObject.hpp"

#include <vtkPolyDataMapper.h>

namespace Graphics
{

VTKCylinderGraphicsObject::VTKCylinderGraphicsObject(const std::string& name, const Sim::RigidCylinder* cyl)
    : CylinderGraphicsObject(name, cyl)
{
    // create the vtkActor from a cylinder source
    _cyl_source = vtkSmartPointer<vtkCylinderSource>::New();
    _cyl_source->SetHeight(cyl->height());
    _cyl_source->SetRadius(cyl->radius());

    vtkNew<vtkPolyDataMapper> data_mapper;
    data_mapper->SetInputConnection(_cyl_source->GetOutputPort());
    
    _cyl_actor = vtkSmartPointer<vtkActor>::New();
    _cyl_actor->SetMapper(data_mapper);

    _vtk_transform = vtkSmartPointer<vtkTransform>::New();

    Mat4r cyl_transform_mat = cyl->transform().asMatrix();
    _vtk_transform->SetMatrix(cyl_transform_mat.data());

    _cyl_actor->SetUserTransform(_vtk_transform);
}

void VTKCylinderGraphicsObject::update()
{
    Mat4r cyl_transform_mat = _cylinder->transform().asMatrix();
    _vtk_transform->SetMatrix(cyl_transform_mat.data());
}

} // namespace Graphics