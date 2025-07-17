#include "graphics/vtk/VTKBoxGraphicsObject.hpp"

#include <vtkPolyDataMapper.h>
#include <vtkTransform.h>

namespace Graphics
{

VTKBoxGraphicsObject::VTKBoxGraphicsObject(const std::string& name, const Sim::RigidBox* box)
    : BoxGraphicsObject(name, box)
{
    // create the vtkActor from a box source
    Vec3r box_size = box->size();
    Vec3r box_loc = box->position();
    _cube_source = vtkSmartPointer<vtkCubeSource>::New();
    _cube_source->SetXLength(box_size[0]);
    _cube_source->SetYLength(box_size[1]);
    _cube_source->SetZLength(box_size[2]);
    // cube_source->SetCenter(box_loc[0], box_loc[1], box_loc[2]);

    vtkNew<vtkPolyDataMapper> data_mapper;
    data_mapper->SetInputConnection(_cube_source->GetOutputPort());
    
    _box_actor = vtkSmartPointer<vtkActor>::New();
    _box_actor->SetMapper(data_mapper);

    _vtk_transform = vtkSmartPointer<vtkTransform>::New();

    // IMPORTANT: use row-major ordering since that is what VTKTransform expects (default for Eigen is col-major)
    Eigen::Matrix<Real, 4, 4, Eigen::RowMajor> box_transform_mat = _box->transform().asMatrix();
    _vtk_transform->SetMatrix(box_transform_mat.data());

    _box_actor->SetUserTransform(_vtk_transform);
}

void VTKBoxGraphicsObject::update()
{
    // IMPORTANT: use row-major ordering since that is what VTKTransform expects (default for Eigen is col-major)
    Eigen::Matrix<Real, 4, 4, Eigen::RowMajor> box_transform_mat = _box->transform().asMatrix();
    _vtk_transform->SetMatrix(box_transform_mat.data());
}

} // namespace Graphics