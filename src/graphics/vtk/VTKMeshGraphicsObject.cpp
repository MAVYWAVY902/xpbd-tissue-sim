#include "graphics/vtk/VTKMeshGraphicsObject.hpp"

#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkPointData.h>

#include <vtkTriangle.h>
#include <vtkPolygon.h>
#include <vtkQuad.h>
#include <vtkCellArray.h>
#include <vtkFloatArray.h>
#include <vtkProperty.h>

#include <vtkTexture.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataTangents.h>
#include <vtkPolyDataMapper.h>
#include <vtkPNGReader.h>
#include <vtkCleanPolyData.h>
#include <vtkImageData.h>

#include <vtkNew.h>

namespace Graphics
{

VTKMeshGraphicsObject::VTKMeshGraphicsObject(const std::string& name, const Geometry::Mesh* mesh)
    : MeshGraphicsObject(name, mesh)
{
    _vtk_poly_data = vtkSmartPointer<vtkPolyData>::New();

    // create points
    vtkNew<vtkPoints> vtk_points;
    for (int vi = 0; vi < _mesh->numVertices(); vi++)
    {
        const Vec3r& v = _mesh->vertex(vi);
        vtk_points->InsertNextPoint(v[0], v[1], v[2]);
    }

    // create faces
    vtkNew<vtkCellArray> vtk_faces;
    for (int fi = 0; fi < _mesh->numFaces(); fi++)
    {
        const Vec3i& f = _mesh->face(fi);
        vtkNew<vtkTriangle> tri;
        tri->GetPointIds()->SetId(0, f[0]);
        tri->GetPointIds()->SetId(1, f[1]);
        tri->GetPointIds()->SetId(2, f[2]);

        vtk_faces->InsertNextCell(tri);
    }

    _vtk_poly_data->SetPoints(vtk_points);
    _vtk_poly_data->SetPolys(vtk_faces);

    // smooth normals
    vtkNew<vtkPolyDataNormals> normal_generator;
    normal_generator->SetInputData(_vtk_poly_data);
    normal_generator->SetFeatureAngle(30.0);
    normal_generator->SplittingOff();
    normal_generator->ConsistencyOn();
    normal_generator->ComputePointNormalsOn();
    normal_generator->ComputeCellNormalsOff();
    normal_generator->Update();

    // vtkNew<vtkPolyDataTangents> tangents;
    // tangents->SetInputConnection(normal_generator->GetOutputPort());
    // tangents->Update();

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(normal_generator->GetOutputPort());
    
    _vtk_actor = vtkSmartPointer<vtkActor>::New();
    _vtk_actor->SetMapper(mapper);

    /** TODO: add more rendering options (color, textures, normals, etc.) */
    
    
}

void VTKMeshGraphicsObject::update() 
{
    vtkPoints* points = _vtk_poly_data->GetPoints();
    for (int vi = 0; vi < _mesh->numVertices(); vi++)
    {
        const Vec3r& v = _mesh->vertex(vi);
        points->SetPoint(vi, v.data());
    }
    points->Modified();
}

} // namespace Graphics

