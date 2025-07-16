#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkConeSource.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkRenderer.h>
#include <vtkOpenGLRenderer.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkTriangle.h>
#include <vtkPolygon.h>
#include <vtkQuad.h>
#include <vtkPolyDataNormals.h>
#include <vtkLight.h>
#include <vtkImageData.h>
#include <vtkFloatArray.h>
#include <vtkTexture.h>
#include <vtkPointData.h>


int main()
{
    vtkNew<vtkConeSource> cone_source;
    cone_source->SetRadius(1.0);
    cone_source->SetHeight(1.0);

    vtkNew<vtkPolyDataMapper> cone_mapper;
    cone_mapper->SetInputConnection(cone_source->GetOutputPort());

    vtkNew<vtkActor> actor;
    actor->SetMapper(cone_mapper);

    // vtkNew<vtkOpenGLRenderer> renderer;
    auto renderer = vtkSmartPointer<vtkOpenGLRenderer>::New();
    renderer->AddActor(actor);

    vtkNew<vtkRenderWindow> render_window;
    render_window->AddRenderer(renderer);
    render_window->SetSize(600,600);

    vtkNew<vtkRenderWindowInteractor> interactor;
    vtkNew<vtkInteractorStyleTrackballCamera> style;
    interactor->SetInteractorStyle(style);
    interactor->SetRenderWindow(render_window);
    render_window->Render();

    interactor->Start();

    return EXIT_SUCCESS;
}