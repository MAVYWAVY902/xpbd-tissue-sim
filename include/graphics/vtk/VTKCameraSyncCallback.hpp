#ifndef __VTK_CAMERA_SYNC_CALLBACK_HPP
#define __VTK_CAMERA_SYNC_CALLBACK_HPP

#include <vtkCommand.h>
#include <vtkCamera.h>

namespace Graphics
{

/** A custom callback that syncs only orientation between cameras, not zoom/position.
 * This allows us to create an orientation widget that doesn't zoom.
 */
class CameraSyncCallback : public vtkCommand
{
public:
    static CameraSyncCallback* New() { return new CameraSyncCallback; }
    
    void Execute(vtkObject* caller, unsigned long eventId, void* callData) override
    {
        vtkCamera* mainCamera = static_cast<vtkCamera*>(caller);
        
        // Copy only the orientation (focal point direction)
        axesCamera->SetPosition(0, 0, 5);  // Fixed distance
        axesCamera->SetFocalPoint(0, 0, 0); // Always look at origin
        
        // Copy the view up vector to maintain orientation
        axesCamera->SetViewUp(mainCamera->GetViewUp());
        
        // Calculate the direction from main camera position to focal point
        double* pos = mainCamera->GetPosition();
        double* focal = mainCamera->GetFocalPoint();
        double direction[3];
        direction[0] = focal[0] - pos[0];
        direction[1] = focal[1] - pos[1];
        direction[2] = focal[2] - pos[2];
        
        // Normalize the direction
        double length = sqrt(direction[0]*direction[0] + 
                           direction[1]*direction[1] + 
                           direction[2]*direction[2]);
        if (length > 0) {
            direction[0] /= length;
            direction[1] /= length;
            direction[2] /= length;
        }
        
        // Set axes camera position based on direction
        double distance = 5.0;  // Fixed distance
        axesCamera->SetPosition(-direction[0] * distance,
                               -direction[1] * distance,
                               -direction[2] * distance);
    }
    
    vtkCamera* axesCamera;
};

} // namespace Graphics

#endif // __VTK_CAMERA_SYNC_CALLBACK_HPP