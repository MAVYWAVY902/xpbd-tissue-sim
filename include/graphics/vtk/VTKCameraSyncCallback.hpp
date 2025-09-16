#ifndef __VTK_CAMERA_SYNC_CALLBACK_HPP
#define __VTK_CAMERA_SYNC_CALLBACK_HPP

#include "common/types.hpp"

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
    
    void Execute(vtkObject* caller, unsigned long /*eventId*/, void* /*callData*/) override
    {
        vtkCamera* mainCamera = static_cast<vtkCamera*>(caller);
        
        // Copy only the orientation (focal point direction)
        axesCamera->SetPosition(0, 0, 5);  // Fixed distance
        axesCamera->SetFocalPoint(0, 0, 0); // Always look at origin
        
        // Copy the view up vector to maintain orientation
        double* up_dir = mainCamera->GetViewUp();
        axesCamera->SetViewUp(up_dir);
        
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

        /** TODO: Need a mutex here? Copying from graphics thread */
        // Set the camera view dir
        (*cam_view_dir)(0) = direction[0];
        (*cam_view_dir)(1) = direction[1];
        (*cam_view_dir)(2) = direction[2];

        // Set the camera up dir
        (*cam_up_dir)(0) = up_dir[0];
        (*cam_up_dir)(1) = up_dir[1];
        (*cam_up_dir)(2) = up_dir[2];

        (*cam_pos)(0) = pos[0];
        (*cam_pos)(1) = pos[1];
        (*cam_pos)(2) = pos[2];
        
        // Set axes camera position based on direction
        double distance = 5.0;  // Fixed distance
        axesCamera->SetPosition(-direction[0] * distance,
                               -direction[1] * distance,
                               -direction[2] * distance);
    }
    
    vtkCamera* axesCamera;

    Vec3r* cam_view_dir;
    Vec3r* cam_up_dir;
    Vec3r* cam_pos;
};

} // namespace Graphics

#endif // __VTK_CAMERA_SYNC_CALLBACK_HPP