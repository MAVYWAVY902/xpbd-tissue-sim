#include "simulation/PalpationSimulation.hpp"

namespace Sim
{

PalpationSimulation::PalpationSimulation(const Config::PalpationSimulationConfig* config)
    : VirtuosoSimulation(config)
{

}

void PalpationSimulation::setup()
{
    VirtuosoSimulation::setup();

    // find the XPBDMeshObject (which we're assuming to be the tissue)
    // first check 1st-order XPBDMeshObjects
    auto& fo_xpbd_objs = _objects.template get<std::unique_ptr<FirstOrderXPBDMeshObject_Base>>();
    auto& xpbd_objs = _objects.template get<std::unique_ptr<XPBDMeshObject_Base>>();
    if (fo_xpbd_objs.size() > 0)
        _tissue_obj = fo_xpbd_objs.front().get();
    else if (xpbd_objs.size() > 0)
        _tissue_obj = xpbd_objs.front().get();
    else
        assert((xpbd_objs.size() + fo_xpbd_objs.size() > 0) && "There must be at least 1 XPBDMeshObject or FirstOrderXPBDMeshObject in the simulation (there is no tissue object!).");
    
    // we'll assume for now that the tissue object being palpated has a flat bottom that we can fix
    Real min_y = _tissue_obj.mesh()->boundingBox().min[1];
    std::vector<int> bottom_vertices = _tissue_obj.mesh()->getVerticesWithY(min_y);
    for (const auto& v : bottom_vertices)
    {
        _tissue_obj.fixVertex(v);
    }

    // once we've found the tissue object, make sure that each virtuoso arm knows that this is the object that they're manipulating
    // (the VirtuosoArm class handles the grasping logic)
    if (_virtuoso_robot->hasArm1())
        _virtuoso_robot->arm1()->setToolManipulatedObject(_tissue_obj);
    if (_virtuoso_robot->hasArm2())
        _virtuoso_robot->arm2()->setToolManipulatedObject(_tissue_obj);
}


void PalpationSimulation::_updateGraphics()
{
    VirtuosoSimulation::_updateGraphics();
}

void PalpationSimulation::_timeStep()
{
    VirtuosoSimulation::_timeStep();

    std::cout << "Collision force: " << _virtuoso_robot->arm1()->netCollisionForce().transpose() << " N" << std::endl;
    if (_input_device == SimulationInput::Device::HAPTIC)
    {
        
        
        HHD handle = _haptic_device_manager->deviceHandles()[0];
    
        if (_haptic_device_manager->button2Pressed(handle))
        {
            // transform dx from haptic input frame to camera frame
            Mat3r rot_mat;
            
            rot_mat.col(1) = _graphics_scene->cameraUpDirection();
            rot_mat.col(2) = _graphics_scene->cameraViewDirection();
            rot_mat.col(0) = rot_mat.col(1).cross(rot_mat.col(2));
            Vec3r cam_force = rot_mat.transpose() * _virtuoso_robot->arm1()->netCollisionForce();
            Vec3r haptic_force = GeometryUtils::Ry(-M_PI) * cam_force;
            const Vec3r cur_force = _haptic_device_manager->force(handle);
            
            Real frac = 0.3;
            const Vec3r new_force = frac*haptic_force + (1-frac)*cur_force;
            
            _haptic_device_manager->setForce(handle, new_force);
        }
        else
        {
            _haptic_device_manager->setForce(handle, Vec3r::Zero());
            // std::cout << "Clutch released!" << std::endl;
        }
    }
}

} // namespace Sim