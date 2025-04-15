#include "simulation/VirtuosoSimulation.hpp"

#include "utils/GeometryUtils.hpp"

#include <filesystem>
#include <stdlib.h>

namespace Sim
{

VirtuosoSimulation::VirtuosoSimulation(const VirtuosoSimulationConfig* config)
    : Simulation(config), _virtuoso_robot(nullptr), _active_arm(nullptr)
{
    _input_device = config->inputDevice();

    if (_input_device == SimulationInputDevice::HAPTIC)
    {
        std::cout << BOLD << "Initializing haptic device..." << RST << std::endl;
        _haptic_device_manager = std::make_unique<HapticDeviceManager>();
        
        _last_haptic_pos = _haptic_device_manager->position(_haptic_device_manager->deviceHandles()[0]);
    }
    if (_input_device == SimulationInputDevice::MOUSE)
    {
        _graphics_scene->viewer()->enableMouseInteraction(false);
    }

    // initialize the keys map with the relevant keycodes
    int key_codes[] = {
        32, // space bar
        81, // Q
        87, // W
        69, // E
        82, // R
        65, // A
        83, // S
        68, // D
        70  // F
    };

    size_t num_key_codes = sizeof(key_codes) / sizeof(key_codes[0]);
    for (unsigned i = 0; i < num_key_codes; i++)
        _keys_held[key_codes[i]] = 0;
}

// VirtuosoSimulation::VirtuosoSimulation()
//     : Simulation()
// {
// }

void VirtuosoSimulation::setup()
{
    Simulation::setup();
    
    // find the VirtuosoRobot object
    for (auto& obj : _objects)
    {
        if (VirtuosoRobot* robot = dynamic_cast<VirtuosoRobot*>(obj.get()))
        {
            _virtuoso_robot = robot;
            break;
        }
    }

    assert(_virtuoso_robot);

    // set the active arm to be arm1
    _active_arm = _virtuoso_robot->arm1();
    assert(_active_arm);

    
    // create an object at the tip of the robot to show where grasping is
    RigidSphereConfig cursor_config("tip_cursor", Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0),
        1.0, 0.002, false, true, false);
    _tip_cursor = dynamic_cast<RigidSphere*>(_addObjectFromConfig(&cursor_config));
    assert(_tip_cursor);
    std::cout << _tip_cursor->position() << std::endl;
    _tip_cursor->setPosition(_active_arm->tipPosition());
    std::cout << _tip_cursor->position() << std::endl;
}

void VirtuosoSimulation::notifyMouseButtonPressed(int button, int action, int modifiers)
{

    // button = 0 ==> left mouse button
    // button = 1 ==> right mouse button
    // action = 0 ==> mouse up
    // action = 1 ==> mouse down

    Simulation::notifyMouseButtonPressed(button, action, modifiers);

}

void VirtuosoSimulation::notifyMouseMoved(double x, double y)
{
    if (_input_device == SimulationInputDevice::MOUSE)
    {
        if (_keys_held[32] > 0) // space bar = clutch
        {
            const double scaling = 0.00005;
            double dx = x - _last_mouse_pos[0];
            double dy = y - _last_mouse_pos[1];

            // camera plane defined by camera up direction and camera right direction
            // changes in mouse y position = changes along camera up direction
            // changes in mouse x position = changes along camera right direction
            const Vec3r up_vec = _graphics_scene->cameraUpDirection();
            const Vec3r right_vec = _graphics_scene->cameraRightDirection();
            
            const Vec3r current_tip_position = _tip_cursor->position();
            const Vec3r offset = right_vec*dx + up_vec*-dy; // negate dy since increasing dy is actually opposite of camera frame up vec
            _moveCursor(offset*scaling);
        }
    }

    _last_mouse_pos[0] = x;
    _last_mouse_pos[1] = y;
}

void VirtuosoSimulation::notifyKeyPressed(int key, int action, int modifiers)
{

    // find key in map
    auto it = _keys_held.find(key);
    if (it != _keys_held.end())
    {
        it->second = (action > 0); // if action > 0, key is pressed or held
    }

    // when 'ALT' is pressed, switch which arm is active
    if (key == 342 && action == 1)
    {
        if (_virtuoso_robot->hasArm2() && _active_arm == _virtuoso_robot->arm1())
        {
            _active_arm = _virtuoso_robot->arm2();
        }
        else if (_virtuoso_robot->hasArm1() && _active_arm == _virtuoso_robot->arm2())
        {
            _active_arm = _virtuoso_robot->arm1();
        }

        _tip_cursor->setPosition(_active_arm->tipPosition());
    }
    // when 'TAB' is pressed, switch the camera view to the endoscope view
    else if (key == 258 && action == 1)
    {
        if (_graphics_scene)
        {
            const Geometry::TransformationMatrix& cam_transform = _virtuoso_robot->camFrame().transform();
            _graphics_scene->setCameraPosition(cam_transform.translation());

            // find view dir
            const Vec3r& z_axis_pt = cam_transform.rotMat() * Vec3r(0,0,1) + cam_transform.translation();
            const Vec3r& y_axis_pt = cam_transform.rotMat() * Vec3r(0,1,0) + cam_transform.translation();
            _graphics_scene->setCameraViewDirection(z_axis_pt - cam_transform.translation());
            _graphics_scene->setCameraUpDirection(y_axis_pt - cam_transform.translation());
            _graphics_scene->setCameraFOV(80.0 * M_PI / 180.0);
        }
    }

    Simulation::notifyKeyPressed(key, action, modifiers);

}

void VirtuosoSimulation::notifyMouseScrolled(double dx, double dy)
{
    if (_input_device == SimulationInputDevice::MOUSE)
    {
        if (_keys_held[32] > 0) // space bar = clutch
        {
            const double scaling = 0.0005;
            const Vec3r view_dir = _graphics_scene->cameraViewDirection();

            const Vec3r current_tip_position = _tip_cursor->position();
            const Vec3r offset = view_dir*dy;
            _moveCursor(offset*scaling);
        }
    }
    
}

void VirtuosoSimulation::_moveCursor(const Vec3r& dp)
{
    const Vec3r current_tip_position = _tip_cursor->position();
    _tip_cursor->setPosition(current_tip_position + dp);
    _active_arm->setTipPosition(_tip_cursor->position());
}

void VirtuosoSimulation::_updateGraphics()
{

    Simulation::_updateGraphics();
}

void VirtuosoSimulation::_timeStep()
{
    if (_input_device == SimulationInputDevice::KEYBOARD)
    {
        if (_keys_held[81] > 0) // Q = CCW inner tube rotation
        {
            const double cur_rot = _active_arm->innerTubeRotation();
            _active_arm->setInnerTubeRotation(cur_rot + IT_ROT_RATE*dt());

        }
        if (_keys_held[87] > 0) // W = CCW outer tube rotation
        {
            const double cur_rot = _active_arm->outerTubeRotation();
            _active_arm->setOuterTubeRotation(cur_rot + OT_ROT_RATE*dt());
        }
        if (_keys_held[69] > 0) // E = inner tube extension
        {
            const double cur_trans = _active_arm->innerTubeTranslation();
            _active_arm->setInnerTubeTranslation(cur_trans + IT_TRANS_RATE*dt());
        }
        if (_keys_held[82] > 0) // R = outer tube extension
        {
            const double cur_trans = _active_arm->outerTubeTranslation();
            _active_arm->setOuterTubeTranslation(cur_trans + OT_TRANS_RATE*dt());
        }
        if (_keys_held[65] > 0) // A = CW inner tube rotation
        {
            const double cur_rot = _active_arm->innerTubeRotation();
            _active_arm->setInnerTubeRotation(cur_rot - IT_ROT_RATE*dt()); 
        }
        if (_keys_held[83] > 0) // S = CW outer tube rotation
        {
            const double cur_rot = _active_arm->outerTubeRotation();
            _active_arm->setOuterTubeRotation(cur_rot - OT_ROT_RATE*dt());
        }
        if (_keys_held[68] > 0) // D = inner tube retraction
        {
            const double cur_trans = _active_arm->innerTubeTranslation();
            _active_arm->setInnerTubeTranslation(cur_trans - IT_TRANS_RATE*dt());
        }
        if (_keys_held[70] > 0) // F = outer tube retraction
        {
            const double cur_trans = _active_arm->outerTubeTranslation();
            _active_arm->setOuterTubeTranslation(cur_trans - OT_TRANS_RATE*dt());
        }
        _tip_cursor->setPosition(_active_arm->tipPosition());
    }

    if (_input_device == SimulationInputDevice::HAPTIC)
    {
        HHD handle = _haptic_device_manager->deviceHandles()[0];
        Vec3r cur_pos = _haptic_device_manager->position(handle);

        bool button2_pressed = _haptic_device_manager->button2Pressed(handle);
        
        if (button2_pressed)
        {
            Vec3r dx = cur_pos - _last_haptic_pos;

            // transform dx from haptic input frame to global coordinates
            Vec3r dx_sim = GeometryUtils::Rx(M_PI/2.0) * dx;
            _moveCursor(dx_sim*0.0001);
        }

        _last_haptic_pos = cur_pos;
        
    }
    

    Simulation::_timeStep();
}


} // namespace Sim
