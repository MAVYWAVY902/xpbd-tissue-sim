#include "simulation/VirtuosoSimulation.hpp"

namespace Sim
{

VirtuosoSimulation::VirtuosoSimulation(const std::string& config_filename)
    : Simulation()
{
    _config = std::make_unique<VirtuosoSimulationConfig>(YAML::LoadFile(config_filename));

    _init();

    // extract parameters from config object
    VirtuosoSimulationConfig* virtuoso_sim_config = dynamic_cast<VirtuosoSimulationConfig*>(_config.get());
    _input_device = virtuoso_sim_config->inputDevice();
    _fixed_faces_filename = virtuoso_sim_config->fixedFacesFilename();

    if (_input_device == SimulationInputDevice::HAPTIC)
    {
        std::cout << BOLD << "Initializing haptic device..." << RST << std::endl;
        _haptic_device_manager = std::make_unique<HapticDeviceManager>();
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

VirtuosoSimulation::VirtuosoSimulation()
    : Simulation()
{
}

void VirtuosoSimulation::setup()
{
    Simulation::setup();
    
    // find the VirtuosoArm object
    for (auto& obj : _objects)
    {
        if (VirtuosoArm* arm = dynamic_cast<VirtuosoArm*>(obj.get()))
        {
            _virtuoso_arm = arm;
            break;
        }
    }

    // create an object at the tip of the robot to show where grasping is
    std::unique_ptr<RigidSphere> tip_cursor_ptr = std::make_unique<RigidSphere>(this, "tip cursor", 0.001);
    _tip_cursor = tip_cursor_ptr.get();
    _addObject(std::move(tip_cursor_ptr));
}

void VirtuosoSimulation::notifyMouseButtonPressed(int button, int action, int modifiers)
{

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
            const Eigen::Vector3d up_vec = _graphics_scene->cameraUpDirection();
            const Eigen::Vector3d right_vec = _graphics_scene->cameraRightDirection();
            
            const Eigen::Vector3d current_tip_position = _tip_cursor->position();
            const Eigen::Vector3d offset = right_vec*dx + up_vec*-dy; // negate dy since increasing dy is actually opposite of camera frame up vec
            _tip_cursor->setPosition(current_tip_position + offset*scaling);
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

}

void VirtuosoSimulation::notifyMouseScrolled(double dx, double dy)
{
    if (_input_device == SimulationInputDevice::MOUSE)
    {
        if (_keys_held[32] > 0) // space bar = clutch
        {
            const double scaling = 0.0005;
            const Eigen::Vector3d view_dir = _graphics_scene->cameraViewDirection();

            const Eigen::Vector3d current_tip_position = _tip_cursor->position();
            const Eigen::Vector3d offset = view_dir*dy;
            _tip_cursor->setPosition(current_tip_position + offset*scaling);
        }
    }
    
}

void VirtuosoSimulation::_timeStep()
{
    if (_input_device == SimulationInputDevice::KEYBOARD)
    {
        if (_keys_held[81] > 0) // Q = CCW inner tube rotation
        {
            const double cur_rot = _virtuoso_arm->innerTubeRotation();
            _virtuoso_arm->setInnerTubeRotation(cur_rot + IT_ROT_RATE*dt());

        }
        if (_keys_held[87] > 0) // W = CCW outer tube rotation
        {
            const double cur_rot = _virtuoso_arm->outerTubeRotation();
            _virtuoso_arm->setOuterTubeRotation(cur_rot + OT_ROT_RATE*dt());
        }
        if (_keys_held[69] > 0) // E = inner tube extension
        {
            const double cur_trans = _virtuoso_arm->innerTubeTranslation();
            _virtuoso_arm->setInnerTubeTranslation(cur_trans + IT_TRANS_RATE*dt());
        }
        if (_keys_held[82] > 0) // R = outer tube extension
        {
            const double cur_trans = _virtuoso_arm->outerTubeTranslation();
            _virtuoso_arm->setOuterTubeTranslation(cur_trans + OT_TRANS_RATE*dt());
        }
        if (_keys_held[65] > 0) // A = CW inner tube rotation
        {
            const double cur_rot = _virtuoso_arm->innerTubeRotation();
            _virtuoso_arm->setInnerTubeRotation(cur_rot - IT_ROT_RATE*dt()); 
        }
        if (_keys_held[83] > 0) // S = CW outer tube rotation
        {
            const double cur_rot = _virtuoso_arm->outerTubeRotation();
            _virtuoso_arm->setOuterTubeRotation(cur_rot - OT_ROT_RATE*dt());
        }
        if (_keys_held[68] > 0) // D = inner tube retraction
        {
            const double cur_trans = _virtuoso_arm->innerTubeTranslation();
            _virtuoso_arm->setInnerTubeTranslation(cur_trans - IT_TRANS_RATE*dt());
        }
        if (_keys_held[70] > 0) // F = outer tube retraction
        {
            const double cur_trans = _virtuoso_arm->outerTubeTranslation();
            _virtuoso_arm->setOuterTubeTranslation(cur_trans - OT_TRANS_RATE*dt());
        }
        
    }

    Simulation::_timeStep();
}

} // namespace Sim
