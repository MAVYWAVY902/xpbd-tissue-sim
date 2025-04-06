#include "simulation/VirtuosoSimulation.hpp"

namespace Sim
{

VirtuosoSimulation::VirtuosoSimulation(const std::string& config_filename)
    : Simulation(), _grasping(false)
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

    for (auto& obj : _objects)
    {
        if (XPBDMeshObject* xpbd_obj = dynamic_cast<XPBDMeshObject*>(obj.get()))
        {
            _tissue_obj = xpbd_obj;
            break;
        }
    }

    assert(_tissue_obj);

    if (_fixed_faces_filename.has_value())
    {
        std::set<unsigned> vertices = MeshUtils::verticesFromFixedFacesFile(_fixed_faces_filename.value());
        for (const auto& v : vertices)
        {
            _tissue_obj->fixVertex(v);
        }
    }

    // create an object at the tip of the robot to show where grasping is
    std::unique_ptr<RigidSphere> tip_cursor_ptr = std::make_unique<RigidSphere>(this, "tip cursor", 0.001);
    tip_cursor_ptr->setPosition(_virtuoso_arm->tipPosition());
    _tip_cursor = tip_cursor_ptr.get();
    _addObject(std::move(tip_cursor_ptr));
}

void VirtuosoSimulation::notifyMouseButtonPressed(int button, int action, int modifiers)
{
    if (_input_device != SimulationInputDevice::MOUSE)
    {
        return;
    }

    // button = 0 ==> left mouse button
    // button = 1 ==> right mouse button
    // action = 0 ==> mouse up
    // action = 1 ==> mouse down
    
    if (button == 0 && action == 1)
    {
        _toggleTissueGrasping();
    }

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
            const Eigen::Vector3d up_vec = _graphics_scene->cameraUpDirection();
            const Eigen::Vector3d right_vec = _graphics_scene->cameraRightDirection();
            
            const Eigen::Vector3d current_tip_position = _tip_cursor->position();
            const Eigen::Vector3d offset = right_vec*dx + up_vec*-dy; // negate dy since increasing dy is actually opposite of camera frame up vec
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
            _moveCursor(offset*scaling);
        }
    }
    
}

void VirtuosoSimulation::_moveCursor(const Eigen::Vector3d& dp)
{
    const Eigen::Vector3d current_tip_position = _tip_cursor->position();
    _tip_cursor->setPosition(current_tip_position + dp);
    _virtuoso_arm->setTipPosition(_tip_cursor->position());
}

void VirtuosoSimulation::_toggleTissueGrasping()
{
    if (_grasping)
    {
        _tissue_obj->clearAttachmentConstraints();
        _grasping = false;
    }
    else
    {
        std::set<unsigned> vertices_to_grasp;

        // quick and dirty way to find all vertices in a sphere
        const Eigen::Vector3d tip_pos = _tip_cursor->position();
        for (int theta = 0; theta < 360; theta+=30)
        {
            for (int phi = 0; phi < 360; phi+=30)
            {
                for (double p = 0; p < _tip_cursor->radius(); p+=_tip_cursor->radius()/5.0)
                {
                    const double x = tip_pos[0] + p*std::sin(phi*M_PI/180)*std::cos(theta*M_PI/180);
                    const double y = tip_pos[1] + p*std::sin(phi*M_PI/180)*std::sin(theta*M_PI/180);
                    const double z = tip_pos[2] + p*std::cos(phi*M_PI/180);
                    unsigned v = _tissue_obj->mesh()->getClosestVertex(Eigen::Vector3d(x, y, z));

                    // make sure v is inside sphere
                    if ((tip_pos - _tissue_obj->mesh()->vertex(v)).norm() <= _tip_cursor->radius())
                        if (!_tissue_obj->vertexFixed(v))
                            vertices_to_grasp.insert(v);
                }
            }
        }

        for (const auto& v : vertices_to_grasp)
        {
            _tissue_obj->addAttachmentConstraint(v, &_tip_cursor->position(), Eigen::Vector3d::Zero());
        }

        _grasping = true;
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
