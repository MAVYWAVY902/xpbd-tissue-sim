#include "simulation/VirtuosoSimulation.hpp"

#include "utils/GeometryUtils.hpp"

namespace Sim
{

VirtuosoSimulation::VirtuosoSimulation(const std::string& config_filename)
    : Simulation(), _virtuoso_robot(nullptr), _active_arm(nullptr), _grasping(false)
{
    _config = std::make_unique<VirtuosoSimulationConfig>(YAML::LoadFile(config_filename));

    _init();

    // extract parameters from config object
    VirtuosoSimulationConfig* virtuoso_sim_config = dynamic_cast<VirtuosoSimulationConfig*>(_config.get());
    _input_device = virtuoso_sim_config->inputDevice();
    _fixed_faces_filename = virtuoso_sim_config->fixedFacesFilename();
    _tumor_faces_filename = virtuoso_sim_config->tumorFacesFilename();
    _goal_filename = virtuoso_sim_config->goalFilename();

    if (_input_device == SimulationInputDevice::HAPTIC)
    {
        std::cout << BOLD << "Initializing haptic device..." << RST << std::endl;
        _haptic_device_manager = std::make_unique<HapticDeviceManager>();
        _last_haptic_pos = _haptic_device_manager->position();
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

VirtuosoSimulation::VirtuosoSimulation()
    : Simulation()
{
}

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


    // find the XPBDMeshObject (which we're assuming to be the tissue)
    for (auto& obj : _objects)
    {
        if (XPBDMeshObject_Base* xpbd_obj = dynamic_cast<XPBDMeshObject_Base*>(obj.get()))
        {
            _tissue_obj = xpbd_obj;
            break;
        }
    }

    assert(_tissue_obj);

    if (_fixed_faces_filename.has_value())
    {
        std::set<int> vertices;
        std::vector<int> faces;
        MeshUtils::verticesAndFacesFromFixedFacesFile(_fixed_faces_filename.value(), vertices, faces);
        for (const auto& v : vertices)
        {
            _tissue_obj->fixVertex(v);
        }
    }

    if (_tumor_faces_filename.has_value())
    {
        std::set<int> vertices;
        MeshUtils::verticesAndFacesFromFixedFacesFile(_tumor_faces_filename.value(), vertices, _tumor_faces);
    }

    // create an object at the tip of the robot to show where grasping is
    RigidSphereConfig cursor_config("tip_cursor", Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0),
        1.0, 0.002, false, true, false);
    _tip_cursor = dynamic_cast<RigidSphere*>(_addObjectFromConfig(&cursor_config));
    _tip_cursor->setPosition(_active_arm->tipPosition());

    // create a goal visualization object that the user tries to match
    if (_goal_filename.has_value())
    {
        RigidMeshObjectConfig goal_config("goal_mesh", Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0),
            1.0, false, true, false,
            _goal_filename.value(), 0.06, std::nullopt, false, true, false, Vec4r(0.4, 0.0, 0.0, 0.0), std::nullopt);
        _goal_obj = dynamic_cast<RigidMeshObject*>(_addObjectFromConfig(&goal_config));
        _goal_obj->mesh()->addFaceProperty<bool>("draw", false);
        
        Geometry::MeshProperty<bool>& draw_face_property = _goal_obj->mesh()->getFaceProperty<bool>("draw");
        for (const auto& face_index : _tumor_faces)
        {
            draw_face_property.set(face_index, true);
        }

        // align meshes
        const Vec3r& v0_f0_tissue_obj = _tissue_obj->mesh()->vertex(_tissue_obj->mesh()->face(0)[0]);
        const Vec3r& v0_f0_goal_obj = _goal_obj->mesh()->vertex(_goal_obj->mesh()->face(0)[0]);

        _goal_obj->setPosition( _goal_obj->position() + (v0_f0_tissue_obj - v0_f0_goal_obj) );

        // std::cout << "Difference: " << v0_f0_goal_obj - v0_f0_tissue_obj << std::endl;
    }
    
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

    if (_input_device == SimulationInputDevice::KEYBOARD && key == 32 && action == 1)
    {
        std::cout << "Toggling grasping..." << std::endl;
        _toggleTissueGrasping();
    }

    // if 'B' is pressed, save the tissue mesh to file
    if (key == 66 && action == 1)
    {
        const std::string filename = "tissue_mesh_" + std::to_string(_time) + "_s.obj";
        _tissue_obj->mesh()->writeMeshToObjFile(filename);
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

void VirtuosoSimulation::_toggleTissueGrasping()
{
    if (_grasping)
    {
        _tissue_obj->clearAttachmentConstraints();
        _grasped_vertices.clear();
        _grasping = false;
    }
    else
    {
        // std::set<unsigned> vertices_to_grasp;
        std::map<int, Vec3r> vertices_to_grasp;

        // quick and dirty way to find all vertices in a sphere
        const Vec3r tip_pos = _tip_cursor->position();
        for (int theta = 0; theta < 360; theta+=30)
        {
            for (int phi = 0; phi < 360; phi+=30)
            {
                for (double p = 0; p < _tip_cursor->radius(); p+=_tip_cursor->radius()/5.0)
                {
                    const double x = tip_pos[0] + p*std::sin(phi*M_PI/180)*std::cos(theta*M_PI/180);
                    const double y = tip_pos[1] + p*std::sin(phi*M_PI/180)*std::sin(theta*M_PI/180);
                    const double z = tip_pos[2] + p*std::cos(phi*M_PI/180);
                    int v = _tissue_obj->mesh()->getClosestVertex(Vec3r(x, y, z));

                    // make sure v is inside sphere
                    if ((tip_pos - _tissue_obj->mesh()->vertex(v)).norm() <= _tip_cursor->radius())
                        if (!_tissue_obj->vertexFixed(v))
                        {
                            const Vec3r attachment_offset = (_tissue_obj->mesh()->vertex(v) - tip_pos) * 0.5;
                            vertices_to_grasp[v] = attachment_offset;
                        }
                }
            }
        }

        for (const auto& [v, offset] : vertices_to_grasp)
        {
            _tissue_obj->addAttachmentConstraint(v, &_tip_cursor->position(), offset);
            _grasped_vertices.push_back(v);
        }

        _grasping = true;

        // TODO: remove once we have actual forces from tissue
        _initial_grasp_pos = tip_pos;
    }
    
}

void VirtuosoSimulation::_updateGraphics()
{
    if (_input_device == SimulationInputDevice::HAPTIC)
    {
        Vec3r cur_pos = _haptic_device_manager->position();
        bool button1_pressed = _haptic_device_manager->button1Pressed();
        bool button2_pressed = _haptic_device_manager->button2Pressed();
        
        if (button2_pressed)
        {
            Vec3r dx = cur_pos - _last_haptic_pos;

            // transform dx from haptic input frame to global coordinates
            Vec3r dx_sim = GeometryUtils::Rx(M_PI/2.0) * dx;
            _moveCursor(dx_sim*0.0001);
        }

        if (!_grasping && button1_pressed)
        {
            _toggleTissueGrasping();
        }
        else if (_grasping && !button1_pressed)
        {
            _toggleTissueGrasping();
        }

        _last_haptic_pos = cur_pos;

        if (_grasping)
        {
            Vec3r total_force = Vec3r::Zero();
            for (const auto& v : _grasped_vertices)
            {
                total_force += _tissue_obj->elasticForceAtVertex(v);
            }
            std::cout << "TOTAL GRASPED FORCE: " << total_force[0] << ", " << total_force[1] << ", " << total_force[2] << std::endl;

            // const Vec3r force = 1000*(_initial_grasp_pos - _tip_cursor->position());

            // transform force from global coordinates into haptic input frame
            const Vec3r haptic_force = GeometryUtils::Rx(-M_PI/2.0) * total_force;
            _haptic_device_manager->setForce(haptic_force);
        }
        else
        {
            _haptic_device_manager->setForce(Vec3r::Zero());
        }
        
    }

    // std::cout << "cursor pos: " << _tip_cursor->position() << std::endl;

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
    

    Simulation::_timeStep();
}

} // namespace Sim
