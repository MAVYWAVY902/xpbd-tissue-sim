#include "simulation/VirtuosoTissueGraspingSimulation.hpp"
#include "config/VirtuosoTissueGraspingSimulationConfig.hpp"

#include "simulation/VirtuosoSimulation.hpp"

#include "utils/GeometryUtils.hpp"

#include <filesystem>
#include <stdlib.h>

namespace Sim
{

VirtuosoTissueGraspingSimulation::VirtuosoTissueGraspingSimulation(const VirtuosoTissueGraspingSimulationConfig* config)
: VirtuosoSimulation(config), _grasping(false)
{
    // extract parameters from config object
    _input_device = config->inputDevice();
    _fixed_faces_filename = config->fixedFacesFilename();
    _tumor_faces_filename = config->tumorFacesFilename();
    _goal_filename = config->goalFilename();
    _goals_folder = config->goalsFolder();
    _goal_active = false;
    _current_score = 0;

    _dummy = Vec3r::Zero();
}

void VirtuosoTissueGraspingSimulation::setup()
{
    VirtuosoSimulation::setup();

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

    // once we've found the tissue object, make sure that each virtuoso arm knows that this is the object that they're manipulating
    // (the VirtuosoArm class handles the grasping logic)
    if (_virtuoso_robot->hasArm1())
        _virtuoso_robot->arm1()->setToolManipulatedObject(_tissue_obj);
    if (_virtuoso_robot->hasArm2())
        _virtuoso_robot->arm2()->setToolManipulatedObject(_tissue_obj);


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
    

    // create a goal visualization object that the user tries to match
    if (_goals_folder.has_value())
    {
        std::cout << "Loading goals..." << std::endl;
        for (const auto & entry : std::filesystem::directory_iterator(_goals_folder.value()))
        {
            RigidMeshObjectConfig goal_config(entry.path(), Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0),
                1.0, false, true, false,
                entry.path(), 0.06, std::nullopt, false, true, false, Vec4r(0.4, 0.0, 0.0, 0.0), std::nullopt);
            RigidMeshObject* goal_obj = dynamic_cast<RigidMeshObject*>(_addObjectFromConfig(&goal_config));
            goal_obj->mesh()->addFaceProperty<bool>("draw", false);
            
            // align meshes based on the first face (the meshes are loaded and moved about their center of mass, which may be different between the two)
            const Vec3r& v0_f0_tissue_obj = _tissue_obj->mesh()->vertex(_tissue_obj->mesh()->face(0)[0]);
            const Vec3r& v0_f0_goal_obj = goal_obj->mesh()->vertex(goal_obj->mesh()->face(0)[0]);

            goal_obj->setPosition( goal_obj->position() + (v0_f0_tissue_obj - v0_f0_goal_obj) );

            _goal_objs.push_back(goal_obj);

            std::cout << entry.path() << std::endl;
        }
        
        // get a random goal obj to start
        _goal_obj_ind = rand() % _goal_objs.size();
    }
    
    else if (_goal_filename.has_value())
    {
        RigidMeshObjectConfig goal_config("goal_mesh", Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0),
            1.0, false, true, false,
            _goal_filename.value(), 0.06, std::nullopt, false, true, false, Vec4r(0.4, 0.0, 0.0, 0.0), std::nullopt);
        RigidMeshObject* goal_obj = dynamic_cast<RigidMeshObject*>(_addObjectFromConfig(&goal_config));
        goal_obj->mesh()->addFaceProperty<bool>("draw", false);

        // align meshes based on the first face (the meshes are loaded and moved about their center of mass, which may be different between the two)
        const Vec3r& v0_f0_tissue_obj = _tissue_obj->mesh()->vertex(_tissue_obj->mesh()->face(0)[0]);
        const Vec3r& v0_f0_goal_obj = goal_obj->mesh()->vertex(goal_obj->mesh()->face(0)[0]);

        goal_obj->setPosition( goal_obj->position() + (v0_f0_tissue_obj - v0_f0_goal_obj) );

        _goal_objs.push_back(goal_obj);
        _goal_obj_ind = 0;

        // std::cout << "Difference: " << v0_f0_goal_obj - v0_f0_tissue_obj << std::endl;
    }

    if (_tumor_faces_filename.has_value())
    {
        std::set<int> vertices;
        MeshUtils::verticesAndFacesFromFixedFacesFile(_tumor_faces_filename.value(), vertices, _tumor_faces);
    }

    if (_goal_objs.size() > 0)
    {
        _graphics_scene->viewer()->addText("score", "Current Score: ", 10.0f, 35.0f, 15.0f, Graphics::Viewer::TextAlignment::LEFT, Graphics::Viewer::Font::MAO, std::array<float,3>({0,0,0}), 0.5f, false);
    }
    
}

void VirtuosoTissueGraspingSimulation::notifyMouseButtonPressed(int button, int action, int modifiers)
{

    // button = 0 ==> left mouse button
    // button = 1 ==> right mouse button
    // action = 0 ==> mouse up
    // action = 1 ==> mouse down
    
    if (_input_device == SimulationInputDevice::MOUSE && button == 0 && action == 1)
    {
        // _toggleTissueGrasping();
        _active_arm->setToolState(!_active_arm->toolState());
    }

    VirtuosoSimulation::notifyMouseButtonPressed(button, action, modifiers);

}

void VirtuosoTissueGraspingSimulation::notifyMouseMoved(double x, double y)
{
    VirtuosoSimulation::notifyMouseMoved(x, y);
}

void VirtuosoTissueGraspingSimulation::notifyKeyPressed(int key, int action, int modifiers)
{

    // if input mode is keyboard, space bar grasps
    if (_input_device == SimulationInputDevice::KEYBOARD && key == 32 && action == 1)
    {
        _active_arm->setToolState(!_active_arm->toolState());
        // _toggleTissueGrasping();
    }

    // if 'B' is pressed, save the tissue mesh to file
    if (key == 66 && action == 1)
    {
        const std::string filename = "tissue_mesh_" + std::to_string(_time) + "_s.obj";
        _tissue_obj->mesh()->writeMeshToObjFile(filename);
    }

    // if 'Z' is pressed, toggle the goal
    if (key == 90 && action == 1 && _goal_objs.size() > 0)
    {
        _toggleGoal();
    }

    // if 'C' is pressed, change goals
    if (key == 67 && action == 1 && _goal_objs.size() > 0)
    {
        _changeGoal();
    }

    VirtuosoSimulation::notifyKeyPressed(key, action, modifiers);

}

void VirtuosoTissueGraspingSimulation::notifyMouseScrolled(double dx, double dy)
{
    VirtuosoSimulation::notifyMouseScrolled(dx, dy);
}

void VirtuosoTissueGraspingSimulation::_toggleTissueGrasping()
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
                            const Vec3r attachment_offset = (_tissue_obj->mesh()->vertex(v) - tip_pos) * 1.0;
                            vertices_to_grasp[v] = attachment_offset;
                        }
                }
            }
        }

        for (const auto& [v, offset] : vertices_to_grasp)
        {
            // _tissue_obj->addAttachmentConstraint(v, &_dummy, offset);
            _tissue_obj->addAttachmentConstraint(v, &_tip_cursor->position(), offset);
            
            _grasped_vertices.push_back(v);
        }

        _grasping = true;
    }
    
}

void VirtuosoTissueGraspingSimulation::_updateGraphics()
{

    Simulation::_updateGraphics();
}

void VirtuosoTissueGraspingSimulation::_timeStep()
{

    VirtuosoSimulation::_timeStep();

    if (_input_device == SimulationInputDevice::HAPTIC)
    {
        HHD handle = _haptic_device_manager->deviceHandles()[0];

        bool button1_pressed = _haptic_device_manager->button1Pressed(handle);

        if (!_grasping && button1_pressed)
        {
            _toggleTissueGrasping();
        }
        else if (_grasping && !button1_pressed)
        {
            _toggleTissueGrasping();
        }

        if (_grasping)
        {
            Vec3r total_force = Vec3r::Zero();
            for (const auto& v : _grasped_vertices)
            {
                total_force += _tissue_obj->elasticForceAtVertex(v);
            }

            // smooth forces
            Vec3r new_force = total_force*0.5 + _last_force*0.5;

            // const Vec3r force = 1000*(_initial_grasp_pos - _tip_cursor->position());

            std::cout << "TOTAL GRASPED FORCE: " << total_force[0] << ", " << total_force[1] << ", " << total_force[2] << std::endl;

            // transform force from global coordinates into haptic input frame
            const Vec3r haptic_force = GeometryUtils::Rx(-M_PI/2.0) * new_force;
            _haptic_device_manager->setForce(handle, haptic_force);

            _last_force = new_force;
        }
        else
        {
            _haptic_device_manager->setForce(handle, Vec3r::Zero());
        }
        
    }

    Simulation::_timeStep();
}

void VirtuosoTissueGraspingSimulation::_toggleGoal()
{
    if (_goal_objs.size() == 0)
        return;

    // toggle class variable
    _goal_active = !_goal_active;

    // update draw property
    Geometry::MeshProperty<bool>& draw_face_property = _goal_objs[_goal_obj_ind]->mesh()->getFaceProperty<bool>("draw");
    for (const auto& face_index : _tumor_faces)
    {
        draw_face_property.set(face_index, _goal_active);
    }
}

void VirtuosoTissueGraspingSimulation::_changeGoal()
{
    // update draw property
    Geometry::MeshProperty<bool>& draw_face_property_old = _goal_objs[_goal_obj_ind]->mesh()->getFaceProperty<bool>("draw");
    for (const auto& face_index : _tumor_faces)
    {
        draw_face_property_old.set(face_index, false);
    }

    // increment goal object index
    _goal_obj_ind++;
    if (_goal_obj_ind >= _goal_objs.size())
        _goal_obj_ind = 0;
    
    // show the new goal
    Geometry::MeshProperty<bool>& draw_face_property_new = _goal_objs[_goal_obj_ind]->mesh()->getFaceProperty<bool>("draw");
    for (const auto& face_index : _tumor_faces)
    {
        draw_face_property_new.set(face_index, _goal_active);
    }
    
}

int VirtuosoTissueGraspingSimulation::_calculateScore()
{
    assert(_goal_objs.size() > 0 && "Cannot calculate score when goal obj does not exist!");

    RigidMeshObject* goal_obj = _goal_objs[_goal_obj_ind];
    double total_mse = 0;
    for (const auto& face_index : _tumor_faces)
    {
        const Vec3i& tissue_face = _tissue_obj->mesh()->face(face_index);
        const Vec3r& tissue_v1 = _tissue_obj->mesh()->vertex(tissue_face[0]);
        const Vec3r& tissue_v2 = _tissue_obj->mesh()->vertex(tissue_face[1]);
        const Vec3r& tissue_v3 = _tissue_obj->mesh()->vertex(tissue_face[2]);

        const Vec3i& goal_face = goal_obj->mesh()->face(face_index);
        const Vec3r& goal_v1 = goal_obj->mesh()->vertex(goal_face[0]);
        const Vec3r& goal_v2 = goal_obj->mesh()->vertex(goal_face[1]);
        const Vec3r& goal_v3 = goal_obj->mesh()->vertex(goal_face[2]);
        total_mse += (tissue_v1 - goal_v1).squaredNorm() + (tissue_v2 - goal_v2).squaredNorm() + (tissue_v3 - goal_v3).squaredNorm();
    }

    return std::max(0, 10000 - static_cast<int>(100000*total_mse));
}

} // namespace Sim
