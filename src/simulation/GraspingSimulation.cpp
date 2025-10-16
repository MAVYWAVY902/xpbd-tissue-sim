#include "simulation/GraspingSimulation.hpp"
#include <cstdio>

namespace Sim
{


GraspingSimulation::GraspingSimulation(const Config::GraspingSimulationConfig* config)
    : Simulation(config), _grasping(false), _cursor(nullptr)
{
    _grasp_radius = config->graspRadius();
    _fix_min_z = config->fixMinZ();

    // initialize the keys map with relevant keycodes for controlling the Virtuoso robot with the keyboard
    SimulationInput::Key keys[] = {
        SimulationInput::Key::SPACE, // space bar
        SimulationInput::Key::W, // W
        SimulationInput::Key::S, // S
    };

    size_t num_keys = sizeof(keys) / sizeof(keys[0]);
    for (unsigned i = 0; i < num_keys; i++)
        _keys_held[keys[i]] = 0;
}

void GraspingSimulation::setup()
{
    Simulation::setup();

    if (_fix_min_z)
    {
        // Fix bottom vertices for both types of XPBD mesh objects
        std::vector<std::unique_ptr<Sim::XPBDMeshObject_Base>>& xpbd_mesh_objs = _objects.template get<std::unique_ptr<Sim::XPBDMeshObject_Base>>();
        std::vector<std::unique_ptr<Sim::FirstOrderXPBDMeshObject_Base>>& fo_xpbd_mesh_objs = _objects.template get<std::unique_ptr<Sim::FirstOrderXPBDMeshObject_Base>>();
        
        auto fix_bottom_vertices = [&](auto& mesh_objs) {
            for (auto& obj : mesh_objs)
            {
                // get min z coordinate of the object's mesh
                Vec3r min_bbox_point = obj->mesh()->boundingBox().min;
                std::vector<int> vertices_to_fix = obj->mesh()->getVerticesWithZ(min_bbox_point[2]);
                for (const auto& v : vertices_to_fix)
                {
                    obj->fixVertex(v);
                }
            }
        };
        
        fix_bottom_vertices(xpbd_mesh_objs);
        fix_bottom_vertices(fo_xpbd_mesh_objs);
    }
    

    // create an object to show where grasping is
    Config::RigidSphereConfig cursor_config("cursor", Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0),
        1.0, _grasp_radius, false, true, false, Config::ObjectRenderConfig());
    _cursor = _addObjectFromConfig(&cursor_config);
    assert(_cursor);
}

void GraspingSimulation::notifyMouseButtonPressed(SimulationInput::MouseButton button, SimulationInput::MouseAction action, int modifiers)
{
    printf("DEBUG: Mouse button event: button=%d, action=%d\n", static_cast<int>(button), static_cast<int>(action));

    // button = 0 ==> left mouse button
    // button = 1 ==> right mouse button
    // action = 0 ==> mouse up
    // action = 1 ==> mouse down

    if (button == SimulationInput::MouseButton::LEFT && action == SimulationInput::MouseAction::PRESS)
    {
        _toggleGrasping();
    }

    Simulation::notifyMouseButtonPressed(button, action, modifiers);

}

void GraspingSimulation::notifyMouseMoved(double x, double y)
{
    printf("DEBUG: Mouse moved: x=%.2f, y=%.2f, space_held=%d\n", x, y, _keys_held.count(SimulationInput::Key::SPACE) ? _keys_held.at(SimulationInput::Key::SPACE) : 0);
    if (_keys_held.count(SimulationInput::Key::SPACE) && _keys_held.at(SimulationInput::Key::SPACE) > 0) // space bar = clutch
    {
    // Increase sensitivity: more world motion per pixel; still slow down when grasping
    const Real base_scaling = _grasp_radius/30.0; // was /50.0 (≈3.3x more sensitive)
    const Real scaling = _grasping ? base_scaling * 0.35 : base_scaling; // was 0.2
        
        Real dx = x - _last_mouse_pos[0];
        Real dy = y - _last_mouse_pos[1];
        
    // Limit maximum mouse movement per frame to prevent explosive motion
    const Real max_mouse_delta = _grasping ? 10.0 : 20.0; // tighter when grasping
    dx = std::max(-max_mouse_delta, std::min(max_mouse_delta, dx));
    dy = std::max(-max_mouse_delta, std::min(max_mouse_delta, dy));

        // camera plane defined by camera up direction and camera right direction
        // changes in mouse y position = changes along camera up direction
        // changes in mouse x position = changes along camera right direction
    const Vec3r up_vec = _graphics_scene->cameraUpDirection();
    const Vec3r right_vec = _graphics_scene->cameraRightDirection();
        
    // Map screen-space mouse to world: right maps to +cameraRight, up maps to +cameraUp
    // Using +dy here so moving mouse up moves tool up in the scene
    const Vec3r offset = right_vec*dx + up_vec*dy;
        _moveCursor(offset*scaling);
    }

    _last_mouse_pos[0] = x;
    _last_mouse_pos[1] = y;
}

void GraspingSimulation::notifyKeyPressed(SimulationInput::Key key, SimulationInput::KeyAction action, int modifiers)
{
    if (key == SimulationInput::Key::SPACE) {
        printf("DEBUG: Spacebar event: action=%d\n", static_cast<int>(action));
    }

    // find key in map
    auto it = _keys_held.find(key);
    if (it != _keys_held.end())
    {
        it->second = (action == SimulationInput::KeyAction::PRESS); // if action > 0, key is pressed or held
    }

    Simulation::notifyKeyPressed(key, action, modifiers);

}

void GraspingSimulation::notifyMouseScrolled(double dx, double dy)
{
    // when using mouse input, mouse scrolling moves the robot tip in and out of the page
    if (_keys_held.count(SimulationInput::Key::SPACE) && _keys_held.at(SimulationInput::Key::SPACE) > 0) // space bar = clutch
    {
    // Increase scroll sensitivity; still slow down when grasping
    const Real base_scaling = _grasp_radius/1.0; // was /2.0 (2x more sensitive)
    const Real scaling = _grasping ? base_scaling * 0.5 : base_scaling; // was 0.3
        
        // Limit scroll delta to prevent explosive motion
        const Real limited_dy = std::max(-2.0, std::min(2.0, dy));
        
        const Vec3r view_dir = _graphics_scene->cameraViewDirection();
        const Vec3r offset = view_dir*limited_dy;
        _moveCursor(offset*scaling);
    }
    

    Simulation::notifyMouseScrolled(dx, dy);
    
}

void GraspingSimulation::_moveCursor(const Vec3r& dp)
{
    // move the tip cursor and the active arm tip position
    const Vec3r current_position = _cursor->position();
    _cursor->setPosition(current_position + dp);
}

void GraspingSimulation::_timeStep()
{
    Real grasp_radius_change = 0;
    if (_keys_held.count(SimulationInput::Key::W) && _keys_held.at(SimulationInput::Key::W) > 0) // W = increase grasping radius
    {
        grasp_radius_change += _grasp_radius/300.0;

    }
    if (_keys_held.count(SimulationInput::Key::S) && _keys_held.at(SimulationInput::Key::S) > 0) // S = decrease grasping radius
    {
        grasp_radius_change += -_grasp_radius/300.0;
    }
    _grasp_radius += grasp_radius_change;
    _cursor->setRadius(_grasp_radius);
    

    Simulation::_timeStep();
}

void GraspingSimulation::_toggleGrasping()
{
    // Get both types of XPBD mesh objects
    std::vector<std::unique_ptr<Sim::XPBDMeshObject_Base>>& xpbd_mesh_objs = _objects.template get<std::unique_ptr<Sim::XPBDMeshObject_Base>>();
    std::vector<std::unique_ptr<Sim::FirstOrderXPBDMeshObject_Base>>& fo_xpbd_mesh_objs = _objects.template get<std::unique_ptr<Sim::FirstOrderXPBDMeshObject_Base>>();
    
    printf("DEBUG: Found %zu XPBDMeshObject_Base objects and %zu FirstOrderXPBDMeshObject_Base objects\n", 
           xpbd_mesh_objs.size(), fo_xpbd_mesh_objs.size());
    
    // if not currently grasping, start grasping vertices inside grasping radius
    if (!_grasping)
    {
    printf("DEBUG: Toggling grasp ON.\n");

    // Ensure a clean slate before adding new attachments
    for (auto& xpbd_mesh_obj : xpbd_mesh_objs) { xpbd_mesh_obj->clearAttachmentConstraints(); }
    for (auto& fo_xpbd_mesh_obj : fo_xpbd_mesh_objs) { fo_xpbd_mesh_obj->clearAttachmentConstraints(); }

        const Vec3r grasp_center = _cursor->position();
        printf("DEBUG: Grasp center is at (%.2f, %.2f, %.2f)\n", grasp_center.x(), grasp_center.y(), grasp_center.z());

        // --- NEW RELIABLE METHOD ---
        printf("DEBUG: Number of deformable objects: %zu\n", xpbd_mesh_objs.size() + fo_xpbd_mesh_objs.size());
        printf("DEBUG: Grasp radius: %.4f\n", _grasp_radius);
        
        int total_vertices_found = 0;
        
        // Process XPBDMeshObject_Base objects
        for (auto& xpbd_mesh_obj : xpbd_mesh_objs)
        {
            printf("DEBUG: Processing XPBDMeshObject with %d vertices\n", xpbd_mesh_obj->mesh()->numVertices());
            int fixed_count = 0;
            int within_radius_count = 0;
            
            // Iterate over every vertex in the mesh
            int attached_here = 0;
            const int kMaxAttachPerObject = 256; // cap constraints per object to improve stability
            for (int v = 0; v < xpbd_mesh_obj->mesh()->numVertices(); ++v)
            {
                // Check if the vertex is already fixed
                if (xpbd_mesh_obj->vertexFixed(v))
                {
                    fixed_count++;
                    continue;
                }

                // Prefer attaching on surface vertices for stability (skip internal nodes if info present)
                if (xpbd_mesh_obj->mesh()->hasVertexProperty<bool>("surface"))
                {
                    if (!xpbd_mesh_obj->mesh()->vertexOnSurface(v))
                        continue;
                }

                const Vec3r vertex_pos = xpbd_mesh_obj->mesh()->vertex(v);
                Real distance = (vertex_pos - grasp_center).norm();
                
                // Check if the vertex is inside the grasping sphere
                if (distance <= _grasp_radius && attached_here < kMaxAttachPerObject)
                {
                    within_radius_count++;
                    total_vertices_found++;
                    // Use RELATIVE OFFSET so initial error is zero: target = cursor_pos + (vertex_pos - cursor_pos)
                    // This avoids a large initial pull to the cursor center and is more stable.
                    const Vec3r attachment_offset = xpbd_mesh_obj->mesh()->vertex(v) - _cursor->position();
                    xpbd_mesh_obj->addAttachmentConstraint(v, &_cursor->position(), attachment_offset);
                    attached_here++;
                    _grasped_vertices.push_back(std::make_pair(xpbd_mesh_obj.get(), v));
                    printf("DEBUG: Vertex %d at (%.2f, %.2f, %.2f) distance %.4f - SELECTED\n", 
                           v, vertex_pos.x(), vertex_pos.y(), vertex_pos.z(), distance);
                }
            }
            printf("DEBUG: XPBDMeshObject summary - Fixed vertices: %d, Vertices within radius: %d\n", fixed_count, within_radius_count);
        }
        
        // Process FirstOrderXPBDMeshObject_Base objects
        for (auto& fo_xpbd_mesh_obj : fo_xpbd_mesh_objs)
        {
            printf("DEBUG: Processing FirstOrderXPBDMeshObject with %d vertices\n", fo_xpbd_mesh_obj->mesh()->numVertices());
            int fixed_count = 0;
            int within_radius_count = 0;
            
            // Iterate over every vertex in the mesh
            int attached_here = 0;
            const int kMaxAttachPerObject = 256; // cap constraints per object to improve stability
            for (int v = 0; v < fo_xpbd_mesh_obj->mesh()->numVertices(); ++v)
            {
                // Check if the vertex is already fixed
                if (fo_xpbd_mesh_obj->vertexFixed(v))
                {
                    fixed_count++;
                    continue;
                }

                // Prefer attaching on surface vertices for stability (skip internal nodes if info present)
                if (fo_xpbd_mesh_obj->mesh()->hasVertexProperty<bool>("surface"))
                {
                    if (!fo_xpbd_mesh_obj->mesh()->vertexOnSurface(v))
                        continue;
                }

                const Vec3r vertex_pos = fo_xpbd_mesh_obj->mesh()->vertex(v);
                Real distance = (vertex_pos - grasp_center).norm();
                
                // Check if the vertex is inside the grasping sphere
                if (distance <= _grasp_radius && attached_here < kMaxAttachPerObject)
                {
                    within_radius_count++;
                    total_vertices_found++;
                    // Use RELATIVE OFFSET so initial error is zero for stability
                    const Vec3r attachment_offset = fo_xpbd_mesh_obj->mesh()->vertex(v) - _cursor->position();
                    fo_xpbd_mesh_obj->addAttachmentConstraint(v, &_cursor->position(), attachment_offset);
                    attached_here++;
                    // Note: Not storing FirstOrder objects in _grasped_vertices due to template type mismatch
                    printf("DEBUG: Vertex %d at (%.2f, %.2f, %.2f) distance %.4f - SELECTED\n", 
                           v, vertex_pos.x(), vertex_pos.y(), vertex_pos.z(), distance);
                }
            }
            printf("DEBUG: FirstOrderXPBDMeshObject summary - Fixed vertices: %d, Vertices within radius: %d\n", fixed_count, within_radius_count);
        }
        // --- END NEW METHOD ---

        printf("DEBUG: Found %d vertices to grasp.\n", total_vertices_found);
    }

    // if tool state has changed from 1 to 0, stop grasping
    else if (_grasping)
    {
        printf("DEBUG: Toggling grasp OFF.\n");
        for (auto& xpbd_mesh_obj : xpbd_mesh_objs)
        {
            xpbd_mesh_obj->clearAttachmentConstraints();
        }
        for (auto& fo_xpbd_mesh_obj : fo_xpbd_mesh_objs)
        {
            fo_xpbd_mesh_obj->clearAttachmentConstraints();
        }
        _grasped_vertices.clear();
    }

    _grasping = !_grasping;
}

} // namespace Sim