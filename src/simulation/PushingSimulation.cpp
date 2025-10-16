#include "simulation/PushingSimulation.hpp"
#include <cstdio>

namespace Sim
{

PushingSimulation::PushingSimulation(const Config::PushingSimulationConfig* config)
    : Simulation(config), _pushing_enabled(false), _cursor(nullptr)
{
    _tool_radius = config->toolRadius();
    _push_stiffness = config->pushStiffness();
    _max_push_force = config->maxPushForce();
    _fix_min_z = config->fixMinZ();

    // Pre-allocate space for push targets to guarantee pointer stability
    _push_targets.reserve(kMaxPushedVertices);

    // initialize the keys map with relevant keycodes
    SimulationInput::Key keys[] = {
        SimulationInput::Key::SPACE, // space bar for cursor movement
        SimulationInput::Key::W,     // W to increase tool radius
        SimulationInput::Key::S,     // S to decrease tool radius
        SimulationInput::Key::Q,     // Q to increase push stiffness
        SimulationInput::Key::A,     // A to decrease push stiffness
    };

    size_t num_keys = sizeof(keys) / sizeof(keys[0]);
    for (unsigned i = 0; i < num_keys; i++)
        _keys_held[keys[i]] = 0;
}

void PushingSimulation::setup()
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

    // create a visual representation of the tool tip
    Config::RigidSphereConfig cursor_config("pushing_tool", Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0),
        1.0, _tool_radius, false, true, false, Config::ObjectRenderConfig());
    _cursor = _addObjectFromConfig(&cursor_config);
    assert(_cursor);
    
    // Note: RigidSphere doesn't have setColor method, so we can't change color dynamically
}

void PushingSimulation::notifyMouseButtonPressed(SimulationInput::MouseButton button, SimulationInput::MouseAction action, int modifiers)
{
    printf("DEBUG: Mouse button event: button=%d, action=%d\n", static_cast<int>(button), static_cast<int>(action));

    // Left mouse button toggles pushing on/off
    if (button == SimulationInput::MouseButton::LEFT && action == SimulationInput::MouseAction::PRESS)
    {
        _togglePushing();
    }

    Simulation::notifyMouseButtonPressed(button, action, modifiers);
}

void PushingSimulation::notifyMouseMoved(double x, double y)
{
    printf("DEBUG: Mouse moved: x=%.2f, y=%.2f, space_held=%d\n", x, y, 
           _keys_held.count(SimulationInput::Key::SPACE) ? _keys_held.at(SimulationInput::Key::SPACE) : 0);
           
    // Move cursor when spacebar is held
    if (_keys_held.count(SimulationInput::Key::SPACE) && _keys_held.at(SimulationInput::Key::SPACE) > 0)
    {
        // Use gentler scaling when pushing is active to prevent explosions
        const Real base_scaling = _tool_radius / 50.0;
        const Real scaling = _pushing_enabled ? base_scaling * 0.2 : base_scaling; // 5x slower when pushing

        Real dx = x - _last_mouse_pos[0];
        Real dy = y - _last_mouse_pos[1];

        // Limit maximum mouse movement per frame to prevent explosive motion
        const Real max_mouse_delta = 20.0; // pixels
        dx = std::max(-max_mouse_delta, std::min(max_mouse_delta, dx));
        dy = std::max(-max_mouse_delta, std::min(max_mouse_delta, dy));

        // camera plane defined by camera up direction and camera right direction
        const Vec3r up_vec = _graphics_scene->cameraUpDirection();
        const Vec3r right_vec = _graphics_scene->cameraRightDirection();
        
        const Vec3r offset = right_vec * dx + up_vec * -dy; // negate dy since increasing dy is opposite of camera up
        _moveCursor(offset * scaling);
    }

    _last_mouse_pos[0] = x;
    _last_mouse_pos[1] = y;
}

void PushingSimulation::notifyKeyPressed(SimulationInput::Key key, SimulationInput::KeyAction action, int modifiers)
{
    if (key == SimulationInput::Key::SPACE) {
        printf("DEBUG: Spacebar event: action=%d\n", static_cast<int>(action));
    }

    // Update key held state
    auto it = _keys_held.find(key);
    if (it != _keys_held.end())
    {
        it->second = (action == SimulationInput::KeyAction::PRESS);
    }

    Simulation::notifyKeyPressed(key, action, modifiers);
}

void PushingSimulation::notifyMouseScrolled(double dx, double dy)
{
    // Mouse scrolling moves the tool tip in/out when spacebar is held
    if (_keys_held.count(SimulationInput::Key::SPACE) && _keys_held.at(SimulationInput::Key::SPACE) > 0)
    {
        // Use gentler scaling when pushing is active
        const Real base_scaling = _tool_radius / 2.0;
        const Real scaling = _pushing_enabled ? base_scaling * 0.3 : base_scaling; // 3x slower when pushing

        // Limit scroll delta to prevent explosive motion
        const Real limited_dy = std::max(-2.0, std::min(2.0, dy));
        const Vec3r view_dir = _graphics_scene->cameraViewDirection();

        const Vec3r offset = view_dir * limited_dy;
        _moveCursor(offset * scaling);
    }

    Simulation::notifyMouseScrolled(dx, dy);
}

void PushingSimulation::_moveCursor(const Vec3r& dp)
{
    // move the tool cursor
    const Vec3r current_position = _cursor->position();
    _cursor->setPosition(current_position + dp);
}

void PushingSimulation::_timeStep()
{
    // Handle tool radius adjustment with W/S keys
    Real radius_change = 0;
    if (_keys_held.count(SimulationInput::Key::W) && _keys_held.at(SimulationInput::Key::W) > 0)
    {
        radius_change += _tool_radius / 300.0;
    }
    if (_keys_held.count(SimulationInput::Key::S) && _keys_held.at(SimulationInput::Key::S) > 0)
    {
        radius_change -= _tool_radius / 300.0;
    }
    
    // Handle push stiffness adjustment with Q/A keys
    Real stiffness_change = 0;
    if (_keys_held.count(SimulationInput::Key::Q) && _keys_held.at(SimulationInput::Key::Q) > 0)
    {
        stiffness_change += _push_stiffness / 100.0;
    }
    if (_keys_held.count(SimulationInput::Key::A) && _keys_held.at(SimulationInput::Key::A) > 0)
    {
        stiffness_change -= _push_stiffness / 100.0;
    }

    _tool_radius = std::max(0.01, _tool_radius + radius_change);
    _push_stiffness = std::max(1.0, _push_stiffness + stiffness_change);
    _cursor->setRadius(_tool_radius);

    // Apply pushing forces if pushing is enabled
    if (_pushing_enabled)
    {
        _applyPushingForces();
    }

    Simulation::_timeStep();
}

void PushingSimulation::_togglePushing()
{
    _pushing_enabled = !_pushing_enabled;
    
    if (_pushing_enabled)
    {
        printf("DEBUG: Pushing ENABLED at tool position (%.2f, %.2f, %.2f)\n", 
               _cursor->position().x(), _cursor->position().y(), _cursor->position().z());
        // Note: RigidSphere doesn't have setColor method
    }
    else
    {
        printf("DEBUG: Pushing DISABLED\n");
        // Note: RigidSphere doesn't have setColor method
    }
}

void PushingSimulation::_applyPushingForces()
{
    // Get both types of XPBD mesh objects
    std::vector<std::unique_ptr<Sim::XPBDMeshObject_Base>>& xpbd_mesh_objs = _objects.template get<std::unique_ptr<Sim::XPBDMeshObject_Base>>();
    std::vector<std::unique_ptr<Sim::FirstOrderXPBDMeshObject_Base>>& fo_xpbd_mesh_objs = _objects.template get<std::unique_ptr<Sim::FirstOrderXPBDMeshObject_Base>>();
    
    const Vec3r tool_center = _cursor->position();
    int vertices_pushed = 0;
    const int max_vertices_pushed = kMaxPushedVertices; // Limit vertices to prevent explosions

    // First clear any previous pushing constraints
    for (auto& xpbd_mesh_obj : xpbd_mesh_objs)
    {
        xpbd_mesh_obj->clearAttachmentConstraints();
    }
    for (auto& fo_xpbd_mesh_obj : fo_xpbd_mesh_objs)
    {
        fo_xpbd_mesh_obj->clearAttachmentConstraints();
    }
    
    // Store push targets to keep them alive during constraint solving
    _push_targets.clear();
    // Ensure we never exceed reserved capacity to avoid pointer invalidation
    _push_targets.reserve(kMaxPushedVertices);
    
    // Process XPBDMeshObject_Base objects
    for (auto& xpbd_mesh_obj : xpbd_mesh_objs)
    {
        for (int v = 0; v < xpbd_mesh_obj->mesh()->numVertices() && vertices_pushed < max_vertices_pushed; ++v)
        {
            if (xpbd_mesh_obj->vertexFixed(v)) continue;

            const Vec3r vertex_pos = xpbd_mesh_obj->mesh()->vertex(v);
            Real distance = (vertex_pos - tool_center).norm();
            
            // Apply pushing if vertex is within tool radius
            if (distance <= _tool_radius)
            {
                Vec3r push_target = _calculatePushTarget(vertex_pos, tool_center, _tool_radius);
                if ((int)_push_targets.size() < max_vertices_pushed) {
                    _push_targets.emplace_back(push_target);
                    // Create attachment constraint to push the vertex towards the target
                    xpbd_mesh_obj->addAttachmentConstraint(v, &_push_targets.back(), Vec3r(0,0,0));
                }
                vertices_pushed++;
                
                if (vertices_pushed <= 5) // Limit debug output
                {
                    printf("DEBUG: Pushing vertex %d from (%.2f, %.2f, %.2f) towards (%.2f, %.2f, %.2f)\n", 
                           v, vertex_pos.x(), vertex_pos.y(), vertex_pos.z(),
                           push_target.x(), push_target.y(), push_target.z());
                }
            }
        }
    }
    
    // Process FirstOrderXPBDMeshObject_Base objects
    for (auto& fo_xpbd_mesh_obj : fo_xpbd_mesh_objs)
    {
        for (int v = 0; v < fo_xpbd_mesh_obj->mesh()->numVertices() && vertices_pushed < max_vertices_pushed; ++v)
        {
            if (fo_xpbd_mesh_obj->vertexFixed(v)) continue;

            const Vec3r vertex_pos = fo_xpbd_mesh_obj->mesh()->vertex(v);
            Real distance = (vertex_pos - tool_center).norm();
            
            // Apply pushing if vertex is within tool radius
            if (distance <= _tool_radius)
            {
                Vec3r push_target = _calculatePushTarget(vertex_pos, tool_center, _tool_radius);
                if ((int)_push_targets.size() < max_vertices_pushed) {
                    _push_targets.emplace_back(push_target);
                    // Create attachment constraint to push the vertex towards the target
                    fo_xpbd_mesh_obj->addAttachmentConstraint(v, &_push_targets.back(), Vec3r(0,0,0));
                }
                vertices_pushed++;
                
                if (vertices_pushed <= 5) // Limit debug output
                {
                    printf("DEBUG: Pushing vertex %d from (%.2f, %.2f, %.2f) towards (%.2f, %.2f, %.2f)\n", 
                           v, vertex_pos.x(), vertex_pos.y(), vertex_pos.z(),
                           push_target.x(), push_target.y(), push_target.z());
                }
            }
        }
    }

    if (vertices_pushed > 0)
    {
        printf("DEBUG: Applied pushing to %d vertices\n", vertices_pushed);
    }
}

Vec3r PushingSimulation::_calculatePushTarget(const Vec3r& vertex_pos, const Vec3r& tool_center, Real tool_radius)
{
    // Direction outward from tool center
    Vec3r displacement = vertex_pos - tool_center;
    Real distance = displacement.norm();
    
    if (distance < 1e-6) // avoid division by zero - push straight up if at center
    {
        return vertex_pos + Vec3r(0, 0, tool_radius * 0.1);
    }
    
    // Calculate how far the vertex should be pushed
    Real penetration = tool_radius - distance;
    
    if (penetration <= 0)
    {
        return vertex_pos; // No pushing needed
    }
    
    // Normalize displacement to get push direction (away from tool center)
    Vec3r push_direction = displacement / distance;

    // Compute push distance: small fraction of penetration to avoid overshoot
    Real push_distance = penetration * std::min<Real>(1.0, _push_stiffness / 5000.0);

    // Clamp push distance to a very small limit to maintain stability
    const Real max_step = tool_radius * 0.05; // 5% of tool radius per frame
    push_distance = std::min(push_distance, max_step);

    // Soft cap: enforce max push force by limiting displacement magnitude heuristically
    const Real force_scaled_step = _max_push_force / 10000.0; // heuristic mapping N -> meters/frame
    push_distance = std::min(push_distance, force_scaled_step);

    return vertex_pos + push_direction * push_distance;
}

} // namespace Sim