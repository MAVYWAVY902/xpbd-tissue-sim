#include "simulation/PushingSimulation.hpp"
#include "config/simobject/RigidMeshObjectConfig.hpp"
#include "simobject/RigidMeshObject.hpp"
#include <cstdio>

namespace Sim
{

PushingSimulation::PushingSimulation(const Config::PushingSimulationConfig* config)
    : Simulation(config), _pushing_enabled(false), _cursor(nullptr)
{
    _tool_radius = config->toolRadius();
    _push_stiffness = config->pushStiffness();
    _push_damping = config->pushDamping();
    _max_push_force = config->maxPushForce();
    _fix_min_z = config->fixMinZ();
    _fix_max_z = config->fixMaxZ();
    _knife_scale_x = config->knifeScaleX();
    _knife_scale_y = config->knifeScaleY();
    _knife_scale_z = config->knifeScaleZ();

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
    
    if (_fix_max_z)
    {
        // Fix top vertices for both types of XPBD mesh objects
        std::vector<std::unique_ptr<Sim::XPBDMeshObject_Base>>& xpbd_mesh_objs = _objects.template get<std::unique_ptr<Sim::XPBDMeshObject_Base>>();
        std::vector<std::unique_ptr<Sim::FirstOrderXPBDMeshObject_Base>>& fo_xpbd_mesh_objs = _objects.template get<std::unique_ptr<Sim::FirstOrderXPBDMeshObject_Base>>();
        
        auto fix_top_vertices = [&](auto& mesh_objs) {
            for (auto& obj : mesh_objs)
            {
                // get max z coordinate of the object's mesh
                Vec3r max_bbox_point = obj->mesh()->boundingBox().max;
                std::vector<int> vertices_to_fix = obj->mesh()->getVerticesWithZ(max_bbox_point[2]);
                for (const auto& v : vertices_to_fix)
                {
                    obj->fixVertex(v);
                }
            }
        };
        
        fix_top_vertices(xpbd_mesh_objs);
        fix_top_vertices(fo_xpbd_mesh_objs);
    }

    // create a visual representation of the knife tool
    std::cout << "[PushingSimulation] Creating knife tool..." << std::endl;
    
    // Graphics-only knife - no collision, controlled manually
    Vec3r knife_initial_position(0.15, 0.0, 0.05);
    
    // Determine scaling mode: directional or uniform
    std::optional<Real> max_size_param = std::nullopt;
    std::optional<Vec3r> size_param = std::nullopt;
    
    if (_knife_scale_x > 0 || _knife_scale_y > 0 || _knife_scale_z > 0) {
        // Directional scaling mode
        Real scale_x = (_knife_scale_x > 0) ? _knife_scale_x : _tool_radius;
        Real scale_y = (_knife_scale_y > 0) ? _knife_scale_y : _tool_radius;
        Real scale_z = (_knife_scale_z > 0) ? _knife_scale_z : _tool_radius;
        size_param = Vec3r(scale_x, scale_y, scale_z);
        std::cout << "[PushingSimulation] Knife directional scale: (" 
                  << scale_x << ", " << scale_y << ", " << scale_z << ") m" << std::endl;
    } else {
        // Uniform scaling mode (default)
        max_size_param = _tool_radius;
        std::cout << "[PushingSimulation] Knife uniform scale: " << _tool_radius << " m" << std::endl;
    }
    
    Config::RigidMeshObjectConfig cursor_config(
        "pushing_tool",                                    // name
        knife_initial_position,                            // initial position (away from objects)
        Vec3r(0,0,0),                                      // initial rotation
        Vec3r(0,0,0),                                      // initial velocity
        Vec3r(0,0,0),                                      // initial angular velocity
        1.0,                                               // density
        false,                                             // collisions (DISABLED - knife is graphics only)
        true,                                              // graphics_only (TRUE = no physics collision)
        false,                                             // fixed (allow manual movement)
        "../resource/tools/convex_knife_scaled.obj",      // filename
        max_size_param,                                    // max_size (uniform scaling)
        size_param,                                        // size (directional scaling)
        false,                                             // draw_points
        true,                                              // draw_edges
        true,                                              // draw_faces
        Vec4r(0.8, 0.8, 0.8, 1.0),                        // color (silver/gray for knife)
        std::nullopt,                                      // sdf_filename
        Config::ObjectRenderConfig()                       // render_config
    );
    _cursor = _addObjectFromConfig(&cursor_config);
    assert(_cursor);
    
    // Store initial position for reset functionality
    _knife_initial_position = Vec3r(0.01, 0.0, 0.01);
    
    // Generate SDF for accurate distance queries
    std::cout << "[PushingSimulation] Creating SDF for knife tool..." << std::endl;
    _cursor->createSDF();
    std::cout << "[PushingSimulation] Knife tool SDF created successfully!" << std::endl;
    
    // Report actual knife dimensions
    Geometry::AABB knife_bbox = _cursor->boundingBox();
    Vec3r bbox_size = knife_bbox.max - knife_bbox.min;
    Real bbox_radius = bbox_size.norm() / 2.0;
    std::cout << "[PushingSimulation] Knife scaled to max dimension: " << _tool_radius << " m" << std::endl;
    std::cout << "[PushingSimulation] Knife bounding box size: (" 
              << bbox_size.x() << ", " << bbox_size.y() << ", " << bbox_size.z() << ") m" << std::endl;
    std::cout << "[PushingSimulation] Knife bounding box diagonal: " << bbox_radius * 2.0 << " m" << std::endl;
    
    // Note: RigidMeshObject doesn't have setColor method, so color is set in config
}

void PushingSimulation::notifyMouseButtonPressed(SimulationInput::MouseButton button, SimulationInput::MouseAction action, int modifiers)
{
    // printf("DEBUG: Mouse button event: button=%d, action=%d\n", static_cast<int>(button), static_cast<int>(action));

    // Left mouse button toggles pushing on/off
    if (button == SimulationInput::MouseButton::LEFT && action == SimulationInput::MouseAction::PRESS)
    {
        _togglePushing();
    }

    Simulation::notifyMouseButtonPressed(button, action, modifiers);
}

void PushingSimulation::notifyMouseMoved(double x, double y)
{
    // printf("DEBUG: Mouse moved: x=%.2f, y=%.2f, space_held=%d\n", x, y, 
    //        _keys_held.count(SimulationInput::Key::SPACE) ? _keys_held.at(SimulationInput::Key::SPACE) : 0);
           
    // Move cursor when spacebar is held
    if (_keys_held.count(SimulationInput::Key::SPACE) && _keys_held.at(SimulationInput::Key::SPACE) > 0)
    {
    // Reduced sensitivity for better control: less world motion per pixel
    const Real base_scaling = _tool_radius / 100.0; // Reduced from /30.0 for finer control
    const Real scaling = _pushing_enabled ? base_scaling * 0.35 : base_scaling; // Slower when pushing

        Real dx = x - _last_mouse_pos[0];
        Real dy = y - _last_mouse_pos[1];

        // Limit maximum mouse movement per frame to prevent explosive motion
        const Real max_mouse_delta = 20.0; // pixels
        dx = std::max(-max_mouse_delta, std::min(max_mouse_delta, dx));
        dy = std::max(-max_mouse_delta, std::min(max_mouse_delta, dy));

        // camera plane defined by camera up direction and camera right direction
    const Vec3r up_vec = _graphics_scene->cameraUpDirection();
    const Vec3r right_vec = _graphics_scene->cameraRightDirection();
        
    // Map screen-space mouse to world: right maps to +cameraRight, up maps to +cameraUp
    // Using +dy here so moving mouse up moves tool up in the scene
    const Vec3r offset = right_vec * dx + up_vec * dy;
    _moveCursor(offset * scaling);
    }

    _last_mouse_pos[0] = x;
    _last_mouse_pos[1] = y;
}

void PushingSimulation::notifyKeyPressed(SimulationInput::Key key, SimulationInput::KeyAction action, int modifiers)
{
    if (key == SimulationInput::Key::SPACE) {
        // printf("DEBUG: Spacebar event: action=%d\n", static_cast<int>(action));
    }
    
    // Reset knife to initial position when 'o' key is pressed
    if (key == SimulationInput::Key::O && action == SimulationInput::KeyAction::PRESS)
    {
        if (_cursor) {
            _cursor->setPosition(_knife_initial_position);
            std::cout << "[PushingSimulation] Knife reset to initial position: (" 
                      << _knife_initial_position.x() << ", " 
                      << _knife_initial_position.y() << ", " 
                      << _knife_initial_position.z() << ")" << std::endl;
        }
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
    // Reduced scroll sensitivity for better depth control
    const Real base_scaling = _tool_radius / 3.0; // Reduced from /1.0 for finer control
    const Real scaling = _pushing_enabled ? base_scaling * 0.5 : base_scaling; // Slower when pushing

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
    // Move the tool cursor - force update even if it's marked as fixed
    // This allows kinematic control (we move it, but it still has collision)
    const Vec3r current_position = _cursor->position();
    const Vec3r new_position = current_position + dp;
    
    // Directly set position, bypassing the fixed check
    // This is necessary for kinematic rigid bodies
    _cursor->setPosition(new_position);
    
    // If setPosition didn't work (because fixed=true), access the member directly
    // Note: This is a workaround - ideally we'd have a "kinematic" rigid body type
    if (_cursor->position() == current_position && dp.norm() > 1e-10) {
        // Position didn't update, probably because it's fixed
        // We need to force the update for kinematic control
        std::cout << "[PushingSimulation] Warning: Knife is fixed, position update may not work properly" << std::endl;
    }
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
    
    // For mesh-based tools, we scale the mesh instead of setting radius
    // Note: Scaling a mesh is complex and requires mesh reconstruction
    // For now, we just update the internal radius value for interaction distance
    // TODO: Implement proper mesh scaling if needed
    if (radius_change != 0.0) {
        std::cout << "[PushingSimulation] Tool radius updated to: " << _tool_radius << " m" << std::endl;
        std::cout << "[PushingSimulation] Note: Visual mesh size not changed (requires mesh scaling)" << std::endl;
    }

    // Apply pushing forces if pushing is enabled
    if (_pushing_enabled)
    {
        // Check if knife is cutting adhesion constraints
        _checkKnifeAdhesionInterference();
        
        // Apply pushing forces for tissue interaction
        _applyPushingForces();
    }

    Simulation::_timeStep();
}

void PushingSimulation::_togglePushing()
{
    _pushing_enabled = !_pushing_enabled;
    
    if (_pushing_enabled)
    {
        // printf("DEBUG: Pushing ENABLED at tool position (%.2f, %.2f, %.2f)\n", 
        //        _cursor->position().x(), _cursor->position().y(), _cursor->position().z());
        // Note: RigidSphere doesn't have setColor method
    }
    else
    {
        // printf("DEBUG: Pushing DISABLED\n");
        // Note: RigidSphere doesn't have setColor method
    }
}

void PushingSimulation::_applyPushingForces()
{
    // Get both types of XPBD mesh objects
    std::vector<std::unique_ptr<Sim::XPBDMeshObject_Base>>& xpbd_mesh_objs = _objects.template get<std::unique_ptr<Sim::XPBDMeshObject_Base>>();
    std::vector<std::unique_ptr<Sim::FirstOrderXPBDMeshObject_Base>>& fo_xpbd_mesh_objs = _objects.template get<std::unique_ptr<Sim::FirstOrderXPBDMeshObject_Base>>();
    
    const Vec3r tool_center = _cursor->position();
    int vertices_contacted = 0;
    int vertices_pushed = 0;
    
    // Get knife SDF for accurate distance queries
    const Geometry::MeshSDF* knife_sdf = _cursor->SDF();
    if (!knife_sdf) {
        std::cerr << "[PushingSimulation] ERROR: Knife SDF not available!" << std::endl;
        return;
    }

    // printf("DEBUG: === PUSHING FRAME START (Using Knife SDF) ===\n");
    // printf("DEBUG: Tool center at (%.3f, %.3f, %.3f), radius: %.3f\n", 
    //        tool_center.x(), tool_center.y(), tool_center.z(), _tool_radius);

    // IMPORTANT: Clear attachment constraints - we don't want any for pushing!
    for (auto& xpbd_mesh_obj : xpbd_mesh_objs)
    {
        xpbd_mesh_obj->clearAttachmentConstraints();
    }
    for (auto& fo_xpbd_mesh_obj : fo_xpbd_mesh_objs)
    {
        fo_xpbd_mesh_obj->clearAttachmentConstraints();
    }
    
    // Process XPBDMeshObject_Base objects
    for (auto& xpbd_mesh_obj : xpbd_mesh_objs)
    {
        // printf("DEBUG: Processing XPBDMeshObject with %d vertices\n", xpbd_mesh_obj->mesh()->numVertices());
        
        for (int v = 0; v < xpbd_mesh_obj->mesh()->numVertices(); ++v)
        {
            if (xpbd_mesh_obj->vertexFixed(v)) continue;

            const Vec3r vertex_pos = xpbd_mesh_obj->mesh()->vertex(v);
            
            // Use SDF to get signed distance (negative = inside knife, positive = outside)
            Real signed_distance = knife_sdf->evaluate(vertex_pos);
            
            // Debug: Show vertices near tool
            if (signed_distance <= _tool_radius * 0.2) // Within 20% of tool radius
            {
                vertices_contacted++;
                // printf("DEBUG: Vertex %d at signed distance %.4f\n", v, signed_distance);
            }
            
            // Apply pushing if vertex is penetrating the knife (negative distance)
            // or very close to it (small positive distance for soft contact)
            Real contact_threshold = 0.001; // 1mm soft contact zone
            if (signed_distance < contact_threshold)
            {
                Real penetration = contact_threshold - signed_distance;
                // printf("DEBUG: PENETRATION detected! Vertex %d, penetration=%.4f\n", v, penetration);
                
                // Get push direction from SDF gradient
                // SDF gradient points in direction of increasing distance (away from knife surface)
                Vec3r sdf_grad = knife_sdf->gradient(vertex_pos);
                Vec3r push_direction;
                
                if (sdf_grad.norm() < 1e-6) // Handle zero gradient
                {
                    // Fallback: push away from tool center
                    Vec3r displacement = vertex_pos - tool_center;
                    if (displacement.norm() < 1e-6) {
                        push_direction = Vec3r(0, 0, 1); // Push upward
                    } else {
                        push_direction = displacement.normalized();
                    }
                    // printf("DEBUG: Zero gradient - using fallback direction\n");
                }
                else
                {
                    push_direction = sdf_grad.normalized();
                }
                
                // Calculate push offset - stronger for deeper penetration
                Real push_magnitude = penetration * (_push_stiffness / 600.0);
                push_magnitude = std::min(push_magnitude, _tool_radius * 0.3);
                
                // Apply damping based on vertex velocity to prevent oscillations
                Vec3r vertex_velocity = xpbd_mesh_obj->vertexVelocity(v);
                Real velocity_along_push = vertex_velocity.dot(push_direction);
                
                // Reduce push if vertex is already moving in push direction (damping)
                // This prevents overshoot and oscillations
                if (velocity_along_push > 0) {
                    Real damping_reduction = _push_damping * velocity_along_push * dt();
                    push_magnitude = std::max(0.0, push_magnitude - damping_reduction);
                }
                
                Vec3r push_offset = push_direction * push_magnitude;
                
                // printf("DEBUG: Pushing vertex %d by offset (%.6f, %.6f, %.6f), magnitude=%.6f\n", 
                //        v, push_offset.x(), push_offset.y(), push_offset.z(), push_magnitude);
                
                // DIRECT VERTEX DISPLACEMENT - NO ATTACHMENT CONSTRAINTS!
                Vec3r new_position = vertex_pos + push_offset;
                xpbd_mesh_obj->mesh()->setVertex(v, new_position);
                
                vertices_pushed++;
                
                // if (vertices_pushed <= 3) // Detailed debug for first few
                // {
                //     printf("DEBUG: Vertex %d: (%.3f,%.3f,%.3f) -> (%.3f,%.3f,%.3f)\n", 
                //            v, vertex_pos.x(), vertex_pos.y(), vertex_pos.z(),
                //            new_position.x(), new_position.y(), new_position.z());
                // }
            }
        }
    }
    
    // Process FirstOrderXPBDMeshObject_Base objects
    for (auto& fo_xpbd_mesh_obj : fo_xpbd_mesh_objs)
    {
        // printf("DEBUG: Processing FirstOrderXPBDMeshObject with %d vertices\n", fo_xpbd_mesh_obj->mesh()->numVertices());
        
        for (int v = 0; v < fo_xpbd_mesh_obj->mesh()->numVertices(); ++v)
        {
            if (fo_xpbd_mesh_obj->vertexFixed(v)) continue;

            const Vec3r vertex_pos = fo_xpbd_mesh_obj->mesh()->vertex(v);
            
            // Use SDF to get signed distance (negative = inside knife, positive = outside)
            Real signed_distance = knife_sdf->evaluate(vertex_pos);
            
            // Debug: Show vertices near tool
            if (signed_distance <= _tool_radius * 0.2) // Within 20% of tool radius
            {
                vertices_contacted++;
                // printf("DEBUG: FO Vertex %d at signed distance %.4f\n", v, signed_distance);
            }
            
            // Apply pushing if vertex is penetrating the knife
            Real contact_threshold = 0.001; // 1mm soft contact zone
            if (signed_distance < contact_threshold)
            {
                Real penetration = contact_threshold - signed_distance;
                // printf("DEBUG: FO PENETRATION detected! Vertex %d, penetration=%.4f\n", v, penetration);
                
                // Get push direction from SDF gradient
                // SDF gradient points in direction of increasing distance (away from knife surface)
                Vec3r sdf_grad = knife_sdf->gradient(vertex_pos);
                Vec3r push_direction;
                
                if (sdf_grad.norm() < 1e-6) // Handle zero gradient
                {
                    // Fallback: push away from tool center
                    Vec3r displacement = vertex_pos - tool_center;
                    if (displacement.norm() < 1e-6) {
                        push_direction = Vec3r(0, 0, 1); // Push upward
                    } else {
                        push_direction = displacement.normalized();
                    }
                    // printf("DEBUG: FO Zero gradient - using fallback direction\n");
                }
                else
                {
                    push_direction = sdf_grad.normalized();
                }
                
                // Calculate push offset
                Real push_magnitude = penetration * (_push_stiffness / 600.0);
                push_magnitude = std::min(push_magnitude, _tool_radius * 0.3);
                
                // Apply damping based on vertex velocity to prevent oscillations
                Vec3r vertex_velocity = fo_xpbd_mesh_obj->vertexVelocity(v);
                Real velocity_along_push = vertex_velocity.dot(push_direction);
                
                // Reduce push if vertex is already moving in push direction (damping)
                // This prevents overshoot and oscillations
                if (velocity_along_push > 0) {
                    Real damping_reduction = _push_damping * velocity_along_push * dt();
                    push_magnitude = std::max(0.0, push_magnitude - damping_reduction);
                }
                
                Vec3r push_offset = push_direction * push_magnitude;
                
                // printf("DEBUG: FO Pushing vertex %d by offset (%.6f, %.6f, %.6f), magnitude=%.6f\n", 
                //        v, push_offset.x(), push_offset.y(), push_offset.z(), push_magnitude);
                
                // DIRECT VERTEX DISPLACEMENT - NO ATTACHMENT CONSTRAINTS!
                Vec3r new_position = vertex_pos + push_offset;
                fo_xpbd_mesh_obj->mesh()->setVertex(v, new_position);
                
                vertices_pushed++;
                
                if (vertices_pushed <= 3) // Detailed debug for first few
                {
                    // printf("DEBUG: FO Vertex %d: (%.3f,%.3f,%.3f) -> (%.3f,%.3f,%.3f)\n", 
                    //        v, vertex_pos.x(), vertex_pos.y(), vertex_pos.z(),
                    //        new_position.x(), new_position.y(), new_position.z());
                }
            }
        }
    }

    // printf("DEBUG: Frame summary - Vertices contacted: %d, Vertices pushed: %d\n", vertices_contacted, vertices_pushed);
    // printf("DEBUG: === PUSHING FRAME END ===\n\n");
}

void PushingSimulation::_checkKnifeAdhesionInterference()
{
    // Get knife SDF for interference detection
    const Geometry::MeshSDF* knife_sdf = _cursor->SDF();
    if (!knife_sdf) {
        return; // No SDF available, skip interference check
    }
    
    // Define interference threshold - if knife is within this distance of attachment point, break constraint
    const Real interference_threshold = 0.003; // 3mm - knife is "cutting" the adhesion
    
    int total_broken = 0;
    
    // Check FirstOrderXPBDMeshObject objects
    std::vector<std::unique_ptr<Sim::FirstOrderXPBDMeshObject_Base>>& fo_xpbd_mesh_objs = 
        _objects.template get<std::unique_ptr<Sim::FirstOrderXPBDMeshObject_Base>>();
    
    for (auto& fo_xpbd_mesh_obj : fo_xpbd_mesh_objs)
    {
        int broken = fo_xpbd_mesh_obj->checkAndBreakConstraintsNearSDF(knife_sdf, interference_threshold);
        total_broken += broken;
    }
    
    // Check XPBDMeshObject objects
    std::vector<std::unique_ptr<Sim::XPBDMeshObject_Base>>& xpbd_mesh_objs = 
        _objects.template get<std::unique_ptr<Sim::XPBDMeshObject_Base>>();
    
    for (auto& xpbd_mesh_obj : xpbd_mesh_objs)
    {
        int broken = xpbd_mesh_obj->checkAndBreakConstraintsNearSDF(knife_sdf, interference_threshold);
        total_broken += broken;
    }
    
    // Report cutting activity (only if constraints were broken)
    if (total_broken > 0) {
        std::cout << "[PushingSimulation] Knife cut " << total_broken << " adhesion constraints" << std::endl;
    }
}

Vec3r PushingSimulation::_calculatePushTarget(const Vec3r& vertex_pos, const Vec3r& tool_center, Real tool_radius)
{
    // Direction outward from tool center
    Vec3r displacement = vertex_pos - tool_center;
    Real distance = displacement.norm();
    
    if (distance < 1e-6) // avoid division by zero - push straight up if at center
    {
        return vertex_pos + Vec3r(0, 0, tool_radius * 0.2);
    }
    
    // Calculate how far the vertex should be pushed
    Real penetration = tool_radius - distance;
    
    if (penetration <= 0)
    {
        return vertex_pos; // No pushing needed
    }
    
    // Normalize displacement to get push direction (away from tool center)
    Vec3r push_direction = displacement / distance;

    // MORE EFFECTIVE PUSHING: Scale penetration more aggressively
    Real push_distance = penetration * (_push_stiffness / 500.0); // Much more responsive (was 5000)

    // Reasonable max step per frame - allow meaningful deformation
    const Real max_step = tool_radius * 0.3; // 30% of tool radius per frame (was 5%)
    push_distance = std::min(push_distance, max_step);

    // Less restrictive force cap
    const Real force_scaled_step = _max_push_force / 100.0; // More responsive (was 10000)
    push_distance = std::min(push_distance, force_scaled_step);

    printf("DEBUG: Push calculation - penetration: %.4f, raw_distance: %.4f, final_distance: %.4f\n", 
           penetration, penetration * (_push_stiffness / 500.0), push_distance);

    return vertex_pos + push_direction * push_distance;
}

} // namespace Sim