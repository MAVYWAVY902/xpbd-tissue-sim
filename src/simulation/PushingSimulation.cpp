#include "simulation/PushingSimulation.hpp"
#include "config/simobject/RigidMeshObjectConfig.hpp"
#include "simobject/RigidMeshObject.hpp"
#include <cstdio>

namespace Sim
{

PushingSimulation::PushingSimulation(const Config::PushingSimulationConfig* config)
    : Simulation(config), _pushing_enabled(false)
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

    // Fixed-base pivot mode
    _fixed_base_mode = config->fixedBaseMode();
    _base_offset_right = config->baseOffsetRight();
    _base_offset_up = config->baseOffsetUp();
    _base_offset_forward = config->baseOffsetForward();
    _knife_shaft_length = config->knifeShaftLength();
    _knife_rest_direction_camera = config->knifeRestDirection();
    _tip_sensitivity = config->tipSensitivity();

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
    const Config::PushingSimulationConfig* pushing_config = dynamic_cast<const Config::PushingSimulationConfig*>(config());
    Vec3r knife_initial_position = pushing_config->knifePosition();
    Vec3r knife_rotation = pushing_config->knifeRotation();
    
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
        knife_rotation,                                    // initial rotation
        Vec3r(0,0,0),                                      // initial velocity
        Vec3r(0,0,0),                                      // initial angular velocity
        1.0,                                               // density
        false,                                             // collisions (we manually add to collision scene below)
        true,                                              // graphics_only (TRUE - not in physics update loop)
        true,                                              // fixed (kinematic - collision system uses StaticCollisionConstraint)
        "../resource/tools/dissector_uv.obj",      // filename
        max_size_param,                                    // max_size (uniform scaling)
        size_param,                                        // size (directional scaling)
        false,                                             // draw_points
        true,                                              // draw_edges
        true,                                              // draw_faces
        Vec4r(0.75, 0.78, 0.8, 1.0),                       // color (steel blue-gray for knife)
        std::nullopt,                                      // sdf_filename
        []() {
            Config::ObjectRenderConfig render_cfg;
            render_cfg.setMetallic(1.0);
            render_cfg.setRoughness(0.25);
            return render_cfg;
        }()                                                // render_config with metallic material
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

    // Middle mouse button toggles pushing on/off
    if (button == SimulationInput::MouseButton::MIDDLE && action == SimulationInput::MouseAction::PRESS)
    {
        _togglePushing();
    }

    Simulation::notifyMouseButtonPressed(button, action, modifiers);
}

void PushingSimulation::notifyMouseMoved(double x, double y)
{
    // Move cursor when spacebar is held
    if (_keys_held.count(SimulationInput::Key::SPACE) && _keys_held.at(SimulationInput::Key::SPACE) > 0)
    {
        Real dx = x - _last_mouse_pos[0];
        Real dy = y - _last_mouse_pos[1];

        // Limit maximum mouse movement per frame to prevent explosive motion
        const Real max_mouse_delta = 20.0; // pixels
        dx = std::max(-max_mouse_delta, std::min(max_mouse_delta, dx));
        dy = std::max(-max_mouse_delta, std::min(max_mouse_delta, dy));

        if (_fixed_base_mode)
        {
            // In fixed-base mode, accumulate tip deflection in camera-local coords
            const Real scaling = _tip_sensitivity / 500.0;
            _tip_deflection_camera[0] += dx * scaling;  // right
            _tip_deflection_camera[1] += dy * scaling;  // up
        }
        else
        {
            // Original translation mode
            const Real base_scaling = _tool_radius / 100.0;
            const Real scaling = _pushing_enabled ? base_scaling * 0.35 : base_scaling;

            const Vec3r up_vec = _graphics_scene->cameraUpDirection();
            const Vec3r right_vec = _graphics_scene->cameraRightDirection();
            const Vec3r offset = right_vec * dx + up_vec * dy;
            _moveCursor(offset * scaling);
        }
    }

    _last_mouse_pos[0] = x;
    _last_mouse_pos[1] = y;
}

void PushingSimulation::notifyKeyPressed(SimulationInput::Key key, SimulationInput::KeyAction action, int modifiers)
{
    if (key == SimulationInput::Key::SPACE) {
        // spacebar press/release tracked via _keys_held
    }
    
    // Reset knife to initial position when 'o' key is pressed
    if (key == SimulationInput::Key::O && action == SimulationInput::KeyAction::PRESS)
    {
        if (_fixed_base_mode)
        {
            _tip_deflection_camera = Vec3r::Zero();
            std::cout << "[PushingSimulation] Tip deflection reset to zero" << std::endl;
        }
        else if (_cursor)
        {
            _cursor->forceSetPosition(_knife_initial_position);
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
        // Limit scroll delta to prevent explosive motion
        const Real limited_dy = std::max(-2.0, std::min(2.0, dy));

        if (_fixed_base_mode)
        {
            // In fixed-base mode, scroll adjusts forward/back deflection
            const Real scaling = _tip_sensitivity / 5.0;
            _tip_deflection_camera[2] += limited_dy * scaling;
        }
        else
        {
            const Real base_scaling = _tool_radius / 3.0;
            const Real scaling = _pushing_enabled ? base_scaling * 0.5 : base_scaling;
            const Vec3r view_dir = _graphics_scene->cameraViewDirection();
            const Vec3r offset = view_dir * limited_dy;
            _moveCursor(offset * scaling);
        }
    }

    Simulation::notifyMouseScrolled(dx, dy);
}

void PushingSimulation::_moveCursor(const Vec3r& dp)
{
    const Vec3r new_position = _cursor->position() + dp;
    // Use forceSetPosition to prevent double-move from update() drift
    _cursor->forceSetPosition(new_position);
}

void PushingSimulation::_timeStep()
{
    // Update knife position/orientation in fixed-base pivot mode
    if (_fixed_base_mode)
    {
        _updateFixedBaseKnife();
    }

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
        // Check if knife is cutting/weakening adhesion constraints
        _checkKnifeAdhesionInterference();
    }

    Simulation::_timeStep();

    // Post-solve: project vertices out of knife SDF after solver finishes.
    // Runs AFTER solver so it doesn't fight with elastic constraints.
    // Uses a generous margin so vertices don't re-penetrate next frame.
    if (_pushing_enabled)
    {
        _postSolveProject();
    }
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
    
    // Bounding sphere radius for early rejection: vertices farther than this
    // from the knife center cannot possibly be within the 1mm contact threshold.
    const Real reject_radius = _tool_radius + 0.002; // tool radius + 2mm margin
    const Real reject_radius_sq = reject_radius * reject_radius;

    // Process XPBDMeshObject_Base objects
    for (auto& xpbd_mesh_obj : xpbd_mesh_objs)
    {
        for (int v = 0; v < xpbd_mesh_obj->mesh()->numVertices(); ++v)
        {
            if (xpbd_mesh_obj->vertexFixed(v)) continue;

            const Vec3r vertex_pos = xpbd_mesh_obj->mesh()->vertex(v);

            // Fast bounding sphere rejection — skip expensive SDF eval
            if ((vertex_pos - tool_center).squaredNorm() > reject_radius_sq) continue;

            // Use SDF to get signed distance (negative = inside knife, positive = outside)
            Real signed_distance = knife_sdf->evaluate(vertex_pos);

            if (signed_distance <= _tool_radius * 0.2)
            {
                vertices_contacted++;
            }

            // Apply pushing if vertex is penetrating or near the knife surface
            Real contact_threshold = 0.001; // 1mm soft contact zone
            if (signed_distance < contact_threshold)
            {
                // Get push direction from SDF gradient
                Vec3r sdf_grad = knife_sdf->gradient(vertex_pos);
                Vec3r push_direction;

                if (sdf_grad.norm() < 1e-6)
                {
                    Vec3r displacement = vertex_pos - tool_center;
                    if (displacement.norm() < 1e-6) {
                        push_direction = Vec3r(0, 0, 1);
                    } else {
                        push_direction = displacement.normalized();
                    }
                }
                else
                {
                    push_direction = sdf_grad.normalized();
                }

                Real push_magnitude;
                if (signed_distance < 0)
                {
                    // FULL PROJECTION: vertex is inside the tool — project to surface + margin
                    // No damping here — penetration must be fully resolved
                    push_magnitude = -signed_distance + contact_threshold;
                }
                else
                {
                    // SOFT ZONE: gentle proportional push to create a buffer zone
                    Real penetration = contact_threshold - signed_distance;
                    push_magnitude = penetration * 0.5;

                    // Only apply damping in the soft zone (not for hard penetration)
                    Vec3r vertex_velocity = xpbd_mesh_obj->vertexVelocity(v);
                    Real velocity_along_push = vertex_velocity.dot(push_direction);
                    if (velocity_along_push > 0) {
                        Real damping_reduction = _push_damping * velocity_along_push * dt();
                        push_magnitude = std::max(0.0, push_magnitude - damping_reduction);
                    }
                }

                Vec3r push_offset = push_direction * push_magnitude;
                Vec3r new_position = vertex_pos + push_offset;
                xpbd_mesh_obj->mesh()->setVertex(v, new_position);

                vertices_pushed++;
            }
        }
    }
    
    // Process FirstOrderXPBDMeshObject_Base objects
    for (auto& fo_xpbd_mesh_obj : fo_xpbd_mesh_objs)
    {
        for (int v = 0; v < fo_xpbd_mesh_obj->mesh()->numVertices(); ++v)
        {
            if (fo_xpbd_mesh_obj->vertexFixed(v)) continue;

            const Vec3r vertex_pos = fo_xpbd_mesh_obj->mesh()->vertex(v);

            // Fast bounding sphere rejection — skip expensive SDF eval
            if ((vertex_pos - tool_center).squaredNorm() > reject_radius_sq) continue;

            // Use SDF to get signed distance (negative = inside knife, positive = outside)
            Real signed_distance = knife_sdf->evaluate(vertex_pos);

            if (signed_distance <= _tool_radius * 0.2)
            {
                vertices_contacted++;
            }

            // Apply pushing if vertex is penetrating or near the knife surface
            Real contact_threshold = 0.001; // 1mm soft contact zone
            if (signed_distance < contact_threshold)
            {
                // Get push direction from SDF gradient
                Vec3r sdf_grad = knife_sdf->gradient(vertex_pos);
                Vec3r push_direction;

                if (sdf_grad.norm() < 1e-6)
                {
                    Vec3r displacement = vertex_pos - tool_center;
                    if (displacement.norm() < 1e-6) {
                        push_direction = Vec3r(0, 0, 1);
                    } else {
                        push_direction = displacement.normalized();
                    }
                }
                else
                {
                    push_direction = sdf_grad.normalized();
                }

                Real push_magnitude;
                if (signed_distance < 0)
                {
                    // FULL PROJECTION: vertex is inside the tool — project to surface + margin
                    push_magnitude = -signed_distance + contact_threshold;
                }
                else
                {
                    // SOFT ZONE: gentle proportional push
                    Real penetration = contact_threshold - signed_distance;
                    push_magnitude = penetration * 0.5;

                    Vec3r vertex_velocity = fo_xpbd_mesh_obj->vertexVelocity(v);
                    Real velocity_along_push = vertex_velocity.dot(push_direction);
                    if (velocity_along_push > 0) {
                        Real damping_reduction = _push_damping * velocity_along_push * dt();
                        push_magnitude = std::max(0.0, push_magnitude - damping_reduction);
                    }
                }

                Vec3r push_offset = push_direction * push_magnitude;
                Vec3r new_position = vertex_pos + push_offset;
                fo_xpbd_mesh_obj->mesh()->setVertex(v, new_position);

                vertices_pushed++;
            }
        }
    }

    // printf("DEBUG: Frame summary - Vertices contacted: %d, Vertices pushed: %d\n", vertices_contacted, vertices_pushed);
    // printf("DEBUG: === PUSHING FRAME END ===\n\n");
}

void PushingSimulation::_postSolveProject()
{
    const Geometry::MeshSDF* knife_sdf = _cursor->SDF();
    if (!knife_sdf) return;

    const Vec3r tool_center = _cursor->position();
    // Generous reject radius: knife bounding diagonal + margin
    const Real reject_radius = _tool_radius * 2.0 + 0.01;
    const Real reject_radius_sq = reject_radius * reject_radius;
    // Surface margin: vertices are pushed to this distance outside the SDF surface.
    // Must be large enough that the solver doesn't pull them back inside next frame.
    const Real surface_margin = 0.002; // 2mm

    auto projectVertices = [&](auto& mesh_obj) {
        for (int v = 0; v < mesh_obj->mesh()->numVertices(); ++v)
        {
            if (mesh_obj->vertexFixed(v)) continue;

            const Vec3r vertex_pos = mesh_obj->mesh()->vertex(v);
            if ((vertex_pos - tool_center).squaredNorm() > reject_radius_sq) continue;

            Real signed_distance = knife_sdf->evaluate(vertex_pos);

            // Project vertices that are inside the tool OR within the margin zone.
            // This prevents vertices from hovering just at the surface and re-penetrating.
            if (signed_distance < surface_margin)
            {
                Vec3r sdf_grad = knife_sdf->gradient(vertex_pos);
                Vec3r push_direction;

                if (sdf_grad.norm() < 1e-6)
                {
                    Vec3r displacement = vertex_pos - tool_center;
                    push_direction = (displacement.norm() < 1e-6)
                        ? Vec3r(0, 0, 1)
                        : displacement.normalized();
                }
                else
                {
                    push_direction = sdf_grad.normalized();
                }

                // Hard projection: move vertex to surface + margin
                Real push_magnitude = -signed_distance + surface_margin;
                Vec3r new_position = vertex_pos + push_direction * push_magnitude;
                mesh_obj->mesh()->setVertex(v, new_position);
            }
        }
    };

    auto& xpbd_mesh_objs = _objects.template get<std::unique_ptr<Sim::XPBDMeshObject_Base>>();
    for (auto& obj : xpbd_mesh_objs) projectVertices(obj);

    auto& fo_xpbd_mesh_objs = _objects.template get<std::unique_ptr<Sim::FirstOrderXPBDMeshObject_Base>>();
    for (auto& obj : fo_xpbd_mesh_objs) projectVertices(obj);
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
    
    // Report cutting activity (disabled for performance)
    // if (total_broken > 0)
    //     std::cout << "[PushingSimulation] Knife cut " << total_broken << " adhesion constraints" << std::endl;
}

Vec3r PushingSimulation::_computeBasePosition() const
{
    const Vec3r cam_pos     = _graphics_scene->cameraPosition();
    const Vec3r cam_right   = _graphics_scene->cameraRightDirection();
    const Vec3r cam_up      = _graphics_scene->cameraUpDirection();
    const Vec3r cam_forward = _graphics_scene->cameraViewDirection();

    return cam_pos
         + cam_right   * _base_offset_right
         + cam_up      * _base_offset_up
         + cam_forward * _base_offset_forward;
}

Vec3r PushingSimulation::_computeTipPosition(const Vec3r& base_world) const
{
    // Combine rest direction with user deflection in camera-local space
    Vec3r tip_dir_camera = _knife_rest_direction_camera + _tip_deflection_camera;

    // Normalize (guard against zero)
    Real len = tip_dir_camera.norm();
    if (len < 1e-8)
        tip_dir_camera = Vec3r(0, 0, 1);  // fallback to forward
    else
        tip_dir_camera /= len;

    // Transform camera-local direction to world space
    const Vec3r cam_right   = _graphics_scene->cameraRightDirection();
    const Vec3r cam_up      = _graphics_scene->cameraUpDirection();
    const Vec3r cam_forward = _graphics_scene->cameraViewDirection();

    Vec3r tip_dir_world = cam_right   * tip_dir_camera[0]
                        + cam_up      * tip_dir_camera[1]
                        + cam_forward * tip_dir_camera[2];

    tip_dir_world.normalize();

    return base_world + tip_dir_world * _knife_shaft_length;
}

Vec4r PushingSimulation::_computeKnifeOrientation(const Vec3r& base_world, const Vec3r& tip_world) const
{
    // Knife mesh long axis is +X, so we need rotation from +X to desired direction
    Vec3r desired_dir = (tip_world - base_world);
    Real dir_len = desired_dir.norm();
    if (dir_len < 1e-8)
        return Vec4r(0, 0, 0, 1);  // identity
    desired_dir /= dir_len;

    const Vec3r from(1, 0, 0);  // knife mesh long axis
    Real dot = from.dot(desired_dir);

    // Handle near-parallel case (already aligned)
    if (dot > 0.999999)
        return Vec4r(0, 0, 0, 1);  // identity

    // Handle anti-parallel case (180 degree rotation about Y)
    if (dot < -0.999999)
        return Vec4r(0, 1, 0, 0);  // 180° about Y axis

    // Shortest-arc quaternion: q = (cross, 1 + dot), then normalize
    Vec3r cross = from.cross(desired_dir);
    Real w = 1.0 + dot;

    Vec4r q(cross[0], cross[1], cross[2], w);
    q.normalize();
    return q;
}

void PushingSimulation::_updateFixedBaseKnife()
{
    if (!_cursor) return;

    Vec3r base = _computeBasePosition();
    Vec3r tip  = _computeTipPosition(base);
    Vec4r quat = _computeKnifeOrientation(base, tip);

    _cursor->forceSetPosition(base);
    _cursor->forceSetOrientation(quat);
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