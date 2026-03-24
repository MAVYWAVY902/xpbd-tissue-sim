#include "simulation/PushingSimulation.hpp"
#include "config/simobject/RigidMeshObjectConfig.hpp"
#include "simobject/RigidMeshObject.hpp"
#include "utils/GeometryUtils.hpp"
#include <cstdio>
#include <unordered_set>

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
        false,                                             // collisions (added to collision scene MANUALLY below)
        true,                                              // graphics_only (TRUE — keeps mesh stable, no update() drift)
        true,                                              // fixed (kinematic — position set manually via forceSetPosition)
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

    // Manually add knife to collision scene.
    // Can't use collisions=true + graphics_only=false because RigidMeshObject::update()
    // accumulates floating-point drift on the mesh vertices (breaks rendering).
    // Instead: graphics_only=true (stable mesh) + manually add to collision scene.
    // CollisionScene uses the SDF (which auto-tracks _p via globalToBody), not mesh vertices.
    _collision_scene->addObject(_cursor);
    std::cout << "[PushingSimulation] Knife added to CollisionScene manually (SDF 256^3)." << std::endl;

    // Report actual knife dimensions
    Geometry::AABB knife_bbox = _cursor->boundingBox();
    Vec3r bbox_size = knife_bbox.max - knife_bbox.min;
    Real bbox_radius = bbox_size.norm() / 2.0;
    std::cout << "[PushingSimulation] Knife scaled to max dimension: " << _tool_radius << " m" << std::endl;
    std::cout << "[PushingSimulation] Knife bounding box size: ("
              << bbox_size.x() << ", " << bbox_size.y() << ", " << bbox_size.z() << ") m" << std::endl;
    std::cout << "[PushingSimulation] Knife bounding box diagonal: " << bbox_radius * 2.0 << " m" << std::endl;

    // Compute blade geometry in body frame for plane-based collision
    {
        const Geometry::Mesh* knife_mesh = _cursor->mesh();
        _blade_body_min = Vec3r(1e10, 1e10, 1e10);
        _blade_body_max = Vec3r(-1e10, -1e10, -1e10);
        for (int i = 0; i < knife_mesh->numVertices(); ++i)
        {
            Vec3r v_body = _cursor->globalToBody(knife_mesh->vertex(i));
            _blade_body_min = _blade_body_min.cwiseMin(v_body);
            _blade_body_max = _blade_body_max.cwiseMax(v_body);
        }
        _blade_half_thickness = (_blade_body_max.y() - _blade_body_min.y()) / 2.0;

        // Reject radius = max distance from body origin to any bounding box corner + margin
        // The blade mesh is NOT centered at origin (X offset ~0.047), so we must use
        // the actual max distance, not half the extent.
        Vec3r abs_min = _blade_body_min.cwiseAbs();
        Vec3r abs_max = _blade_body_max.cwiseAbs();
        Vec3r furthest = abs_min.cwiseMax(abs_max);
        Real max_dist = furthest.norm() + 0.01; // 10mm margin
        _blade_reject_radius_sq = max_dist * max_dist;

        std::cout << "[PushingSimulation] Blade body-frame bounds: ("
                  << _blade_body_min.transpose() << ") to (" << _blade_body_max.transpose() << ")\n"
                  << "[PushingSimulation] Blade half-thickness: " << (_blade_half_thickness * 1000) << "mm\n"
                  << "[PushingSimulation] Blade reject radius: " << (max_dist * 1000) << "mm\n";
    }
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
            // Original translation mode — reduced sensitivity for finer control
            const Real base_scaling = _tool_radius / 500.0;
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
            const Real base_scaling = _tool_radius / 15.0;
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
    _cursor->forceSetPosition(new_position);
}

void PushingSimulation::_timeStep()
{
    // PER-STEP motion clamping: limit how far knife can move between physics steps.
    // Mouse events accumulate between steps; this clamps the TOTAL displacement.
    // Prevents tunneling even with fast mouse movement.
    if (_cursor && _knife_prev_initialized)
    {
        const Real max_step_dist = 0.003;  // 3mm max per physics step (was 0.8mm — too jerky)
        Vec3r knife_delta = _cursor->position() - _knife_prev_pos;
        Real delta_norm = knife_delta.norm();
        if (delta_norm > max_step_dist)
        {
            Vec3r clamped_pos = _knife_prev_pos + knife_delta * (max_step_dist / delta_norm);
            _cursor->forceSetPosition(clamped_pos);
        }
    }
    // Save knife state for next step's clamping and CCD
    if (_cursor)
    {
        if (!_knife_prev_initialized)
        {
            _knife_prev_pos = _cursor->position();
            _knife_prev_orient = _cursor->orientation();
            _knife_prev_initialized = true;
        }
    }

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

    // Debug: count collision constraints on tumor
    {
        static int dbg = 0;
        dbg++;
        if (dbg % 500 == 0)
        {
            auto& fo_objs = _objects.template get<std::unique_ptr<Sim::FirstOrderXPBDMeshObject_Base>>();
            for (auto& obj : fo_objs)
            {
                // Check how many vertices are inside knife SDF after solve
                const Geometry::MeshSDF* knife_sdf = _cursor ? _cursor->SDF() : nullptr;
                if (!knife_sdf) continue;
                int inside = 0;
                for (int v = 0; v < obj->mesh()->numVertices(); v++)
                {
                    Real d = knife_sdf->evaluate(obj->mesh()->vertex(v));
                    if (d < 0) inside++;
                }
                std::cout << "[PostSolve] frame=" << dbg
                          << " tumor_verts_inside_knife=" << inside
                          << std::endl;
            }
        }
    }

    // Update knife previous state for next step's motion clamping
    if (_cursor)
    {
        _knife_prev_pos = _cursor->position();
        _knife_prev_orient = _cursor->orientation();
    }
}

void PushingSimulation::_togglePushing()
{
    _pushing_enabled = !_pushing_enabled;
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

void PushingSimulation::_addKnifeCollisionConstraints()
{
    // Create SDF-based collision constraints for the knife, added to the XPBD solver.
    // Called every physics step so constraints are always fresh (no stale surface_point).
    // Uses SDF for shape-accurate detection + body-frame Y for stable normal.
    if (!_cursor) return;

    const Geometry::MeshSDF* knife_sdf = _cursor->SDF();
    if (!knife_sdf) return;

    const Vec3r knife_pos = _cursor->position();
    const Real contact_threshold = 0.001;  // 1mm influence zone outside blade surface

    // Blade normal in global coords
    const Vec3r origin_global = _cursor->bodyToGlobal(Vec3r::Zero());
    Vec3r blade_normal_global = _cursor->bodyToGlobal(Vec3r(0, 1, 0)) - origin_global;
    blade_normal_global.normalize();

    static int debug_frame = 0;
    debug_frame++;
    int total_bsphere = 0;
    int total_in_threshold = 0;
    int total_inside = 0;       // SDF < 0
    int total_near = 0;         // 0 <= SDF < threshold
    Real min_sdf = 1e10;
    Real max_sdf_constrained = -1e10;

    auto processObject = [&](auto& mesh_obj) {
        Geometry::Mesh* mesh = mesh_obj->mesh();

        for (int fi = 0; fi < mesh->numFaces(); ++fi)
        {
            const Eigen::Vector3i face = mesh->face(fi);

            // Skip faces with fixed vertices
            if (mesh_obj->vertexFixed(face[0]) ||
                mesh_obj->vertexFixed(face[1]) ||
                mesh_obj->vertexFixed(face[2]))
                continue;

            const Vec3r p1 = mesh->vertex(face[0]);
            const Vec3r p2 = mesh->vertex(face[1]);
            const Vec3r p3 = mesh->vertex(face[2]);
            const Vec3r centroid = (p1 + p2 + p3) / 3.0;

            // Quick rejection on centroid
            if ((centroid - knife_pos).squaredNorm() > _blade_reject_radius_sq) continue;

            total_bsphere++;

            // Check SDF at centroid — catches triangle-level penetration
            // even when all 3 vertices are outside the blade
            Real signed_distance = knife_sdf->evaluate(centroid);
            if (signed_distance < min_sdf) min_sdf = signed_distance;

            if (signed_distance > contact_threshold) continue;

            total_in_threshold++;
            if (signed_distance < 0) total_inside++;
            else total_near++;

            // Determine push direction from centroid body-frame Y
            Vec3r centroid_body = _cursor->globalToBody(centroid);
            Real side = (centroid_body.y() >= 0) ? 1.0 : -1.0;
            Vec3r collision_normal = blade_normal_global * side;

            // Surface point at blade surface (or at centroid if outside)
            Real correction = -signed_distance;
            if (correction < 0) correction = 0;
            Vec3r surface_point = centroid + collision_normal * correction;

            // Use centroid barycentric coords (1/3, 1/3, 1/3)
            // Force distributed evenly across all 3 vertices of the face
            const Real one_third = 1.0 / 3.0;
            mesh_obj->addStaticCollisionConstraint(
                nullptr, surface_point, collision_normal, fi, one_third, one_third, one_third);
        }
    };

    auto& xpbd_objs = _objects.template get<std::unique_ptr<Sim::XPBDMeshObject_Base>>();
    auto& fo_xpbd_objs = _objects.template get<std::unique_ptr<Sim::FirstOrderXPBDMeshObject_Base>>();
    for (auto& obj : xpbd_objs) processObject(obj);
    for (auto& obj : fo_xpbd_objs) processObject(obj);

    if (debug_frame % 500 == 0 && total_bsphere > 0)
    {
        std::cout << "[KnifeConstraint] frame=" << debug_frame
                  << " bsphere=" << total_bsphere
                  << " constrained=" << total_in_threshold
                  << " (inside=" << total_inside
                  << " near=" << total_near << ")"
                  << " min_sdf=" << (min_sdf * 1000) << "mm"
                  << " knife=(" << knife_pos.x() << "," << knife_pos.y() << "," << knife_pos.z() << ")"
                  << std::endl;
    }
}

void PushingSimulation::_onPostCollisionDetection()
{
    // Knife collision handled by standard CollisionScene.
    // No custom constraints needed — Frank-Wolfe + SDF 256³ gradient handles it.
}

void PushingSimulation::_bladeCollisionCheck()
{
    // SDF-based collision check: runs EVERY physics step.
    //
    // The dissector mesh IS watertight (verified: 0 boundary edges, Euler=2).
    // SDF inside/outside classification is reliable.
    // Combined with per-step motion clamping (0.8mm/step), prevents tunneling.
    //
    // ONE-SIDED: only pushes vertices with SDF < 0 (inside blade).
    // When knife moves away, SDF > 0 → no projection → no sticking.
    if (!_cursor) return;

    const Geometry::MeshSDF* knife_sdf = _cursor->SDF();
    if (!knife_sdf) return;

    const Vec3r knife_pos = _cursor->position();
    // Larger margin so elastic constraints can't pull vertex back inside next step.
    // "Proxy pop-out" approach from Triangle-Proxy CCD literature.
    const Real surface_margin = 0.0015;  // 1.5mm outside blade surface

    static int debug_frame = 0;
    debug_frame++;
    int total_checked = 0;
    int total_projected = 0;

    auto processMesh = [&](auto& mesh_obj) {
        Geometry::Mesh* mesh = mesh_obj->mesh();

        for (int v = 0; v < mesh->numVertices(); ++v)
        {
            if (mesh_obj->vertexFixed(v)) continue;

            const Vec3r x = mesh->vertex(v);

            // Quick bounding-sphere rejection
            if ((x - knife_pos).squaredNorm() > _blade_reject_radius_sq) continue;

            total_checked++;

            // SDF: is vertex inside the blade? (watertight mesh → reliable)
            Real signed_distance = knife_sdf->evaluate(x);
            if (signed_distance >= 0) continue;  // outside → ONE-SIDED, no action

            // Inside blade → project out along body-frame Y axis
            Vec3r v_body = _cursor->globalToBody(x);
            Real side = (v_body.y() >= 0) ? 1.0 : -1.0;

            // Blade normal in global coords
            Vec3r blade_normal = _cursor->bodyToGlobal(Vec3r(0, side, 0))
                               - _cursor->bodyToGlobal(Vec3r::Zero());
            blade_normal.normalize();

            // Project to blade surface + margin
            Vec3r corrected = x + blade_normal * (-signed_distance + surface_margin);
            mesh->setVertex(v, corrected);
            mesh_obj->setVertexPreviousPosition(v, corrected);
            total_projected++;
        }
    };

    auto& xpbd_objs = _objects.template get<std::unique_ptr<Sim::XPBDMeshObject_Base>>();
    auto& fo_xpbd_objs = _objects.template get<std::unique_ptr<Sim::FirstOrderXPBDMeshObject_Base>>();
    for (auto& obj : xpbd_objs) processMesh(obj);
    for (auto& obj : fo_xpbd_objs) processMesh(obj);

    if (debug_frame % 500 == 0 && total_checked > 0)
    {
        std::cout << "[BladeCheck] frame=" << debug_frame
                  << " checked=" << total_checked
                  << " projected=" << total_projected
                  << std::endl;
    }
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