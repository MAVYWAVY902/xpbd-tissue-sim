#pragma once

#include "simulation/Simulation.hpp"
#include "config/simulation/PushingSimulationConfig.hpp"
#include "simobject/RigidPrimitives.hpp"
#include <vector>

namespace Sim
{
    class RigidMeshObject;

/**
 * @brief A simulation for pushing deformable objects with a knife tool
 * 
 * This simulation allows the user to control a knife tool (mesh-based) and apply pushing forces
 * to deformable mesh objects. The tool can be moved with mouse + spacebar controls,
 * and pushing forces are applied when the tool intersects with the deformable object.
 */
class PushingSimulation : public Simulation
{
public:
    PushingSimulation(const Config::PushingSimulationConfig* config);

    void setup() override;

    /// @brief called when a mouse button is pressed
    void notifyMouseButtonPressed(SimulationInput::MouseButton button, SimulationInput::MouseAction action, int modifiers) override;
    
    /// @brief called when the mouse moves
    void notifyMouseMoved(double x, double y) override;

    /// @brief called when a key is pressed
    void notifyKeyPressed(SimulationInput::Key key, SimulationInput::KeyAction action, int modifiers) override;

    /// @brief called when the mouse wheel is scrolled
    void notifyMouseScrolled(double dx, double dy) override;

protected:
    void _timeStep() override;
    void _onPostCollisionDetection() override;

    Sim::RigidMeshObject* _cursor = nullptr;   ///< visual representation of the knife tool

    /// @brief Enable/disable pushing (for subclasses like HapticDissectionSimulation)
    void _setPushingEnabled(bool enabled) { _pushing_enabled = enabled; }

    // Fixed-base pivot mode (accessible by HapticDissectionSimulation)
    bool _fixed_base_mode = false;
    Real _base_offset_right = 0.03;
    Real _base_offset_up = -0.04;
    Real _base_offset_forward = 0.08;
    Real _knife_shaft_length = 0.06;
    Vec3r _knife_rest_direction_camera = Vec3r(0, 0, 1);
    Real _tip_sensitivity = 0.5;
    Vec3r _tip_deflection_camera = Vec3r::Zero();  ///< accumulated user input in camera-local coords

private:
    /// @brief move the tool cursor by a given displacement
    void _moveCursor(const Vec3r& dp);

    /// @brief compute the base (pivot) position in world coords from camera frame
    Vec3r _computeBasePosition() const;

    /// @brief compute the tip position in world coords given base position
    Vec3r _computeTipPosition(const Vec3r& base_world) const;

    /// @brief compute knife orientation quaternion from base→tip direction
    Vec4r _computeKnifeOrientation(const Vec3r& base_world, const Vec3r& tip_world) const;

    /// @brief update knife position and orientation in fixed-base pivot mode
    void _updateFixedBaseKnife();

    /// @brief apply pushing forces to vertices within the tool radius
    void _applyPushingForces();

    /// @brief hard-project penetrating vertices to tool surface (post-solve, no damping)
    void _postSolveProject();

    /// @brief check if knife interferes with adhesion constraints and mark them for breaking
    void _checkKnifeAdhesionInterference();

    /// @brief toggle pushing on/off
    void _togglePushing();

    /// @brief calculate pushing target position for a vertex based on penetration depth
    Vec3r _calculatePushTarget(const Vec3r& vertex_pos, const Vec3r& tool_center, Real tool_radius);

public:
    /// @brief Get knife SDF for constraint interference checking (static access)
    const Geometry::MeshSDF* getKnifeSDF() const { return _cursor ? _cursor->SDF() : nullptr; }

private:
    // Upper bound on how many vertices we will try to push per step
    static constexpr int kMaxPushedVertices = 32;

    Real _tool_radius;                    ///< effective radius for tool scaling and interaction
    Real _push_stiffness;                 ///< stiffness coefficient for pushing forces
    Real _push_damping;                   ///< damping coefficient to prevent oscillations (0-1)
    Real _max_push_force;                 ///< maximum force that can be applied
    bool _fix_min_z;                      ///< whether to fix vertices at minimum z coordinate
    bool _fix_max_z;                      ///< whether to fix vertices at maximum z coordinate
    bool _pushing_enabled;                ///< whether pushing is currently enabled
    Real _knife_scale_x;                  ///< knife X-axis scale (-1 = use uniform scaling)
    Real _knife_scale_y;                  ///< knife Y-axis scale (-1 = use uniform scaling)
    Real _knife_scale_z;                  ///< knife Z-axis scale (-1 = use uniform scaling)
    
    std::map<SimulationInput::Key, int> _keys_held;  ///< map of currently held keys
    std::array<double, 2> _last_mouse_pos;           ///< last mouse position
    
    Vec3r _knife_initial_position;       ///< initial position of the knife for reset functionality

    // Storage for per-vertex push targets to ensure stable pointer lifetimes
    std::vector<Vec3r> _push_targets;

    // Blade geometry in body frame (computed once at setup)
    Vec3r _blade_body_min;               ///< blade bounding box min in body frame
    Vec3r _blade_body_max;               ///< blade bounding box max in body frame
    Real  _blade_half_thickness{0.0};    ///< half the blade Y extent
    Real  _blade_reject_radius_sq{0.0};  ///< squared bounding sphere radius for early rejection
};

} // namespace Sim