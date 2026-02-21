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

    Sim::RigidMeshObject* _cursor = nullptr;   ///< visual representation of the knife tool

private:
    /// @brief move the tool cursor by a given displacement
    void _moveCursor(const Vec3r& dp);

    /// @brief apply pushing forces to vertices within the tool radius
    void _applyPushingForces();

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
};

} // namespace Sim