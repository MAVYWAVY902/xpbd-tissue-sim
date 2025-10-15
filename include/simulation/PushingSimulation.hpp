#pragma once

#include "simulation/Simulation.hpp"
#include "config/simulation/PushingSimulationConfig.hpp"
#include "simobject/RigidPrimitives.hpp"

namespace Sim
{

/**
 * @brief A simulation for pushing deformable objects with a spherical tool tip
 * 
 * This simulation allows the user to control a spherical tool tip and apply pushing forces
 * to deformable mesh objects. The tool tip can be moved with mouse + spacebar controls,
 * and pushing forces are applied when the tool tip intersects with the deformable object.
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

private:
    /// @brief move the tool cursor by a given displacement
    void _moveCursor(const Vec3r& dp);

    /// @brief apply pushing forces to vertices within the tool radius
    void _applyPushingForces();

    /// @brief toggle pushing on/off
    void _togglePushing();

    /// @brief calculate pushing target position for a vertex based on penetration depth
    Vec3r _calculatePushTarget(const Vec3r& vertex_pos, const Vec3r& tool_center, Real tool_radius);

private:
    Real _tool_radius;                    ///< radius of the spherical tool tip
    Real _push_stiffness;                 ///< stiffness coefficient for pushing forces
    Real _max_push_force;                 ///< maximum force that can be applied
    bool _fix_min_z;                      ///< whether to fix vertices at minimum z coordinate
    bool _pushing_enabled;                ///< whether pushing is currently enabled
    
    std::map<SimulationInput::Key, int> _keys_held;  ///< map of currently held keys
    std::array<double, 2> _last_mouse_pos;           ///< last mouse position
    
    Sim::RigidSphere* _cursor;           ///< visual representation of the tool tip
};

} // namespace Sim