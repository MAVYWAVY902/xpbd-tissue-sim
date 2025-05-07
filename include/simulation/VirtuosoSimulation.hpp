#ifndef __VIRTUOSO_SIMULATION_HPP
#define __VIRTUOSO_SIMULATION_HPP

#include "simulation/Simulation.hpp"
#include "simobject/VirtuosoRobot.hpp"
#include "simobject/VirtuosoArm.hpp"
#include "simobject/RigidMeshObject.hpp"
#include "simobject/RigidPrimitives.hpp"
#include "simobject/XPBDMeshObject.hpp"

#include "config/VirtuosoSimulationConfig.hpp"

#include "haptics/HapticDeviceManager.hpp"

#include <map>

namespace Sim
{

class VirtuosoSimulation : public Simulation
{
    public:
    VirtuosoSimulation(const VirtuosoSimulationConfig* config);

    virtual std::string type() const override { return "VirtuosoSimulation"; }

    virtual void setup() override;

    virtual void notifyKeyPressed(int key, int action, int modifiers) override;

    virtual void notifyMouseButtonPressed(int button, int action, int modifiers) override;

    virtual void notifyMouseMoved(double x, double y) override;

    virtual void notifyMouseScrolled(double dx, double dy) override;

    const VirtuosoRobot* virtuosoRobot() const { return _virtuoso_robot; }
    const VirtuosoArm* activeArm() const { return _active_arm; }
    const Vec3r activeTipPosition() const { return _tip_cursor->position(); }

    protected:

    void _updateGraphics() override;
    
    void _timeStep() override;

    void _moveCursor(const Vec3r& dp);

    protected:

    VirtuosoRobot* _virtuoso_robot; // the Virtuoso robot (includes both arms)
    VirtuosoArm* _active_arm;       // whichever arm is being actively controlled (assuming only one input device)

    
    SimulationInputDevice _input_device;    // the type of input device used (Keyboard, Mouse, or Haptic)

    RigidSphere* _tip_cursor;       // spherical object for visualizing grasp area 

    /** MOUSE INPUT */
    Vec2r _last_mouse_pos;    // tracks the last mouse position (used in when mouse input is used to control the arms)

    /** HAPTIC INPUT */
    Vec3r _last_haptic_pos;   // tracks the last haptic posiion
    std::unique_ptr<HapticDeviceManager> _haptic_device_manager;

    /** KEYBOARD INPUT */
    constexpr static double IT_ROT_RATE = 3; // rad/s
    constexpr static double IT_TRANS_RATE = 0.005; // m/s
    constexpr static double OT_ROT_RATE = 3; // rad/s
    constexpr static double OT_TRANS_RATE = 0.005; // m/s

    std::map<int, bool> _keys_held;     // tracks which keys are held (from a pre-defined set of keys)
};

} // namespace Sim

#endif // __VIRTUOSO_SIMULATION_HPP