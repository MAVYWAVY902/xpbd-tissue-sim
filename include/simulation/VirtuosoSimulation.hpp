#ifndef __VIRTUOSO_SIMULATION_HPP
#define __VIRTUOSO_SIMULATION_HPP

#include "simulation/Simulation.hpp"
#include "simobject/VirtuosoRobot.hpp"
#include "simobject/VirtuosoArm.hpp"
#include "simobject/RigidMeshObject.hpp"
#include "simobject/RigidPrimitives.hpp"
#include "simobject/XPBDMeshObject.hpp"

#include "config/simulation/VirtuosoSimulationConfig.hpp"

#include "haptics/HapticDeviceManager.hpp"

#include <map>
#include <atomic>

namespace Sim
{

class VirtuosoSimulation : public Simulation
{
    public:
    VirtuosoSimulation(const Config::VirtuosoSimulationConfig* config);

    virtual std::string type() const override { return "VirtuosoSimulation"; }

    virtual void setup() override;

    virtual void notifyKeyPressed(SimulationInput::Key key, SimulationInput::KeyAction action, int modifiers) override;

    virtual void notifyMouseButtonPressed(SimulationInput::MouseButton button, SimulationInput::MouseAction action, int modifiers) override;

    virtual void notifyMouseMoved(double x, double y) override;

    virtual void notifyMouseScrolled(double dx, double dy) override;

    const VirtuosoRobot* virtuosoRobot() const { return _virtuoso_robot; }
    const VirtuosoArm* activeArm() const { return _active_arm; }
    const Vec3r activeTipPosition() const { return _tip_cursor->position(); }

    void setArm1JointState(double ot_rot, double ot_trans, double it_rot, double it_trans, int tool);
    void setArm2JointState(double ot_rot, double ot_trans, double it_rot, double it_trans, int tool);

    protected:

    struct VirtuosoArmJointState
    {
        double outer_tube_rotation;
        double outer_tube_translation;
        double inner_tube_rotation;
        double inner_tube_translation;
        int tool;

        VirtuosoArmJointState(double ot_rot, double ot_trans, double it_rot, double it_trans, int tool_)
            : outer_tube_rotation(ot_rot), outer_tube_translation(ot_trans), inner_tube_rotation(it_rot), inner_tube_translation(it_trans), tool(tool_)
        {}

        VirtuosoArmJointState()
            : outer_tube_rotation(0.0), outer_tube_translation(0.0), inner_tube_rotation(0.0), inner_tube_translation(0.0), tool(0)
        {}
    };

    void _updateGraphics() override;
    
    void _timeStep() override;

    void _moveCursor(const Vec3r& dp);

    protected:

    VirtuosoRobot* _virtuoso_robot; // the Virtuoso robot (includes both arms)
    VirtuosoArm* _active_arm;       // whichever arm is being actively controlled (assuming only one input device)

    std::mutex _arm1_joint_state_mtx;
    std::mutex _arm2_joint_state_mtx;
    std::atomic<bool> _has_new_arm1_joint_state;
    std::atomic<bool> _has_new_arm2_joint_state;
    VirtuosoArmJointState _new_arm1_joint_state;
    VirtuosoArmJointState _new_arm2_joint_state;

    
    SimulationInput::Device _input_device;    // the type of input device used (Keyboard, Mouse, or Haptic)

    RigidSphere* _tip_cursor;       // spherical object for visualizing grasp area 

    /** MOUSE INPUT */
    Vec2r _last_mouse_pos;    // tracks the last mouse position (used in when mouse input is used to control the arms)

    /** HAPTIC INPUT */
    Vec3r _last_haptic_pos;   // tracks the last haptic posiion
    std::unique_ptr<HapticDeviceManager> _haptic_device_manager;

    /** KEYBOARD INPUT */
    constexpr static Real IT_ROT_RATE = 3; // rad/s
    constexpr static Real IT_TRANS_RATE = 0.005; // m/s
    constexpr static Real OT_ROT_RATE = 3; // rad/s
    constexpr static Real OT_TRANS_RATE = 0.005; // m/s

    std::map<SimulationInput::Key, bool> _keys_held;     // tracks which keys are held (from a pre-defined set of keys)
};

} // namespace Sim

#endif // __VIRTUOSO_SIMULATION_HPP