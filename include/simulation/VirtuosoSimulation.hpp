#ifndef __VIRTUOSO_SIMULATION_HPP
#define __VIRTUOSO_SIMULATION_HPP

#include "simulation/Simulation.hpp"
#include "simobject/VirtuosoArm.hpp"

#include "config/VirtuosoSimulationConfig.hpp"

#include "haptics/HapticDeviceManager.hpp"

#include "simobject/RigidPrimitives.hpp"
#include "simobject/XPBDMeshObject.hpp"

#include <map>

namespace Sim
{

class VirtuosoSimulation : public Simulation
{
    public:
    VirtuosoSimulation(const std::string& config_filename);

    VirtuosoSimulation();

    virtual std::string type() const override { return "VirtuosoSimulation"; }

    virtual void setup() override;

    /** Notifies the simulation that a key has been pressed in the viewer.
     * @param key : the key that was pressed
     * @param action : the action performed on the keyboard
     * @param modifiers : the modifiers (i.e. Shift, Ctrl, Alt)
     */
    virtual void notifyKeyPressed(int key, int action, int modifiers) override;

    virtual void notifyMouseButtonPressed(int button, int action, int modifiers) override;

    virtual void notifyMouseMoved(double x, double y) override;

    virtual void notifyMouseScrolled(double dx, double dy) override;

    protected:

    void _updateGraphics() override;
    
    void _timeStep() override;

    void _moveCursor(const Eigen::Vector3d& dp);

    void _toggleTissueGrasping();

    protected:

    XPBDMeshObject* _tissue_obj;    // the tissue XPBD object that is being manipulated
    std::optional<std::string> _fixed_faces_filename;   // a .txt filename that lists all the faces that should be held fixed

    VirtuosoArm* _virtuoso_arm1;    // one Virtuoso arm
    VirtuosoArm* _virtuoso_arm2;    // the second Virtuoso arm (optional)

    VirtuosoArm* _active_arm;       // whichever arm is being actively controlled (assuming only one input device)
    
    RigidSphere* _tip_cursor;       // spherical object for visualizing grasp area 
    bool _grasping;                 // whether or not we are actively grasping the tissue

    
    SimulationInputDevice _input_device;    // the type of input device used (Keyboard, Mouse, or Haptic)

    /** MOUSE INPUT */
    Eigen::Vector2d _last_mouse_pos;    // tracks the last mouse position (used in when mouse input is used to control the arms)

    /** HAPTIC INPUT */
    Eigen::Vector3d _last_haptic_pos;   // tracks the last haptic posiion
    std::unique_ptr<HapticDeviceManager> _haptic_device_manager;    // manages haptic device and provides an interface to their state

    /** KEYBOARD INPUT */
    constexpr static double IT_ROT_RATE = 3; // rad/s
    constexpr static double IT_TRANS_RATE = 0.005; // m/s
    constexpr static double OT_ROT_RATE = 3; // rad/s
    constexpr static double OT_TRANS_RATE = 0.005; // m/s

    std::map<int, bool> _keys_held;     // tracks which keys are held (from a pre-defined set of keys)
};

} // namespace Sim

#endif // __VIRTUOSO_SIMULATION_HPP