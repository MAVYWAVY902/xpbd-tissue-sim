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
    void _timeStep() override;

    void _moveCursor(const Eigen::Vector3d& dp);

    void _toggleTissueGrasping();

    // TODO: make these settable simulation parameters
    constexpr static double IT_ROT_RATE = 10; // rad/s
    constexpr static double IT_TRANS_RATE = 0.005; // m/s
    constexpr static double OT_ROT_RATE = 10; // rad/s
    constexpr static double OT_TRANS_RATE = 0.005; // m/s

    VirtuosoArm* _virtuoso_arm1;
    VirtuosoArm* _virtuoso_arm2;

    VirtuosoArm* _active_arm;

    XPBDMeshObject* _tissue_obj;

    std::optional<std::string> _fixed_faces_filename;
    SimulationInputDevice _input_device;

    RigidSphere* _tip_cursor;
    bool _grasping;

    std::map<int, bool> _keys_held;

    Eigen::Vector2d _last_mouse_pos;

    /** Manages haptic device(s) */
    std::unique_ptr<HapticDeviceManager> _haptic_device_manager;
};

} // namespace Sim

#endif // __VIRTUOSO_SIMULATION_HPP