#ifndef __VIRTUOSO_TISSUE_GRASPING_SIMULATION_HPP
#define __VIRTUOSO_TISSUE_GRASPING_SIMULATION_HPP

#include "simulation/VirtuosoSimulation.hpp"
#include "config/simulation/VirtuosoTissueGraspingSimulationConfig.hpp"

namespace Sim
{

class VirtuosoTissueGraspingSimulation : public VirtuosoSimulation
{
    public:
    struct TissueClasses
    {
        constexpr static int TRACHEA=0;
        constexpr static int TUMOR=1;
    };

    VirtuosoTissueGraspingSimulation(const Config::VirtuosoTissueGraspingSimulationConfig* config);

    virtual std::string type() const override { return "VirtuosoTissueGraspingSimulation"; }

    virtual void setup() override;

    /** Notifies the simulation that a key has been pressed in the viewer.
     * @param key : the key that was pressed
     * @param action : the action performed on the keyboard
     * @param modifiers : the modifiers (i.e. Shift, Ctrl, Alt)
     */
    virtual void notifyKeyPressed(SimulationInput::Key key, SimulationInput::KeyAction action, int modifiers) override;

    virtual void notifyMouseButtonPressed(SimulationInput::MouseButton button, SimulationInput::MouseAction action, int modifiers) override;

    virtual void notifyMouseMoved(double x, double y) override;

    virtual void notifyMouseScrolled(double dx, double dy) override;

    const Geometry::TetMesh* tissueMesh() const { assert(_tissue_obj); return _tissue_obj->tetMesh(); }

    protected:

    void _updateGraphics() override;
    
    void _timeStep() override;

    void _moveCursor(const Vec3r& dp);

    void _toggleGoal();
    void _changeGoal();

    int _calculateScore();

    protected:

    XPBDMeshObject_Base* _tissue_obj;    // the tissue XPBD object that is being manipulated
    std::optional<std::string> _fixed_faces_filename;   // a .txt filename that lists all the faces that should be held fixed

    std::optional<std::string> _tumor_faces_filename;   // a .txt filename that lists all the faces that are part of the tumor
    std::vector<int> _tumor_faces;  // list of face indices corresponding to the tumor

    int _current_score;

    bool _goal_active;
    std::optional<std::string> _goal_filename;      // a .obj filename that has a deformed tissue mesh that represents the goal
    std::optional<std::string> _goals_folder;       // a folder path that has .obj files representing goal tissue states
    // RigidMeshObject* _goal_obj;          // the goal state of the tissue surface
    std::vector<RigidMeshObject*> _goal_objs;   // vector of tissue goal states
    int _goal_obj_ind;

    bool _grasping;                 // whether or not we are actively grasping the tissue
    std::vector<int> _grasped_vertices; // indexes of the grasped vertices in the tissue mesh

    /** MOUSE INPUT */
    Vec2r _last_mouse_pos;    // tracks the last mouse position (used in when mouse input is used to control the arms)

    /** HAPTIC INPUT */
    Vec3r _dummy;
    Vec3r _last_haptic_pos;   // tracks the last haptic posiion
    std::unique_ptr<HapticDeviceManager> _haptic_device_manager;
    // std::unique_ptr<HapticDeviceManager> _haptic_device1;    // manages haptic device and provides an interface to their state
    // std::unique_ptr<HapticDeviceManager> _haptic_device2;    // second haptic device (only used in the "Double Haptic" input mode)

    Vec3r _last_force;  // the last force sent to the haptic device (used for smoothing)

    /** KEYBOARD INPUT */
    constexpr static Real IT_ROT_RATE = 3; // rad/s
    constexpr static Real IT_TRANS_RATE = 0.005; // m/s
    constexpr static Real OT_ROT_RATE = 3; // rad/s
    constexpr static Real OT_TRANS_RATE = 0.005; // m/s

    std::map<int, bool> _keys_held;     // tracks which keys are held (from a pre-defined set of keys)
};

} // namespace Sim

#endif // __VIRTUOSO_TISSUE_GRASPING_SIMULATION_HPP