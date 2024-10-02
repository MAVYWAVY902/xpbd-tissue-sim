#ifndef __TISSUE_GRASPING_SIMULATION_HPP
#define __TISSUE_GRASPING_SIMULATION_HPP

#include "OutputSimulation.hpp"
#include "HapticDeviceManager.hpp"
#include "TissueGraspingSimulationConfig.hpp"
#include <Eigen/Dense>

class TissueGraspingSimulation : public OutputSimulation
{
    public:

    explicit TissueGraspingSimulation(const std::string& config_filename);

    virtual std::string type() const override { return "TissueGraspingSimulation"; }

    virtual void setup() override;

    virtual void printInfo() const override;

    virtual void notifyMouseButtonPressed(int button, int action, int modifiers) override;

    virtual void notifyMouseMoved(double x, double y) override;

    /** Notifies the simulation that a key has been pressed in the viewer.
         * @param key : the key that was pressed
         * @param action : the action performed on the keyboard
         * @param modifiers : the modifiers (i.e. Shift, Ctrl, Alt)
         */
    virtual void notifyKeyPressed(int key, int action, int modifiers) override;

    protected:

    // void _timeStep() override;
    void _updateGraphics() override;
    void _toggleTissueGrasping();
    std::set<unsigned> _getAllVerticesInGraspingArea();

    Eigen::Vector3d _transformInputPosition(const Eigen::Vector3d& input_position);

    double _grasp_size;
    double _z_scaling;
    SimulationInputDevice _input_device;

    bool _grasping;
    Eigen::Vector2d _mouse_pos_2d;
    Eigen::Vector3d _mouse_pos_3d;
    std::vector<std::shared_ptr<StaticVertexDriver> > _grasped_vertex_drivers;
    ElasticMeshObject* _tissue_block;
    RigidMeshObject* _grasp_tip;
    Eigen::Vector3d _grasp_tip_position;

    /** Manages haptic device(s) */
    std::unique_ptr<HapticDeviceManager> _haptic_device_manager;
};

#endif // __TISSUE_GRASPING_SIMULATION_HPP