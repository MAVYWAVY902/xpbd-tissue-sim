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

    protected:

    void _timeStep() override;
    void _toggleTissueGrasping();
    std::set<unsigned> _getAllVerticesInGraspingArea();

    double _grasp_size;
    double _z_scaling;
    SimulationInputDevice _input_device;

    bool _grasping;
    Eigen::Vector2d _mouse_pos_2d;
    Eigen::Vector3d _mouse_pos_3d;
    std::vector<std::shared_ptr<StaticVertexDriver> > _grasped_vertex_drivers;
    ElasticMeshObject* _tissue_block;

    /** Manages haptic device(s) */
    std::unique_ptr<HapticDeviceManager> _haptic_device_manager;
};

#endif // __TISSUE_GRASPING_SIMULATION_HPP