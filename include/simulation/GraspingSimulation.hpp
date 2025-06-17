#ifndef __GRASPING_SIMULATION_HPP
#define __GRASPING_SIMULATION_HPP

#include "simulation/Simulation.hpp"

#include "config/simulation/GraspingSimulationConfig.hpp"

namespace Sim
{

class GraspingSimulation : public Simulation
{
    public:
    GraspingSimulation(const Config::GraspingSimulationConfig* config);

    virtual std::string type() const override { return "GraspingSimulation"; }

    virtual void setup() override;

    virtual void notifyKeyPressed(int key, int action, int modifiers) override;

    virtual void notifyMouseButtonPressed(int button, int action, int modifiers) override;

    virtual void notifyMouseMoved(double x, double y) override;

    virtual void notifyMouseScrolled(double dx, double dy) override;

    protected:

    void _timeStep() override;

    void _moveCursor(const Vec3r& dp);

    void _toggleGrasping();
    

    protected:

    Real _grasp_radius;

    bool _grasping;
    std::vector<std::pair<Sim::XPBDMeshObject_Base*, int>> _grasped_vertices;

    RigidSphere* _cursor;

    Vec2r _last_mouse_pos;

    std::map<int, bool> _keys_held;     // tracks which keys are held (from a pre-defined set of keys)
};

} // namespace Sim

#endif // __GRASPING_SIMULATION_HPP