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

    Real _grasp_radius; // current grasp radius
    bool _fix_min_z; // whether or not to fix all vertices that share the minimum z coordinate

    bool _grasping; // whether or not we are currently grasping
    std::vector<std::pair<Sim::XPBDMeshObject_Base*, int>> _grasped_vertices;   // stores the currently grasped vertices as pairs of object pointers and vertex indices

    RigidSphere* _cursor;   // the cursor visualization object that shows the user where the grasping volume is

    Vec2r _last_mouse_pos;  // on-screen mouse position for the last frame

    std::map<int, bool> _keys_held;     // tracks which keys are held (from a pre-defined set of keys)
};

} // namespace Sim

#endif // __GRASPING_SIMULATION_HPP