#ifndef __XPBD_SOLVER_UPDATES_HPP
#define __XPBD_SOLVER_UPDATES_HPP

namespace Sim
{
    class RigidObject;
}

namespace Solver
{

struct CoordinateUpdate
{
    float* ptr;
    float update;
};

struct RigidBodyUpdate
{
    Sim::RigidObject* obj_ptr;
    float position_update[3];
    float orientation_update[4];
};

} // namespace Solver

#endif // __XPBD_SOLVER_UPDATES_HPP