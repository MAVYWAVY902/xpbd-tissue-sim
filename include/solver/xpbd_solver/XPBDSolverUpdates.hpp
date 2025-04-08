#ifndef __XPBD_SOLVER_UPDATES_HPP
#define __XPBD_SOLVER_UPDATES_HPP

#include "common/types.hpp"

namespace Sim
{
    class RigidObject;
}

namespace Solver
{

struct CoordinateUpdate
{
    Real* ptr;
    Real update;
};

struct RigidBodyUpdate
{
    Sim::RigidObject* obj_ptr;
    Real position_update[3];
    Real orientation_update[4];
};

} // namespace Solver

#endif // __XPBD_SOLVER_UPDATES_HPP