#ifndef __XPBD_PARALLEL_JACOBI_SOLVER_HPP
#define __XPBD_PARALLEL_JACOBI_SOLVER_HPP

#ifdef HAVE_CUDA

#include "solver/XPBDSolver.hpp"
#include "gpu/XPBDMeshObjectGPUResource.hpp"
#include "gpu/ArrayGPUResource.hpp"
#include "gpu/WritableArrayGPUResource.hpp"

namespace Solver
{

class XPBDParallelJacobiSolver : public XPBDSolver
{
    public:
    explicit XPBDParallelJacobiSolver(Sim::XPBDMeshObject* obj, int num_iter, XPBDResidualPolicy residual_policy);

    virtual void solve() override;

    virtual void _solveConstraints(Real* /*data*/) override {}

    private:
    std::vector<float> _element_Qs;
    std::vector<float> _element_volumes;
    std::vector<float> _temp_vertices;

    Sim::XPBDMeshObjectGPUResource* _xpbd_obj_resource;
    Sim::ArrayGPUResource<float> _Qs_resource;
    Sim::ArrayGPUResource<float> _volumes_resource;
    Sim::ArrayGPUResource<float> _temp_vertices_resource;
};

} // namespace Solver

#endif

#endif // __XPBD_PARALLEL_JACOBI_SOLVER_HPP