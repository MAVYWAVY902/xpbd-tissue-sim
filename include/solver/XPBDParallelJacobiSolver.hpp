#ifndef __XPBD_PARALLEL_JACOBI_SOLVER_HPP
#define __XPBD_PARALLEL_JACOBI_SOLVER_HPP

#ifdef HAVE_CUDA

#include "solver/XPBDSolver.hpp"
#include "gpu/resource/XPBDMeshObjectGPUResource.hpp"
#include "gpu/resource/ArrayGPUResource.hpp"
#include "gpu/resource/VectorGPUResource.hpp"
#include "gpu/resource/WritableArrayGPUResource.hpp"

#include "gpu/projector/GPUConstraintProjector.cuh"
#include "gpu/projector/GPUCombinedConstraintProjector.cuh"
#include "gpu/projector/GPUNeohookeanCombinedConstraintProjector.cuh"

#include "gpu/XPBDSolver.cuh"

#include "common/VariadicContainer.hpp"
#include "common/VariadicVectorContainer.hpp"

#include <type_traits>

template<typename T>
struct TypeIdentity
{
    using type = T;
};

template<typename T>
using TypeIdentity_t = typename TypeIdentity<T>::type;

namespace Solver
{

template<typename ...GPUConstraintProjectors>
class XPBDParallelJacobiSolver : public XPBDSolver
{
    public:
    explicit XPBDParallelJacobiSolver(Sim::XPBDMeshObject* obj, int num_iter, XPBDResidualPolicy residual_policy)
        : XPBDSolver(obj, num_iter, residual_policy),
        _temp_vertices(obj->mesh()->numVertices()*3),
        _temp_vertices_resource(_temp_vertices.data(), _temp_vertices.size())
    {
        // createGPUResource() will create the GPU resource for the XPBDMeshObject AND allocate memory for it
        obj->createGPUResource();
        _xpbd_obj_resource = dynamic_cast<Sim::XPBDMeshObjectGPUResource*>(obj->gpuResource());
        _xpbd_obj_resource->fullCopyToDevice();

        // point the VectorGPUResources to their appropriate vectors of GPUConstraintProjectors
        // using an ugly fold expression
        ((_gpu_projector_resources.template get<Sim::VectorGPUResource<GPUConstraintProjectors>>()  // get the vector resource corresponding to the type
            .setVecPtr(&_gpu_projectors.template get<GPUConstraintProjectors>()), ...));            // and point it to the vector corresponding to the type

        // allocate memory for the temporary vertices on the GPU
        _temp_vertices_resource.allocate();

    }

    template<class ProjectorType>
    void setNumProjectorsOfType(int size)
    {
        _gpu_projectors.template resize<ProjectorType>(size);
        // since the vector's size was changed we need to reallocate the corresponding GPU resource
        _gpu_projector_resources.template get<ProjectorType>().allocate();
        
    }

    virtual void solve() override
    {
        auto copy_projectors = [&](auto dummy)
        {
            using ProjectorType = typename decltype(dummy)::type;
            auto& projector_resource = _gpu_projector_resources.template get<Sim::VectorGPUResource<ProjectorType>>();
            projector_resource.allocate();
            projector_resource.fullCopyToDevice();
        };

        (copy_projectors(TypeIdentity<GPUConstraintProjectors>{}), ...);

        _xpbd_obj_resource->partialCopyToDevice();

        float* vertices = _xpbd_obj_resource->meshGpuResource().gpuVertices();
        float* new_vertices = _temp_vertices_resource.gpuArr();

        CopyVerticesMemcpy(new_vertices, vertices, _obj->mesh()->numVertices());

        // launch kernel for each type of constraint
        auto launch_kernel = [&](auto dummy)
        {
            using ProjectorType = typename decltype(dummy)::type;
            ProjectorType* projectors = _gpu_projector_resources.template get<Sim::VectorGPUResource<ProjectorType>>().gpuArr();
            int num_projectors = _gpu_projectors.template get<ProjectorType>().size();
            // std::cout << "Num projectors: " << num_projectors << std::endl;
            if (num_projectors > 0)
                LaunchProjectConstraints<ProjectorType>(projectors, num_projectors, vertices, new_vertices);
        };

        (launch_kernel(TypeIdentity<GPUConstraintProjectors>{}), ...);

        CopyVerticesMemcpy(vertices, new_vertices, _obj->mesh()->numVertices());

        _xpbd_obj_resource->copyFromDevice();
    }

    virtual void _solveConstraints(Real* /*data*/) override {}

    template<class... Constraints>
    int addConstraintProjector(Real dt, const ConstraintProjectorOptions& options, Constraints* ... constraints)
    {
        std::cout << "addConstraintProjector..." << std::endl;
        _addConstraintProjector(dt, options, constraints...);
        return 0;
    }

    protected:
    template<class Constraint>
    int _addConstraintProjector(Real dt, const ConstraintProjectorOptions& options, Constraint* constraint)
    {
        typename Constraint::GPUConstraintType gpu_constraint = constraint->createGPUConstraint();
        typedef GPUConstraintProjector<typename Constraint::GPUConstraintType> ProjectorType;
        ProjectorType projector(std::move(gpu_constraint), dt);
        _gpu_projectors.template push_back<ProjectorType>(std::move(projector));

        return 0;
    }

    template<class Constraint1, class Constraint2>
    int _addConstraintProjector(Real dt, const ConstraintProjectorOptions& options, Constraint1* constraint1, Constraint2* constraint2)
    {
        std::cout << "_addConstraintProjector 2 constraints" << std::endl;
        typename Constraint1::GPUConstraintType gpu_constraint1 = constraint1->createGPUConstraint();
        typename Constraint2::GPUConstraintType gpu_constraint2 = constraint2->createGPUConstraint();
        typedef GPUCombinedConstraintProjector<typename Constraint1::GPUConstraintType, typename Constraint2::GPUConstraintType> ProjectorType;
        ProjectorType projector(std::move(gpu_constraint1), std::move(gpu_constraint2), dt);
        _gpu_projectors.template push_back<ProjectorType>(std::move(projector));

        return 0;
    }

    private:
    VariadicVectorContainer<GPUConstraintProjectors...> _gpu_projectors;
    VariadicContainer<Sim::VectorGPUResource<GPUConstraintProjectors>...> _gpu_projector_resources;

    // std::vector<float> _element_Qs;
    // std::vector<float> _element_volumes;

    Sim::XPBDMeshObjectGPUResource* _xpbd_obj_resource;
    // Sim::ArrayGPUResource<float> _Qs_resource;
    // Sim::ArrayGPUResource<float> _volumes_resource;

    std::vector<float> _temp_vertices;
    Sim::ArrayGPUResource<float> _temp_vertices_resource;
    
};

} // namespace Solver

#endif

#endif // __XPBD_PARALLEL_JACOBI_SOLVER_HPP