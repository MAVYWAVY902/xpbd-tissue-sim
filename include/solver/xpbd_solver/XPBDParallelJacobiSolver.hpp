#ifndef __XPBD_PARALLEL_JACOBI_SOLVER_HPP
#define __XPBD_PARALLEL_JACOBI_SOLVER_HPP

#ifdef HAVE_CUDA

#include "solver/xpbd_solver/XPBDSolver.hpp"
#include "gpu/resource/XPBDMeshObjectGPUResource.hpp"
#include "gpu/resource/ArrayGPUResource.hpp"
#include "gpu/resource/VectorGPUResource.hpp"
#include "gpu/resource/MonitoredVectorGPUResource.hpp"
#include "gpu/resource/WritableArrayGPUResource.hpp"

#include "gpu/projector/GPUConstraintProjector.cuh"
#include "gpu/projector/GPUCombinedConstraintProjector.cuh"
#include "gpu/projector/GPUNeohookeanCombinedConstraintProjector.cuh"

#include "gpu/XPBDSolver.cuh"

#include "common/VariadicContainer.hpp"
#include "common/VariadicVectorContainer.hpp"
#include "common/MonitoredVector.hpp"

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

template<bool IsFirstOrder, typename ...ConstraintProjectors>
class XPBDParallelJacobiSolver : public XPBDSolver<IsFirstOrder, ConstraintProjectors...>
{ 
    public:
    explicit XPBDParallelJacobiSolver(Sim::XPBDMeshObject_Base* obj, int num_iter, XPBDSolverResidualPolicyEnum residual_policy)
        : XPBDSolver<IsFirstOrder, ConstraintProjectors...>(obj, num_iter, residual_policy) 
    { 

    }

    void setup()
    {
        this->XPBDSolver<IsFirstOrder, ConstraintProjectors...>::setup();

        _temp_vertices.resize(this->_obj->mesh()->numVertices()*3);
        _temp_vertices_resource = Sim::ArrayGPUResource<float>(_temp_vertices.data(), _temp_vertices.size());

        // createGPUResource() will create the GPU resource for the XPBDMeshObject AND allocate memory for it
        this->_obj->createGPUResource();
        // TODO: remove dynamic_cast
        _xpbd_obj_resource = dynamic_cast<Sim::XPBDMeshObjectGPUResource*>(this->_obj->gpuResource());
        _xpbd_obj_resource->fullCopyToDevice();

        // point the VectorGPUResources to their appropriate vectors of GPUConstraintProjectors
        // using an ugly fold expression
        ((_gpu_projector_resources.template get<Sim::MonitoredVectorGPUResource<typename ConstraintProjectors::GPUConstraintProjectorType>>()  // get the vector resource corresponding to the type
            .setVecPtr(&_gpu_projectors.template get<MonitoredVector<typename ConstraintProjectors::GPUConstraintProjectorType>>()), ...));            // and point it to the vector corresponding to the type

        // allocate memory for the temporary vertices on the GPU
        _temp_vertices_resource.allocate();
    }

    template<class CPUProjectorType>
    void setNumProjectorsOfType(int size)
    {
        _gpu_projectors.template get<MonitoredVector<typename CPUProjectorType::GPUConstraintProjectorType>>().resize(size);
        // since the vector's size was changed we need to reallocate the corresponding GPU resource
        _gpu_projector_resources.template get<Sim::MonitoredVectorGPUResource<typename CPUProjectorType::GPUConstraintProjectorType>>().allocate();
        
    }

    void solve()
    {
        auto copy_projectors = [&](auto dummy)
        {
            using ProjectorType = typename decltype(dummy)::type;
            auto& projector_resource = _gpu_projector_resources.template get<Sim::MonitoredVectorGPUResource<ProjectorType>>();
            projector_resource.allocate();
            projector_resource.fullCopyToDevice();
            _gpu_projectors.template get<MonitoredVector<ProjectorType>>().resetStateChanged();
        };

        (copy_projectors(TypeIdentity<typename ConstraintProjectors::GPUConstraintProjectorType>{}), ...);

        _xpbd_obj_resource->partialCopyToDevice();

        float* vertices = _xpbd_obj_resource->meshGpuResource().gpuVertices();
        float* new_vertices = _temp_vertices_resource.gpuArr();

        CopyVerticesMemcpy(new_vertices, vertices, this->_obj->mesh()->numVertices());

        // launch kernel for each type of constraint
        auto launch_kernel = [&](auto dummy)
        {
            using ProjectorType = typename decltype(dummy)::type;
            ProjectorType* projectors = _gpu_projector_resources.template get<Sim::MonitoredVectorGPUResource<ProjectorType>>().gpuArr();
            int num_projectors = _gpu_projectors.template get<MonitoredVector<ProjectorType>>().size();
            // std::cout << "Num projectors: " << num_projectors << std::endl;
            if (num_projectors > 0)
                LaunchProjectConstraints<ProjectorType>(projectors, num_projectors, vertices, new_vertices);
        };

        (launch_kernel(TypeIdentity<typename ConstraintProjectors::GPUConstraintProjectorType>{}), ...);

        CopyVerticesMemcpy(vertices, new_vertices, this->_obj->mesh()->numVertices());

        _xpbd_obj_resource->copyFromDevice();
    }

    // not needed - constraint solve happens on GPU
    virtual void _iterateConstraints() override {}

    template<class... Constraints>
    void addConstraintProjector(Real dt, Constraints* ... constraints)
    {
        auto projector = this->_createConstraintProjector(dt, constraints...);
        auto gpu_projector = projector.createGPUConstraintProjector();
        _gpu_projectors.template get<MonitoredVector<decltype(gpu_projector)>>().push_back(std::move(gpu_projector));
    }

    template<class... Constraints>
    void setConstraintProjector(int index, Real dt, Constraints*... constraints)
    {
        auto projector = this->_createConstraintProjector(dt, constraints...);
        auto gpu_projector = projector.createGPUConstraintProjector();
        _gpu_projectors.template get<MonitoredVector<decltype(gpu_projector)>>()[index] = gpu_projector;
    }

    template<class CPUProjectorType>
    const typename CPUProjectorType::GPUConstraintProjectorType& getConstraintProjector(int index) const
    {
        return _gpu_projectors.template get<MonitoredVector<typename CPUProjectorType::GPUConstraintProjectorType>>()[index];
    }

    template<class CPUProjectorType>
    void setProjectorValidity(int index, bool is_valid)
    {
        _gpu_projectors.template get<MonitoredVector<typename CPUProjectorType::GPUConstraintProjectorType>>()[index].valid = is_valid;
    }

    template<class CPUProjectorType>
    void setAllProjectorsOfTypeInvalid()
    {
        using VecType = MonitoredVector<typename CPUProjectorType::GPUConstraintProjectorType>;

        VecType& vec = _gpu_projectors.template get<VecType>();
        for (auto& elem : vec.vec())
        {
            elem.valid = false;
        }
    }

    template<class CPUProjectorType>
    void clearProjectorsOfType()
    {
        using VecType = MonitoredVector<typename CPUProjectorType::GPUConstraintProjectorType>;

        VecType& vec = _gpu_projectors.template get<VecType>();
        vec.clear();
    }

    protected:
    template<class Constraint>
    int _addConstraintProjector(Real dt,  Constraint* constraint)
    {
        typename Constraint::GPUConstraintType gpu_constraint = constraint->createGPUConstraint();
        typedef GPUConstraintProjector<IsFirstOrder, typename Constraint::GPUConstraintType> ProjectorType;
        ProjectorType projector(std::move(gpu_constraint), dt);
        _gpu_projectors.template get<ProjectorType>().push_back(std::move(projector));

        return 0;
    }

    template<class Constraint1, class Constraint2>
    int _addConstraintProjector(Real dt, Constraint1* constraint1, Constraint2* constraint2)
    {
        typename Constraint1::GPUConstraintType gpu_constraint1 = constraint1->createGPUConstraint();
        typename Constraint2::GPUConstraintType gpu_constraint2 = constraint2->createGPUConstraint();
        typedef GPUCombinedConstraintProjector<IsFirstOrder, typename Constraint1::GPUConstraintType, typename Constraint2::GPUConstraintType> ProjectorType;
        ProjectorType projector(std::move(gpu_constraint1), std::move(gpu_constraint2), dt);
        _gpu_projectors.template get<MonitoredVector<ProjectorType>>().push_back(std::move(projector));

        return 0;
    }

    private:
    // VariadicVectorContainer<GPUConstraintProjectors...> _gpu_projectors;
    // VariadicContainer<Sim::VectorGPUResource<GPUConstraintProjectors>...> _gpu_projector_resources;
    VariadicContainer<MonitoredVector<typename ConstraintProjectors::GPUConstraintProjectorType>...> _gpu_projectors;
    VariadicContainer<Sim::MonitoredVectorGPUResource<typename ConstraintProjectors::GPUConstraintProjectorType>...> _gpu_projector_resources;
    

    // std::vector<float> _element_Qs;
    // std::vector<float> _element_volumes;

    Sim::XPBDMeshObjectGPUResource* _xpbd_obj_resource;
    // Sim::ArrayGPUResource<float> _Qs_resource;
    // Sim::ArrayGPUResource<float> _volumes_resource;

    std::vector<float> _temp_vertices;
    Sim::ArrayGPUResource<float> _temp_vertices_resource;
    
};

} // namespace Solver
#else

namespace Solver
{
    template<bool IsFirstOrder, typename ...ConstraintProjectors>
    class XPBDParallelJacobiSolver : public XPBDSolver<IsFirstOrder, ConstraintProjectors...>
    {
        public:
        using projector_reference_container_type = typename XPBDSolver<IsFirstOrder, ConstraintProjectors...>::projector_reference_container_type;

        public:
        explicit XPBDParallelJacobiSolver(Sim::XPBDMeshObject_Base_<IsFirstOrder>* obj, int num_iter, XPBDSolverResidualPolicyEnum residual_policy)
        : XPBDSolver<IsFirstOrder, ConstraintProjectors...>(obj, num_iter, residual_policy) 
        { 
            assert(0 && "Not implemented for CPU!");
        }
        // not needed - constraint solve happens on GPU
        virtual void _iterateConstraints() override
        {
            assert(0 && "Not implemented for CPU!");
        }

        // not needed - constraint solve happens on GPU
        virtual void _iterateConstraints(projector_reference_container_type& /*projector_references*/) override
        {
            assert(0 && "Not implemented for CPU!");
        }
    };
}

#endif

#endif // __XPBD_PARALLEL_JACOBI_SOLVER_HPP