#ifndef __GPU_RESOURCE_MANAGER_HPP
#define __GPU_RESOURCE_MANAGER_HPP

#include "gpu/GPUResource.hpp"
#include "gpu/ArrayGPUResource.hpp"
#include "geometry/Mesh.hpp"
#include "geometry/SDF.hpp"

#include <memory>
#include <unordered_map>

#include <iostream>

namespace Sim
{

class GPUResourceManager
{
    public:

    explicit GPUResourceManager();

    // void createManagedResource(const void* ptr);

    void createManagedResource(const Geometry::Mesh* mesh_ptr);

    void createManagedResource(const Geometry::SDF* sdf_ptr);

    // void createManagedResource(void* ptr);

    // template <typename T>
    // void createManagedResource(T* arr_ptr, int num_elements);

    template <typename T>
    void createManagedResource(T* arr_ptr, int num_elements)
    {
        const std::uintptr_t key = reinterpret_cast<std::uintptr_t>(arr_ptr);
        
        // make sure that there is not already a GPU resource for this object
        if (_resources.count(key) > 0)
        {
            std::cerr << "GPUResource for this object at " << arr_ptr << " already exists!" << std::endl;
            assert(0);
        }

        _resources[key] = std::make_unique<ArrayGPUResource<T>>(arr_ptr, num_elements);
        _resources[key]->allocate();
        
    }

    const HostReadableGPUResource* getResource(const void* ptr) const;

    HostReadableGPUResource* getResource(const void* ptr);

    void deleteResource(const void* ptr);

    protected:
    std::unordered_map<std::uintptr_t, std::unique_ptr<HostReadableGPUResource>> _resources;

};

} // namespace Sim

#endif // __GPU_RESOURCE_MANAGER_HPP