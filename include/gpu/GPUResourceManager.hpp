#ifndef __GPU_RESOURCE_MANAGER_HPP
#define __GPU_RESOURCE_MANAGER_HPP

#include "gpu/GPUResource.hpp"
#include "simobject/Object.hpp"

#include <memory>
#include <unordered_map>

namespace Sim
{

class GPUResourceManager
{
    public:

    explicit GPUResourceManager();

    void createManagedResource(void* ptr);

    const GPUResource* getResource(void* ptr) const;

    GPUResource* getResource(void* ptr);

    void deleteResource(void* ptr);

    protected:
    std::unordered_map<void*, std::unique_ptr<GPUResource>> _resources;

};

} // namespace Sim

#endif // __GPU_RESOURCE_MANAGER_HPP