#include "gpu/GPUResourceManager.hpp"

#include "gpu/MeshGPUResource.hpp"

namespace Sim
{

GPUResourceManager::GPUResourceManager()
{

}

void GPUResourceManager::createManagedResource(void* ptr)
{
    // TODO - try casting ptr as various different types and create a GPU resource for it depending on the type
}

const GPUResource* GPUResourceManager::getResource(void* ptr) const
{
    std::unordered_map<void*, std::unique_ptr<GPUResource>>::const_iterator it = _resources.find(ptr);
    if (it != _resources.end())
        return _resources.at(ptr).get();
    else
    {
        std::cerr << "No GPUResource found for object at " << ptr << "!" << std::endl;
        assert(0);
    }
}

GPUResource* GPUResourceManager::getResource(void* ptr)
{
    std::unordered_map<void*, std::unique_ptr<GPUResource>>::iterator it = _resources.find(ptr);
    if (it != _resources.end())
        return _resources.at(ptr).get();
    else
    {
        std::cerr << "No GPUResource found for object at " << ptr << "!" << std::endl;
        assert(0);
    }
}

void GPUResourceManager::deleteResource(void* ptr)
{
    std::unordered_map<void*, std::unique_ptr<GPUResource>>::iterator it = _resources.find(ptr);
    if (it != _resources.end())
        _resources.erase(ptr);
    else
    {
        std::cerr << "No GPUResource found for object at " << ptr << "!" << std::endl;
        assert(0);
    }
}

} // namespace Sim