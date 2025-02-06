#include "gpu/GPUResourceManager.hpp"

#include "gpu/MeshGPUResource.hpp"
#include "gpu/TetMeshGPUResource.hpp"
#include "gpu/SphereSDFGPUResource.hpp"
#include "gpu/BoxSDFGPUResource.hpp"
#include "gpu/CylinderSDFGPUResource.hpp"
#include "gpu/ArrayGPUResource.hpp"

namespace Sim
{

GPUResourceManager::GPUResourceManager()
{

}

// void GPUResourceManager::createManagedResource(const void* obj_ptr)
// {
//     const std::uintptr_t key = reinterpret_cast<std::uintptr_t>(obj_ptr);

//     // make sure that there is not already a GPU resource for this object
//     if (_resources.count(key) > 0)
//     {
//         std::cerr << "GPUResource for this object at " << obj_ptr << " already exists!" << std::endl;
//         assert(0);
//     }

//     if (const Geometry::TetMesh* m = reinterpret_cast<const Geometry::TetMesh*>(obj_ptr))
//         _resources[key] = std::make_unique<TetMeshGPUResource>(m);
//     else if (const Geometry::Mesh* m = reinterpret_cast<const Geometry::Mesh*>(obj_ptr))
//         _resources[key] = std::make_unique<MeshGPUResource>(m);
//     else if (const Geometry::SphereSDF* sdf = reinterpret_cast<const Geometry::SphereSDF*>(obj_ptr))
//         _resources[key] = std::make_unique<SphereSDFGPUResource>(sdf);
//     else if (const Geometry::BoxSDF* sdf = reinterpret_cast<const Geometry::BoxSDF*>(obj_ptr))
//         _resources[key] = std::make_unique<BoxSDFGPUResource>(sdf);
//     else if (const Geometry::CylinderSDF* sdf = reinterpret_cast<const Geometry::CylinderSDF*>(obj_ptr))
//         _resources[key] = std::make_unique<CylinderSDFGPUResource>(sdf);
//     else
//     {
//         assert(0);
//     }

//     // allocate the resources
//     _resources[key]->allocate();
// }

// void GPUResourceManager::createManagedResource(void* obj_ptr)
// {
//     assert(0);
// }

void GPUResourceManager::createManagedResource(const Geometry::Mesh* mesh_ptr)
{
    const std::uintptr_t key = reinterpret_cast<std::uintptr_t>(mesh_ptr);

    if (const Geometry::TetMesh* m = reinterpret_cast<const Geometry::TetMesh*>(mesh_ptr))
    {
        _resources[key] = std::make_unique<TetMeshGPUResource>(m);
    }
    else
    {
        _resources[key] = std::make_unique<MeshGPUResource>(m);
    }

    _resources[key]->allocate();
}

void GPUResourceManager::createManagedResource(const Geometry::SDF* sdf_ptr)
{
    const std::uintptr_t key = reinterpret_cast<std::uintptr_t>(sdf_ptr);

    if (const Geometry::SphereSDF* sdf = reinterpret_cast<const Geometry::SphereSDF*>(sdf_ptr))
        _resources[key] = std::make_unique<SphereSDFGPUResource>(sdf);
    else if (const Geometry::BoxSDF* sdf = reinterpret_cast<const Geometry::BoxSDF*>(sdf_ptr))
        _resources[key] = std::make_unique<BoxSDFGPUResource>(sdf);
    else if (const Geometry::CylinderSDF* sdf = reinterpret_cast<const Geometry::CylinderSDF*>(sdf_ptr))
        _resources[key] = std::make_unique<CylinderSDFGPUResource>(sdf);

    _resources[key]->allocate();
}

const HostReadableGPUResource* GPUResourceManager::getResource(const void* ptr) const
{
    const std::uintptr_t key = reinterpret_cast<std::uintptr_t>(ptr);
    std::unordered_map<std::uintptr_t, std::unique_ptr<HostReadableGPUResource>>::const_iterator it = _resources.find(key);
    if (it != _resources.end())
        return it->second.get();
    else
    {
        std::cerr << "No GPUResource found for object at " << ptr << "!" << std::endl;
        assert(0);
    }
}

HostReadableGPUResource* GPUResourceManager::getResource(const void* ptr)
{
    const std::uintptr_t key = reinterpret_cast<std::uintptr_t>(ptr);
    std::unordered_map<std::uintptr_t, std::unique_ptr<HostReadableGPUResource>>::iterator it = _resources.find(key);
    if (it != _resources.end())
        return it->second.get();
    else
    {
        std::cerr << "No GPUResource found for object at " << ptr << "!" << std::endl;
        assert(0);
    }
}

void GPUResourceManager::deleteResource(const void* ptr)
{
    const std::uintptr_t key = reinterpret_cast<std::uintptr_t>(ptr);
    std::unordered_map<std::uintptr_t, std::unique_ptr<HostReadableGPUResource>>::iterator it = _resources.find(key);
    if (it != _resources.end())
        _resources.erase(it);
    else
    {
        std::cerr << "No GPUResource found for object at " << ptr << "!" << std::endl;
        assert(0);
    }
}

} // namespace Sim