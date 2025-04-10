#ifndef __XPBD_MESH_OBJECT_GPU_RESOURCE_HPP
#define __XPBD_MESH_OBJECT_GPU_RESOURCE_HPP

#include "gpu/resource/GPUResource.hpp"
#include "gpu/resource/ArrayGPUResource.hpp"
#include "gpu/resource/WritableArrayGPUResource.hpp"
#include "gpu/resource/TetMeshGPUResource.hpp"

// #include "simobject/XPBDMeshObject.hpp"

namespace Sim
{

class XPBDMeshObject_Base;

class XPBDMeshObjectGPUResource : public HostReadableGPUResource, HostWritableGPUResource
{
    public:
    explicit XPBDMeshObjectGPUResource(XPBDMeshObject_Base* xpbd_obj);

    virtual ~XPBDMeshObjectGPUResource();

    virtual void allocate() override;

    virtual void fullCopyToDevice() const override;

    virtual void partialCopyToDevice() const override;

    virtual void copyFromDevice() override;

    const TetMeshGPUResource& meshGpuResource() const { return _mesh_gpu_resource; }

    // const ArrayGPUResource<float>& massesGpuResource() const { return _masses_resource; }

    private:
    XPBDMeshObject_Base* _xpbd_obj;
    TetMeshGPUResource _mesh_gpu_resource;
    // WritableArrayGPUResource<float> _velocity_resource;
    // ArrayGPUResource<float> _masses_resource;
    // ArrayGPUResource<float> _volumes_resource;
    // ArrayGPUResource<float> _Qs_resource;
};

} // namespace Sim

#endif // __XPBD_MESH_OBJECT_GPU_RESOURCE_HPP