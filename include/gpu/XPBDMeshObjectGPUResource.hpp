#ifndef __XPBD_MESH_OBJECT_GPU_RESOURCE_HPP
#define __XPBD_MESH_OBJECT_GPU_RESOURCE_HPP

#include "gpu/GPUResource.hpp"
#include "gpu/ArrayGPUResource.hpp"
#include "gpu/WritableArrayGPUResource.hpp"
#include "gpu/TetMeshGPUResource.hpp"

#include "simobject/XPBDMeshObject.hpp"

namespace Sim
{

class XPBDMeshObjectGPUResource : public HostReadableGPUResource, HostWritableGPUResource
{
    public:
    explicit XPBDMeshObjectGPUResource(XPBDMeshObject* xpbd_obj)
        : _xpbd_obj(xpbd_obj), 
          _mesh_gpu_resource(xpbd_obj->tetMesh()),
          _velocity_resource(xpbd_obj->_vertex_velocities.data(), xpbd_obj->_vertex_velocities.size()),
          _masses_resource(xpbd_obj->_vertex_masses.data(), xpbd_obj->_vertex_masses.size())
    {
    }

    virtual ~XPBDMeshObjectGPUResource()
    {

    }

    virtual void allocate() override
    {
        _mesh_gpu_resource.allocate();
        _velocity_resource.allocate();
        _masses_resource.allocate();
    }

    virtual void fullCopyToDevice() const override
    {
        _mesh_gpu_resource.fullCopyToDevice();
        _velocity_resource.fullCopyToDevice();
        _masses_resource.fullCopyToDevice();
    }

    virtual void partialCopyToDevice() const override
    {
        _mesh_gpu_resource.partialCopyToDevice();
    }

    virtual void copyFromDevice() override
    {
        _mesh_gpu_resource.copyFromDevice();
        _velocity_resource.copyFromDevice();
    }

    const TetMeshGPUResource& meshGpuResource() const { return _mesh_gpu_resource; }

    const ArrayGPUResource<float>& massesGpuResource() const { return _masses_resource; }

    private:
    XPBDMeshObject* _xpbd_obj;
    TetMeshGPUResource _mesh_gpu_resource;
    WritableArrayGPUResource<float> _velocity_resource;
    ArrayGPUResource<float> _masses_resource;
    // ArrayGPUResource<float> _volumes_resource;
    // ArrayGPUResource<float> _Qs_resource;
};

} // namespace Sim

#endif // __XPBD_MESH_OBJECT_GPU_RESOURCE_HPP