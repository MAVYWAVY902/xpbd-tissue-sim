#include "gpu/XPBDMeshObjectGPUResource.hpp"

#include "simobject/XPBDMeshObject.hpp"

namespace Sim
{
XPBDMeshObjectGPUResource::XPBDMeshObjectGPUResource(XPBDMeshObject* xpbd_obj)
    : _xpbd_obj(xpbd_obj), 
        _mesh_gpu_resource(xpbd_obj->tetMesh()),
        _velocity_resource(xpbd_obj->_vertex_velocities.data(), xpbd_obj->_vertex_velocities.size()),
        _masses_resource(xpbd_obj->_vertex_masses.data(), xpbd_obj->_vertex_masses.size())
{
}

XPBDMeshObjectGPUResource::~XPBDMeshObjectGPUResource()
{

}

void XPBDMeshObjectGPUResource::allocate()
{
    _mesh_gpu_resource.allocate();
    _velocity_resource.allocate();
    _masses_resource.allocate();
}

void XPBDMeshObjectGPUResource::fullCopyToDevice() const
{
    _mesh_gpu_resource.fullCopyToDevice();
    _velocity_resource.fullCopyToDevice();
    _masses_resource.fullCopyToDevice();
}

void XPBDMeshObjectGPUResource::partialCopyToDevice() const
{
    _mesh_gpu_resource.partialCopyToDevice();
}

void XPBDMeshObjectGPUResource::copyFromDevice()
{
    _mesh_gpu_resource.copyFromDevice();
    _velocity_resource.copyFromDevice();
}

} // namespace Sim