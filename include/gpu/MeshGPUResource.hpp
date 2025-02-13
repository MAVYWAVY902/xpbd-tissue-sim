#ifndef __MESH_GPU_RESOURCE_HPP
#define __MESH_GPU_RESOURCE_HPP

#include "gpu/GPUResource.hpp"
#include "geometry/Mesh.hpp"

#include <iostream>

namespace Sim
{

class MeshGPUResource : public HostReadableGPUResource
{
    public:
    explicit MeshGPUResource(const Geometry::Mesh* mesh)
        : _mesh(mesh)
    {
    }

    virtual ~MeshGPUResource()
    {
        cudaFree(_d_vertices);
        cudaFree(_d_faces);
    }

    virtual void allocate() override
    {
        _vertices_size = _mesh->numVertices() * 3 * sizeof(float);
        _faces_size = _mesh->numFaces() * 3 * sizeof(int);
        // gpuErrchk(cudaMalloc((void**)&_d_vertices, _vertices_size));
        // gpuErrchk(cudaMalloc((void**)&_d_faces, _faces_size));
        // TODO: this is gross! const_cast is ugly let's do something about it
        float* vp = const_cast<float*>(_mesh->vertices().data());
        int* fp = const_cast<int*>(_mesh->faces().data());
        gpuErrchk(cudaHostRegister(vp, _vertices_size, cudaHostRegisterMapped));
        gpuErrchk(cudaHostRegister(fp, _faces_size, cudaHostRegisterMapped));
        gpuErrchk(cudaHostGetDevicePointer((void**)&_d_vertices, vp, 0));
        gpuErrchk(cudaHostGetDevicePointer((void**)&_d_faces, fp, 0));
    }

    virtual void copyToDevice() const override
    {
        gpuErrchk(cudaMemcpy(_d_vertices, _mesh->vertices().data(), _vertices_size, cudaMemcpyHostToDevice));
        gpuErrchk(cudaMemcpy(_d_faces, _mesh->faces().data(), _faces_size, cudaMemcpyHostToDevice));
    }

    void copyVerticesToDevice() const
    {
        gpuErrchk(cudaMemcpy(_d_vertices, _mesh->vertices().data(), _vertices_size, cudaMemcpyHostToDevice));
    }

    float* gpuVertices() const { return _d_vertices; }
    int* gpuFaces() const { return _d_faces; }

    private:
    const Geometry::Mesh* _mesh;

    size_t _vertices_size;
    size_t _faces_size;
    float* _d_vertices;
    int* _d_faces;
};

} // namespace Sim

#endif // __MESH_GPU_RESOURCE_HPP