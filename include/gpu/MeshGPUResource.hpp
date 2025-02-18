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
        // CHECK_CUDA_ERROR(cudaMalloc((void**)&_d_vertices, _vertices_size));
        // CHECK_CUDA_ERROR(cudaMalloc((void**)&_d_faces, _faces_size));
        // TODO: this is gross! const_cast is ugly let's do something about it
        float* vp = const_cast<float*>(_mesh->vertices().data());
        int* fp = const_cast<int*>(_mesh->faces().data());
        CHECK_CUDA_ERROR(cudaHostRegister(vp, _vertices_size, cudaHostRegisterMapped));
        CHECK_CUDA_ERROR(cudaHostRegister(fp, _faces_size, cudaHostRegisterMapped));
        CHECK_CUDA_ERROR(cudaHostGetDevicePointer((void**)&_d_vertices, vp, 0));
        CHECK_CUDA_ERROR(cudaHostGetDevicePointer((void**)&_d_faces, fp, 0));
    }

    virtual void fullCopyToDevice() const override
    {
        CHECK_CUDA_ERROR(cudaMemcpy(_d_vertices, _mesh->vertices().data(), _vertices_size, cudaMemcpyHostToDevice));
        CHECK_CUDA_ERROR(cudaMemcpy(_d_faces, _mesh->faces().data(), _faces_size, cudaMemcpyHostToDevice));
    }

    void partialCopyToDevice() const override
    {
        // only the vertices change throughout the course of the simulation - faces stay constant
        CHECK_CUDA_ERROR(cudaMemcpy(_d_vertices, _mesh->vertices().data(), _vertices_size, cudaMemcpyHostToDevice));
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