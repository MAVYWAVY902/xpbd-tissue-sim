#ifndef __MESH_GPU_RESOURCE_HPP
#define __MESH_GPU_RESOURCE_HPP

#include "gpu/GPUResource.hpp"
#include "geometry/Mesh.hpp"


namespace Sim
{

class MeshGPUResource : GPUResource
{
    public:
    explicit MeshGPUResource(const Geometry::Mesh* mesh)
        : GPUResource(), _mesh(mesh)
    {
    }

    virtual ~MeshGPUResource()
    {
        cudaFree(_d_vertices);
        cudaFree(_d_faces);
    }

    virtual void allocate() override
    {
        vertices_size = _mesh->numVertices() * 3 * sizeof(float);
        faces_size = _mesh->numFaces() * 3 * sizeof(int);
        cudaMalloc((void**)&_d_vertices, vertices_size);
        cudaMalloc((void**)&_d_faces, faces_size);
    }

    virtual void copyToDevice() const override
    {
        cudaMemcpy(_d_vertices, _mesh->vertices().data(), vertices_size, cudaMemcpyHostToDevice);
        cudaMemcpy(_d_faces, _mesh->faces().data(), vertices_size, cudaMemcpyHostToDevice);
    }

    virtual void copyFromDevice() const override
    {
        // not implemented - mesh should not be being edited on the GPU
        assert(0);
    }

    private:
    const Geometry::Mesh* _mesh;

    size_t vertices_size;
    size_t faces_size;
    float* _d_vertices;
    int* _d_faces;
};

} // namespace Sim

#endif // __MESH_GPU_RESOURCE_HPP