#ifndef __TET_MESH_GPU_RESOURCE_HPP
#define __TET_MESH_GPU_RESOURCE_HPP

#include "gpu/MeshGPUResource.hpp"
#include "geometry/TetMesh.hpp"

namespace Sim
{

class TetMeshGPUResource : public MeshGPUResource
{
    public:
    explicit TetMeshGPUResource(const Geometry::TetMesh* tet_mesh)
        : MeshGPUResource(tet_mesh), _tet_mesh(tet_mesh)
    {
    }

    virtual ~TetMeshGPUResource()
    {
        cudaFree(_d_elements);
    }

    virtual void allocate() override
    {
        MeshGPUResource::allocate();

        _elements_size = _tet_mesh->numElements() * 4 * sizeof(int);
        cudaMalloc((void**)&_d_elements, _elements_size);
    }

    virtual void fullCopyToDevice() const override
    {
        MeshGPUResource::fullCopyToDevice();

        cudaMemcpy(_d_elements, _tet_mesh->elements().data(), _elements_size, cudaMemcpyHostToDevice);
    }

    int* gpuElements() const { return _d_elements; }

    private:
    const Geometry::TetMesh* _tet_mesh;

    size_t _elements_size;
    int* _d_elements;
};

} // namespace Sim

#endif // __TET_MESH_GPU_RESOURCE_HPP