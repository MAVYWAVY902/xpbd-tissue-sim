#ifndef __MESH_SDF_GPU_RESOURCE_HPP
#define __MESH_SDF_GPU_RESOURCE_HPP

#include <Mesh2SDF/array3.hpp>

#include "gpu/GPUResource.hpp"
#include "gpu/GPUStructs.hpp"

#include "geometry/MeshSDF.hpp"

#include <cuda_runtime.h>

namespace Sim
{

class MeshSDFGPUResource : public HostReadableGPUResource
{
    public:
    explicit MeshSDFGPUResource(const Geometry::MeshSDF* mesh_sdf)
        : _sdf(mesh_sdf)
    {

    }

    virtual ~MeshSDFGPUResource()
    {
        cudaFree(_d_sdf);
    }

    virtual void allocate() override
    {
        cudaMalloc((void**)&_d_sdf, sizeof(GPUMeshSDF));

        const mesh2sdf::Array3<Real>& distance_grid = _sdf->distanceGrid();
        const mesh2sdf::Array3<Vec3r>& gradient_grid = _sdf->gradientGrid();

        cudaExtent dist_grid_extent = make_cudaExtent(distance_grid.ni * sizeof(float), distance_grid.nj, distance_grid.nk);
        cudaMalloc3D(&_d_sdf->dist_grid, dist_grid_extent);
    }

    virtual void copyToDevice() const override
    {
        const mesh2sdf::Array3<Real>& distance_grid = _sdf->distanceGrid();
        cudaExtent dist_grid_extent = make_cudaExtent(distance_grid.ni * sizeof(float), distance_grid.nj, distance_grid.nk);

        cudaMemcpy3DParms dist_copy_params = {0};
        dist_copy_params.srcPtr = make_cudaPitchedPtr(const_cast<float*>(distance_grid.a.data()), distance_grid.ni*sizeof(float), distance_grid.nj, distance_grid.nk);
        dist_copy_params.dstPtr = _d_sdf->dist_grid;
        dist_copy_params.extent = dist_grid_extent;
        dist_copy_params.kind = cudaMemcpyHostToDevice;

        cudaMemcpy3D(&dist_copy_params);
    }

    GPUMeshSDF* gpuSDF() const { return _d_sdf; }


    private:
    const Geometry::MeshSDF* _sdf;
    size_t _dist_grid_size;
    size_t _grad_grid_size;
    GPUMeshSDF* _d_sdf;
};


} // namespace Sim


#endif // __MESH_SDF_GPU_RESOURCE_HPP