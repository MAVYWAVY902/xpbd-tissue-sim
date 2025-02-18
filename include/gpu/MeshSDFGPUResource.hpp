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
        CHECK_CUDA_ERROR(cudaMalloc((void**)&_d_sdf, sizeof(GPUMeshSDF)));

        const mesh2sdf::Array3<Real>& distance_grid = _sdf->distanceGrid();
        const mesh2sdf::Array3<Vec3r>& gradient_grid = _sdf->gradientGrid();

        cudaExtent dist_grid_extent = make_cudaExtent(distance_grid.ni * sizeof(float), distance_grid.nj, distance_grid.nk);
        CHECK_CUDA_ERROR(cudaMalloc3D(&_d_dist_grid_ptr, dist_grid_extent));
    }

    virtual void copyToDevice() const override
    {
        const mesh2sdf::Array3<Real>& distance_grid = _sdf->distanceGrid();
        cudaExtent dist_grid_extent = make_cudaExtent(distance_grid.ni * sizeof(float), distance_grid.nj, distance_grid.nk);

        cudaMemcpy3DParms dist_copy_params = {0};
        dist_copy_params.srcPtr = make_cudaPitchedPtr(const_cast<float*>(distance_grid.a.data()), distance_grid.ni*sizeof(float), distance_grid.nj, distance_grid.nk);
        dist_copy_params.dstPtr = _d_dist_grid_ptr;
        dist_copy_params.extent = dist_grid_extent;
        dist_copy_params.kind = cudaMemcpyHostToDevice;

        cudaMemcpy3D(&dist_copy_params);

        GPUMeshSDF gpu_sdf;
        const Vec3r& pos = _sdf->meshObj()->position();
        const Vec4r& ori = _sdf->meshObj()->orientation();
        const Vec3r& cell_size = _sdf->gridCellSize();
        const Vec3r& bbox_min = _sdf->gridBoundingBox().min;

        gpu_sdf.position = make_float3(pos[0], pos[1], pos[2]);
        gpu_sdf.orientation = make_float4(ori[0], ori[1], ori[2], ori[3]);
        gpu_sdf.grid_cell_size = make_float3(cell_size[0], cell_size[1], cell_size[2]);
        gpu_sdf.grid_bbox_min = make_float3(bbox_min[0], bbox_min[1], bbox_min[2]);
        gpu_sdf.grid_dims = make_int3(distance_grid.ni, distance_grid.nj, distance_grid.nk);
        gpu_sdf.dev_dist_grid_ptr = _d_dist_grid_ptr;

        cudaMemcpy(_d_sdf, &gpu_sdf, sizeof(GPUMeshSDF), cudaMemcpyHostToDevice);
    }

    GPUMeshSDF* gpuSDF() const { return _d_sdf; }
    cudaPitchedPtr devDistGrid() const { return _d_dist_grid_ptr; }

    private:
    const Geometry::MeshSDF* _sdf;
    cudaPitchedPtr _d_dist_grid_ptr;
    size_t _dist_grid_size;
    size_t _grad_grid_size;
    GPUMeshSDF* _d_sdf;
};


} // namespace Sim


#endif // __MESH_SDF_GPU_RESOURCE_HPP