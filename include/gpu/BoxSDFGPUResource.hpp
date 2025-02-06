#ifndef __BOX_SDF_GPU_RESOURCE_HPP
#define __BOX_SDF_GPU_RESOURCE_HPP

#include "gpu/GPUResource.hpp"
#include "gpu/GPUStructs.hpp"

#include "geometry/BoxSDF.hpp"

#include "utils/CudaHelperMath.h"

namespace Sim
{

class BoxSDFGPUResource : public HostReadableGPUResource
{

     public:
    explicit BoxSDFGPUResource(const Geometry::BoxSDF* box_sdf)
        : _sdf(box_sdf)
    {

    }

    virtual ~BoxSDFGPUResource()
    {
        cudaFree(_d_sdf);
    }

    virtual void allocate() override
    {
        cudaMalloc((void**)&_d_sdf, sizeof(GPUBoxSDF));
    }

    virtual void copyToDevice() const override
    {
        GPUBoxSDF gpu_sdf;
        const Vec3r& pos = _sdf->box()->position();
        const Vec4r& ori = _sdf->box()->orientation();
        const Vec3r& sz = _sdf->box()->size();
        gpu_sdf.position = make_float3(pos[0], pos[1], pos[2]);
        gpu_sdf.orientation = make_float4(ori[0], ori[1], ori[2], ori[3]);
        gpu_sdf.size = make_float3(sz[0], sz[1], sz[2]);

        cudaMemcpy(_d_sdf, &gpu_sdf, sizeof(GPUBoxSDF), cudaMemcpyHostToDevice);
    }

    GPUBoxSDF* gpuSDF() const { return _d_sdf; }

    private:
    const Geometry::BoxSDF* _sdf;
    GPUBoxSDF* _d_sdf;

};


} // namespace Sim

#endif // __BOX_SDF_GPU_RESOURCE_HPP