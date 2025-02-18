#ifndef __CYLINDER_SDF_GPU_RESOURCE_HPP
#define __CYLINDER_SDF_GPU_RESOURCE_HPP

#include "gpu/GPUResource.hpp"
#include "gpu/GPUStructs.hpp"

#include "geometry/CylinderSDF.hpp"

#include "utils/CudaHelperMath.h"

namespace Sim
{

class CylinderSDFGPUResource : public HostReadableGPUResource
{

     public:
    explicit CylinderSDFGPUResource(const Geometry::CylinderSDF* cyl_sdf)
        : _sdf(cyl_sdf)
    {

    }

    virtual ~CylinderSDFGPUResource()
    {
        cudaFree(_d_sdf);
    }

    virtual void allocate() override
    {
        cudaMalloc((void**)&_d_sdf, sizeof(GPUCylinderSDF));
    }

    virtual void fullCopyToDevice() const override
    {
        GPUCylinderSDF gpu_sdf;
        const Vec3r& pos = _sdf->cylinder()->position();
        const Vec4r& ori = _sdf->cylinder()->orientation();
        gpu_sdf.position = make_float3(pos[0], pos[1], pos[2]);
        gpu_sdf.orientation = make_float4(ori[0], ori[1], ori[2], ori[3]);
        gpu_sdf.radius = _sdf->cylinder()->radius();
        gpu_sdf.height = _sdf->cylinder()->height();

        cudaMemcpy(_d_sdf, &gpu_sdf, sizeof(GPUCylinderSDF), cudaMemcpyHostToDevice);
    }

    virtual void partialCopyToDevice() const override
    {
        fullCopyToDevice();
    }

    GPUCylinderSDF* gpuSDF() const { return _d_sdf; }

    private:
    const Geometry::CylinderSDF* _sdf;
    GPUCylinderSDF* _d_sdf;

};


} // namespace Sim

#endif // __CYLINDER_SDF_GPU_RESOURCE_HPP