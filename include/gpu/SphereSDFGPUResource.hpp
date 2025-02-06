#ifndef __SPHERE_SDF_GPU_RESOURCE_HPP
#define __SPHERE_SDF_GPU_RESOURCE_HPP

#include "gpu/GPUResource.hpp"
#include "gpu/GPUStructs.hpp"

#include "geometry/SphereSDF.hpp"

#include "utils/CudaHelperMath.h"

namespace Sim
{

class SphereSDFGPUResource : public HostReadableGPUResource
{
    public:
    explicit SphereSDFGPUResource(const Geometry::SphereSDF* sphere_sdf)
        : _sdf(sphere_sdf)
    {

    }

    virtual ~SphereSDFGPUResource()
    {
        cudaFree(_d_sdf);
    }

    virtual void allocate() override
    {
        cudaMalloc((void**)&_d_sdf, sizeof(GPUSphereSDF));
    }

    virtual void copyToDevice() const override
    {
        GPUSphereSDF gpu_sdf;
        const Vec3r& pos = _sdf->sphere()->position();
        gpu_sdf.position = make_float3(pos[0], pos[1], pos[2]);
        gpu_sdf.radius = _sdf->sphere()->radius();

        cudaMemcpy(_d_sdf, &gpu_sdf, sizeof(GPUSphereSDF), cudaMemcpyHostToDevice);
    }

    GPUSphereSDF* gpuSDF() const { return _d_sdf; }

    private:
    const Geometry::SphereSDF* _sdf;
    GPUSphereSDF* _d_sdf;

};

} // namespace Sim

#endif // __SPHERE_SDF_GPU_RESOURCE_HPP