#ifndef __SDF_HPP
#define __SDF_HPP

#include "common/types.hpp"

#ifdef HAVE_CUDA
#include <memory>
#include "gpu/resource/GPUResource.hpp"
#endif

namespace Geometry
{

/** Abstract interface for a signed distance field (SDF).
 * A SDF is just a function F(x) that returns the distance to a shape boundary, where F(x) = 0.
 * Inside the shape, the distance is negative (hence it being a "signed" distance function).
 * Outside the shape, the distance is positive.
 * The gradient of the function has magnitude 1 and is in the direction of increasing distance.
 */
class SDF
{
    public:
    virtual ~SDF() = default;

    /** Evaluates F(x) - i.e. finds the distance from point x to the shape boundary.
     * @param x - the point at which to evaluate the SDF
     * @returns the distance from x to the shape boundary ( F(x) )
     */
    virtual Real evaluate(const Vec3r& x) const = 0;

    /** Evaluates the gradient of F at x.
     * @param x - the point at which to evaluate the graient of the SDF
     * @returns the gradient of the SDF at x.
     */
    virtual Vec3r gradient(const Vec3r& x) const = 0;

 #ifdef HAVE_CUDA
    virtual void createGPUResource() = 0;
    virtual const Sim::HostReadableGPUResource* gpuResource() const { assert(_gpu_resource); return _gpu_resource.get(); }
 #endif

    protected:
 #ifdef HAVE_CUDA
    std::unique_ptr<Sim::HostReadableGPUResource> _gpu_resource;
 #endif
};

} // namespace Geometry

#endif // __SDF_HPP