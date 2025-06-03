#ifndef __VIRTUOSO_ARM_SDF_HPP
#define __VIRTUOSO_ARM_SDF_HPP

#include "geometry/SDF.hpp"
#include "geometry/TransformationMatrix.hpp"
// #include "simobject/VirtuosoArm.hpp"

namespace Sim
{
    class VirtuosoArm;
}

namespace Geometry
{

// TODO: fix SDF for revamped VirtuosoArm
class VirtuosoArmSDF : public SDF
{
    public:
    VirtuosoArmSDF(const Sim::VirtuosoArm* arm);

    /** Evaluates F(x) for a Virtuoso Arm in its current state.
     * @param x - the point at which to evaluate the SDF
     * @returns the distance from x to the shape boundary ( F(x) )
    */
    virtual Real evaluate(const Vec3r& x) const override;

    /** Evaluates the gradient of F at x.
     * @param x - the point at which to evaluate the graient of the SDF
     * @returns the gradient of the SDF at x.
     */
    virtual Vec3r gradient(const Vec3r& x) const override;

    #ifdef HAVE_CUDA
    virtual void createGPUResource() override { assert(0); /* not implemented */ }
    #endif

    private:
    Vec3r _bodyToGlobalInnerTubeGradient(const Vec3r& grad) const;
    Vec3r _globalToBodyInnerTube(const Vec3r& x) const;
    Real _cylinderSDFDistance(const Vec3r& x, Real cyl_radius, Real cyl_height) const;
    Vec3r _cylinderSDFGradient(const Vec3r& x, Real cyl_radius, Real cyl_height) const;

    private:
    const Sim::VirtuosoArm* _virtuoso_arm;
};

} // namespace Geometry


#endif // __VIRTUOSO_ARM_SDF_HPP