#ifndef __VIRTUOSO_ARM_SDF_HPP
#define __VIRTUOSO_ARM_SDF_HPP

#include "geometry/SDF.hpp"
#include "simobject/VirtuosoArm.hpp"

namespace Geometry
{

class VirtuosoArmSDF : public SDF
{
    public:
    VirtuosoArmSDF(const Sim::VirtuosoArm* arm)
        : _virtuoso_arm(arm)
    {
    }

    /** Evaluates F(x) for a Virtuoso Arm in its current state.
     * @param x - the point at which to evaluate the SDF
     * @returns the distance from x to the shape boundary ( F(x) )
    */
    virtual double evaluate(const Eigen::Vector3d& x) const override
    {
        return 0.0;
    }

    /** Evaluates the gradient of F at x.
     * @param x - the point at which to evaluate the graient of the SDF
     * @returns the gradient of the SDF at x.
     */
    virtual Eigen::Vector3d gradient(const Eigen::Vector3d& x) const override
    {
        return Eigen::Vector3d::Zero();
    }

    private:
    const Sim::VirtuosoArm* _virtuoso_arm;
};

} // namespace Geometry


#endif // __VIRTUOSO_ARM_SDF_HPP