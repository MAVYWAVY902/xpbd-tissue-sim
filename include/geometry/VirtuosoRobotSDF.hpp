#ifndef __VIRTUOSO_ROBOT_SDF_HPP
#define __VIRTUOSO_ROBOT_SDF_HPP

#include "geometry/VirtuosoArmSDF.hpp"

#include <limits>

namespace Sim
{
    class VirtuosoRobot;
}

namespace Geometry
{

class VirtuosoRobotSDF : public SDF
{
    public:
    VirtuosoRobotSDF(Sim::VirtuosoRobot* virtuoso_robot);
    

    virtual Real evaluate(const Vec3r& x) const override;

    virtual Vec3r gradient(const Vec3r& x) const override;

    private:
    const Sim::VirtuosoRobot* _virtuoso_robot;

    const Geometry::VirtuosoArmSDF* _arm1_sdf;
    const Geometry::VirtuosoArmSDF* _arm2_sdf;
};

} // namespace Geometry

#endif // __VIRTUOSO_ROBOT_SDF_HPP