#ifndef __CYLINDER_GRAPHICS_OBJECT_HPP
#define __CYLINDER_GRAPHICS_OBJECT_HPP

#include "graphics/GraphicsObject.hpp"
#include "simobject/RigidPrimitives.hpp"

namespace Graphics
{

class CylinderGraphicsObject : public GraphicsObject
{
    public:
    explicit CylinderGraphicsObject(const std::string& name, const Sim::RigidCylinder* cylinder)
        : GraphicsObject(name), _cylinder(cylinder)
    {
    }

    const Sim::RigidCylinder* cylinder() const { return _cylinder; }

    protected:
    const Sim::RigidCylinder* _cylinder;
};

} // namespace Graphics

#endif // __CYLINDER_GRAPHICS_OBJECT_HPP