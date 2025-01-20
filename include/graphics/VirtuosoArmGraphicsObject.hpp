#ifndef __VIRTUOSO_ARM_GRAPHICS_OBJECT_HPP
#define __VIRTUOSO_ARM_GRAPHICS_OBJECT_HPP

#include "graphics/GraphicsObject.hpp"
#include "simobject/VirtuosoArm.hpp"

namespace Graphics
{

class VirtuosoArmGraphicsObject : public GraphicsObject
{
    public:
    explicit VirtuosoArmGraphicsObject(const std::string& name, const Sim::VirtuosoArm* virtuoso_arm)
        : GraphicsObject(name), _virtuoso_arm(virtuoso_arm)
    {

    }

    const Sim::VirtuosoArm* virtuosoArm() const { return _virtuoso_arm; }

    protected:
    const Sim::VirtuosoArm* _virtuoso_arm;

};

} // namespace Graphics

#endif // __VIRTUOSO_ARM_GRAPHICS_OBJECT_HPP