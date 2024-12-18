#ifndef __BOX_GRAPHICS_OBJECT_HPP
#define __BOX_GRAPHICS_OBJECT_HPP

#include "graphics/GraphicsObject.hpp"
#include "simobject/RigidPrimitives.hpp"

namespace Graphics
{

class BoxGraphicsObject : public GraphicsObject
{
    public:
    explicit BoxGraphicsObject(const std::string& name, const Sim::RigidBox* box)
        : GraphicsObject(name), _box(box)
    {
    }

    const Sim::RigidBox* box() const { return _box; }

    protected:
    const Sim::RigidBox* _box;
};

} // namespace Graphics

#endif // __BOX_GRAPHICS_OBJECT_HPP