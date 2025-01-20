#ifndef __SPHERE_GRAPHICS_OBJECT_HPP
#define __SPHERE_GRAPHICS_OBJECT_HPP

#include "graphics/GraphicsObject.hpp"
#include "simobject/RigidPrimitives.hpp"

namespace Graphics
{

class SphereGraphicsObject : public GraphicsObject
{
    public:
    explicit SphereGraphicsObject(const std::string& name, const Sim::RigidSphere* sphere)
        : GraphicsObject(name), _sphere(sphere)
    {}
    
    const Sim::RigidSphere* sphere() const { return _sphere; }

    protected:
    const Sim::RigidSphere* _sphere;

};

} // namespace Graphics

#endif // __SPHERE_GRAPHICS_OBJECT_HPP