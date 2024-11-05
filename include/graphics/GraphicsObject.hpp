#ifndef __GRAPHICS_OBJECT_HPP
#define __GRAPHICS_OBJECT_HPP

#include <string>
#include <iostream>

namespace Graphics
{

class GraphicsObject 
{
    public:
    explicit GraphicsObject(const std::string& name)
        : _name(name) {}

    virtual ~GraphicsObject() {};

    virtual void update() = 0;

    std::string name() const { return _name; }

    protected:
    std::string _name;
};

} // namespace Graphics

#endif // __GRAPHICS_OBJECT_HPP