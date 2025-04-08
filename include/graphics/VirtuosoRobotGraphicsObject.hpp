#ifndef __VIRTUOSO_ROBOT_GRAPHICS_OBJECT_HPP
#define __VIRTUOSO_ROBOT_GRAPHICS_OBJECT_HPP

#include "simobject/VirtuosoRobot.hpp"
#include "graphics/GraphicsObject.hpp"

namespace Graphics
{

class VirtuosoRobotGraphicsObject : public GraphicsObject
{
    public:
    explicit VirtuosoRobotGraphicsObject(const std::string& name, const Sim::VirtuosoRobot* virtuoso_robot)
        : GraphicsObject(name), _virtuoso_robot(virtuoso_robot)
    {

    }

    const Sim::VirtuosoRobot* virtuosoRobot() const { return _virtuoso_robot; }

    protected:
    const Sim::VirtuosoRobot* _virtuoso_robot;

};

} // namespace Graphics

#endif // __VIRTUOSO_ROBOT_GRAPHICS_OBJECT_HPP