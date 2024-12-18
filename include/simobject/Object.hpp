#ifndef __OBJECT_HPP
#define __OBJECT_HPP

#include <string>

#include "geometry/AABB.hpp"
#include "config/ObjectConfig.hpp"

namespace Sim
{

class Simulation;

class Object
{
    public:
    Object(const Simulation* sim, const ObjectConfig* config)
        : _name(config->name()), _sim(sim)
    {}

    Object(const Simulation* sim, const std::string& name)
        : _name(name), _sim(sim)
    {
    }

    virtual ~Object() = default;

    /** Returns a string with all relevant information about this object. 
     * @param indent : the level of indentation to use for formatting new lines of the string
    */
    virtual std::string toString(const int indent) const
    {
        return std::string(indent, '\t') + "Name: " + _name;
    }
    
    /** Returns a string with the type of the object. */
    virtual std::string type() const { return "Object"; }

    /** Returns the name of this object. */
    std::string name() const { return _name; }

    /** Performs any necessary setup for this object.
     * Called after instantiation (i.e. outside the constructor) and before update() is called for the first time.
     */
    virtual void setup() = 0;

    /** Evolves this object one time step forward in time. 
     * Completely up to the derived classes to decide how they should step forward in time.
    */
    virtual void update() = 0;
    
    /** Returns the axis-aligned bounding-box (AABB) for this Object in global simulation coordinates. */
    virtual Geometry::AABB boundingBox() const = 0;

    protected:
    /** Name of the object */
    std::string _name;

    /** Pointer to the Simulation object that created this Object.
     * Usefule for querying things like current sim time or time step or acceleration due to gravity.
    */
    const Simulation* _sim;
};

} // namespace Simulation

#endif // __OBJECT_HPP