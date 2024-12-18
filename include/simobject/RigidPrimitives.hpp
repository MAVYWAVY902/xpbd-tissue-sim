#ifndef __RIGID_PRIMITIVES_HPP
#define __RIGID_PRIMITIVES_HPP

#include "simobject/RigidObject.hpp"
#include "config/RigidPrimitiveConfigs.hpp"

namespace Sim
{

class RigidSphere : public RigidObject
{
    public:
    RigidSphere(const Simulation* sim, const RigidSphereConfig* config);

    /** Returns a string with all relevant information about this object. 
     * @param indent : the level of indentation to use for formatting new lines of the string
    */
    virtual std::string toString(const int indent) const override;
    
    /** Returns a string with the type of the object. */
    virtual std::string type() const override { return "RigidObject"; }

    double radius() const { return _radius; }

    virtual void setup() override;

    virtual Geometry::AABB boundingBox() const override;

    protected:
    double _radius;

};


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

class RigidBox : public RigidObject
{
    public:
    RigidBox(const Simulation* sim, const RigidBoxConfig* config);

    /** Returns a string with all relevant information about this object. 
     * @param indent : the level of indentation to use for formatting new lines of the string
    */
    virtual std::string toString(const int indent) const override;
    
    /** Returns a string with the type of the object. */
    virtual std::string type() const override { return "RigidObject"; }

    Eigen::Vector3d size() const { return _size; }

    virtual void setup() override;

    virtual Geometry::AABB boundingBox() const override;
    
    protected:
    Eigen::Vector3d _size;
    Eigen::Matrix<double, 3, 8> _origin_bbox_points;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

class RigidCylinder : public RigidObject
{
    public:
    RigidCylinder(const Simulation* sim, const RigidCylinderConfig* config);

    /** Returns a string with all relevant information about this object. 
     * @param indent : the level of indentation to use for formatting new lines of the string
    */
    virtual std::string toString(const int indent) const override;
    
    /** Returns a string with the type of the object. */
    virtual std::string type() const override { return "RigidObject"; }

    double radius() const { return _radius; }

    double height() const { return _height; }

    virtual void setup() override;

    virtual Geometry::AABB boundingBox() const override;

    protected:
    double _radius;
    double _height;
    Eigen::Matrix<double, 3, 8> _origin_bbox_points;
};

} //namespace Sim

#endif // __RIGID_PRIMITIVES_HPP