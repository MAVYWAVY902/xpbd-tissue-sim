#ifndef __RIGID_OBJECT_HPP
#define __RIGID_OBJECT_HPP

#include "simobject/Object.hpp"
#include "config/RigidObjectConfig.hpp"

namespace Sim
{

class RigidObject : public Object
{
    public:
    RigidObject(const Simulation* sim, const RigidObjectConfig* config);

    RigidObject(const Simulation* sim, const std::string& name);

    RigidObject(const Simulation* sim, const std::string& name, const Eigen::Vector3d& position, const Eigen::Vector4d& orientation);

    RigidObject(const Simulation* sim, const std::string& name, const Eigen::Vector3d& position, const Eigen::Vector4d& orientation, const double mass, const Eigen::Matrix3d& inertia_mat);

    /** Returns a string with all relevant information about this object. 
     * @param indent : the level of indentation to use for formatting new lines of the string
    */
    virtual std::string toString(const int indent) const override;
    
    /** Returns a string with the type of the object. */
    virtual std::string type() const override { return "RigidObject"; }

    /** Evolves this object one time step forward in time. 
     * Completely up to the derived classes to decide how they should step forward in time.
    */
    virtual void update() override;
    
    /** Getters and setters */
    virtual void setLinearVelocity(const Eigen::Vector3d& lin_velocity) { _v = lin_velocity; }
    Eigen::Vector3d linearVelocity() const { return _v; }

    virtual void setAngularVelocity(const Eigen::Vector3d& ang_velocity) { _w = ang_velocity; }
    Eigen::Vector3d angularVelocity() const { return _w; }

    virtual void setPosition(const Eigen::Vector3d& position) { _p = position; }
    Eigen::Vector3d position() const { return _p; }

    virtual void setOrientation(const Eigen::Vector4d& orientation) { _q = orientation; }
    Eigen::Vector4d orientation() const { return _q; }

    double mass() const { return _m; }
    Eigen::Matrix3d I() const { return _I; }

    protected:
    /** Position of rigid body */
    Eigen::Vector3d _p;
    /** Orientation of rigid body, expressed as a quaternion. */
    Eigen::Vector4d _q;

    /** Total mass of rigid body. */
    double _m;
    /** Moment of inertia matrix. */
    Eigen::Matrix3d _I;
    

    /** Translational velocity of rigid body. */
    Eigen::Vector3d _v;
    /** Rotational velocity of rigid body. */
    Eigen::Vector3d _w;
    

};

} // namespace Simulation

#endif // __RIGID_OBJECT_HPP