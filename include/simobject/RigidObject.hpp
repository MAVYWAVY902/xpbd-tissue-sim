#ifndef __RIGID_OBJECT_HPP
#define __RIGID_OBJECT_HPP

#include "simobject/Object.hpp"
#include "config/RigidObjectConfig.hpp"
#include "utils/GeometryUtils.hpp"

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

    virtual void velocityUpdate() override;
    
    /** Getters and setters */
    virtual void setGlobalLinearVelocity(const Eigen::Vector3d& lin_velocity) { _v = lin_velocity; }
    Eigen::Vector3d globalLinearVelocity() const { return _v; }
    Eigen::Vector3d bodyLinearVelocity() const { return GeometryUtils::rotateVectorByQuat(_v, GeometryUtils::inverseQuat(_q)); }

    virtual void setGlobalAngularVelocity(const Eigen::Vector3d& ang_velocity) { _w = ang_velocity; }
    Eigen::Vector3d globalAngularVelocity() const { return _w; }
    Eigen::Vector3d bodyAngularVelocity() const { return GeometryUtils::rotateVectorByQuat(_w, GeometryUtils::inverseQuat(_q)); }

    virtual void setPosition(const Eigen::Vector3d& position) { if (!_fixed) _p = position; }
    Eigen::Vector3d position() const { return _p; }
    Eigen::Vector3d prevPosition() const { return _p_prev; }

    virtual void setOrientation(const Eigen::Vector4d& orientation) { if (!_fixed) _q = orientation; }
    Eigen::Vector4d orientation() const { return _q; }
    Eigen::Vector4d prevOrientation() const { return _q_prev; }

    double mass() const { return _m; }
    Eigen::Matrix3d I() const { return _I; }
    Eigen::Matrix3d invI() const { return _I_inv; }

    bool isFixed() const { return _fixed; }

    /** Applies the force f to the rigid body at point p for a single time step.
     * @param f : the force vector in global coordinates
     * @param p : the point at which to apply the force to the rigid body, in global coordinates
     */
    void applyForceAtPoint(const Eigen::Vector3d& f, const Eigen::Vector3d& p);

    /** Returns the coordinates of p (which is specified in global coords) w.r.t the current body frame of this rigid object.
     * @param p : the point expressed in world frame coords
     * @returns the point p expressed in the current body-frame coordinates
    */
    Eigen::Vector3d globalToBody(const Eigen::Vector3d& p) const;

    /** Returns the coordinates of p (which is specified in body-frame coords) w.r.t the global XYZ coordinate system.
     * @param p : a point in body-frame coordinates
     * @returns the point p expressed in world frame coords
    */
    Eigen::Vector3d bodyToGlobal(const Eigen::Vector3d& p) const;

    protected:
    /** Position of rigid body */
    Eigen::Vector3d _p;
    /** Preious position of rigid body. */
    Eigen::Vector3d _p_prev;
    /** Orientation of rigid body, expressed as a quaternion. */
    Eigen::Vector4d _q;
    /** Previous orientation of rigid body. */
    Eigen::Vector4d _q_prev;

    /** Total mass of rigid body. */
    double _m;
    /** Moment of inertia matrix. */
    Eigen::Matrix3d _I;
    /** Inerse of moment of inertia matrix. */
    Eigen::Matrix3d _I_inv;
    

    /** Translational velocity of rigid body in the global frame */
    Eigen::Vector3d _v;
    /** Rotational velocity of rigid body in the global frame */
    Eigen::Vector3d _w;

    /** If the rigid body is fixed (i.e. can't move or rotate). */
    bool _fixed;
    

};

} // namespace Simulation

#endif // __RIGID_OBJECT_HPP