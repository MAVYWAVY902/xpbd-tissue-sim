#ifndef __RIGID_BODY_CONSTRAINT_HPP
#define __RIGID_BODY_CONSTRAINT_HPP

#include "solver/Constraint.hpp"
#include "simobject/RigidObject.hpp"
#include "utils/GeometryUtils.hpp"

namespace Solver
{

class RigidBodyXPBDHelper
{
    public:
    RigidBodyXPBDHelper(const Sim::RigidObject* rigid_obj)
        : _rigid_obj(rigid_obj)
    {}

    virtual double weight() const = 0;

    virtual void update(double dlam, double* update_vec) const = 0;

    protected:
    const Sim::RigidObject* _rigid_obj;
};

class PositionalRigidBodyXPBDHelper : public RigidBodyXPBDHelper
{
    public:
    PositionalRigidBodyXPBDHelper(const Sim::RigidObject* rigid_obj, const Eigen::Vector3d& direction, const Eigen::Vector3d& point_on_body)
        : RigidBodyXPBDHelper(rigid_obj), _direction(direction), _point_on_body(point_on_body)
    {}

    virtual double weight() const override
    {
        const Eigen::Vector3d r_body = _rigid_obj->globalToBody(_point_on_body);
        const Eigen::Vector3d n_body = GeometryUtils::rotateVectorByQuat(_direction, GeometryUtils::inverseQuat(_rigid_obj->orientation()));
        const Eigen::Vector3d r_cross_n = r_body.cross(n_body);
        return 1.0/_rigid_obj->mass() + r_cross_n.transpose() * _rigid_obj->invI() * r_cross_n;
    }

    virtual void update(double dlam, double* update_vec) const override
    {
        const Eigen::Vector3d r_body = _rigid_obj->globalToBody(_point_on_body);
        const Eigen::Vector3d n_body = GeometryUtils::rotateVectorByQuat(_direction, GeometryUtils::inverseQuat(_rigid_obj->orientation()));
        const Eigen::Vector3d position_update = dlam * _direction / _rigid_obj->mass();

        const Eigen::Vector3d omega_body = 0.5 * _rigid_obj->invI() * (r_body.cross(dlam * n_body));
        const Eigen::Vector3d omega_spatial = GeometryUtils::rotateVectorByQuat(omega_body, GeometryUtils::inverseQuat(_rigid_obj->orientation()));
        const Eigen::Vector4d orientation_update = GeometryUtils::quatMult(Eigen::Vector4d(omega_spatial[0], omega_spatial[1], omega_spatial[2], 0), _rigid_obj->orientation());
    
        update_vec[0] = position_update[0];
        update_vec[1] = position_update[1];
        update_vec[2] = position_update[2];

        update_vec[3] = orientation_update[0];
        update_vec[4] = orientation_update[1];
        update_vec[5] = orientation_update[2];
        update_vec[6] = orientation_update[3];

    }

    protected:
    Eigen::Vector3d _direction;
    Eigen::Vector3d _point_on_body;
};

class AngularRigidBodyXPBDHelper : public RigidBodyXPBDHelper
{
    public:
    AngularRigidBodyXPBDHelper(const Sim::RigidObject* rigid_obj, const Eigen::Vector3d& rot_axis)
        : RigidBodyXPBDHelper(rigid_obj), _rot_axis(rot_axis)
    {}

    virtual double weight() const override
    {
        const Eigen::Vector3d rot_axis_body = GeometryUtils::rotateVectorByQuat(_rot_axis, GeometryUtils::inverseQuat(_rigid_obj->orientation()));
        return rot_axis_body.transpose() * _rigid_obj->invI() * rot_axis_body;
    }

    virtual void update(double dlam, double* update_vec) const override
    {
        const Eigen::Vector3d rot_axis_body = GeometryUtils::rotateVectorByQuat(_rot_axis, GeometryUtils::inverseQuat(_rigid_obj->orientation()));
        const Eigen::Vector3d omega_body = 0.5 * _rigid_obj->invI() * (dlam * rot_axis_body);
        const Eigen::Vector3d omega_spatial = GeometryUtils::rotateVectorByQuat(omega_body, GeometryUtils::inverseQuat(_rigid_obj->orientation()));
        const Eigen::Vector4d orientation_update = GeometryUtils::quatMult(Eigen::Vector4d(omega_spatial[0], omega_spatial[1], omega_spatial[2], 0), _rigid_obj->orientation());

        update_vec[0] = 0;
        update_vec[1] = 0;
        update_vec[2] = 0;

        update_vec[3] = orientation_update[0];
        update_vec[4] = orientation_update[1];
        update_vec[5] = orientation_update[2];
        update_vec[6] = orientation_update[3];
    }

    protected:
    Eigen::Vector3d _rot_axis;

};

class RigidBodyConstraint : public Constraint
{
    public:
    RigidBodyConstraint(const std::vector<PositionReference>& positions, const std::vector<Sim::RigidObject*>& rigid_bodies, double alpha=0)
        : Constraint(positions, alpha), _rigid_bodies(rigid_bodies), _rigid_body_helpers()
    {
    }

    const std::vector<Sim::RigidObject*> rigidBodies() const { return _rigid_bodies; }
    int numRigidBodies() const { return _rigid_bodies.size(); }

    const std::vector<std::unique_ptr<RigidBodyXPBDHelper>>& rigidBodyHelpers() const { return _rigid_body_helpers; }

    protected:
    std::vector<Sim::RigidObject*> _rigid_bodies;
    std::vector<std::unique_ptr<RigidBodyXPBDHelper> > _rigid_body_helpers;
};

} // namespace Solver

#endif // __RIGID_BODY_CONSTRAINT_HPP