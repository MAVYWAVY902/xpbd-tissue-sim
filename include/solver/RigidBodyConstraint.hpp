#ifndef __RIGID_BODY_CONSTRAINT_HPP
#define __RIGID_BODY_CONSTRAINT_HPP

#include "solver/Constraint.hpp"
#include "simobject/RigidObject.hpp"
#include "utils/GeometryUtils.hpp"

// This file provides implementations for constraints that involve one or more rigid bodies.
// Based off the paper "Detailed Rigid Body Simulation with Extended Position Based Dynamics" by Muller et al. (2020)
// https://matthias-research.github.io/pages/publications/PBDBodies.pdf

namespace Solver
{

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** A helper class for evaluating the inertial "weight" of the rigid body in the lambda calculation, and calculating the rigid body position and orientation update
 *    given the delta lambda.
 * This class exists because the formula provided in the paper does not really fit with the original XPBD formula - the weight of the rigid body in the
 *   lambda update formula does not involve a constraint gradient, but instead has a special formula. This formula involves quantities from the constraint itself:
 *   i.e. the direction of the position correction and/or the point on the rigid body (in body coordinates).
 * In this codebase for normal XPBD, the evaluation of a constraint and its gradient are separated from the XPBD update formula (the notion of a constraint function
 *   and its gradient exists on its own, outside of XPBD). The XPBD update formula is implemented in the "ConstraintProjector" class.
 * However, the rigid body XPBD update formula needs constraint-specific quantities other than the constraint and its gradient, which poses a problem for the generic
 *   interface defined for constraints (how do we get the information from the constraint to the constraint projector?). What's more, the weight and update formulas
 *   change depending on the TYPE of constraint being implemented (i.e. positional or angular constraints), posing another problem for a generic interface.
 * The RigidBodyXPBDHelper provides a generic interface for the constraint projector to get the information it needs to perform the XPBD updates.
 * Each RigidBodyConstraint (defined below) will provide, for each rigid body involved in the constraint, a Helper class that the constraint projector (specifically
 *   the RigidBodyConstraintProjector) will use to compute the proper delta lambda and rigid body updates.
 */
class RigidBodyXPBDHelper
{
    public:
    RigidBodyXPBDHelper(const Sim::RigidObject* rigid_obj)
        : _rigid_obj(rigid_obj)
    {}

    /** The inertial "weight" of the rigid body in the update formula.
     * The update formula (for 2 rigid bodies) is
     * 
     * (-c - alpha*lambda) / (w1 + w2 + alpha)
     * 
     * where w1 and w2 are the inertial weights of the rigid body.
     */
    virtual double weight() const = 0;

    /** Computes the rigid body update (position and orientation) given the delta lambda computed with the XPBD formula.
     * @param dlam : the delta lambda
     * @param update_vec (OUTPUT) : a (currently empty) pointer to a 7x1 vector. The first 3 numbers are the position update, the next 4 are the orientation update (as a quaternion).
     */
    virtual void update(double dlam, double* update_vec) const = 0;

    protected:
    const Sim::RigidObject* _rigid_obj;
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** A helper class for evaluating the intertial weight and rigid body update of a rigid body involved in a "positional constraint" (as defined in the paper 
 * "Detailed Rigid Body Simulation with Extended Position Based Dynamics (2020)").
 */
class PositionalRigidBodyXPBDHelper : public RigidBodyXPBDHelper
{
    public:
    /** Takes in the direction of the correction and the point on the body in global coordinates. */
    PositionalRigidBodyXPBDHelper(const Sim::RigidObject* rigid_obj, const Eigen::Vector3d& direction, const Eigen::Vector3d& point_on_body)
        : RigidBodyXPBDHelper(rigid_obj), _direction(direction), _point_on_body(point_on_body)
    {}

    /** The inertial "weight" of the rigid body in the update formula.
     * For a rigid body in a positional constraint, this is given by:
     * 
     * w = 1/m + (r x n)^T * I^-1 * (r x n)
     * 
     * where m is the mass, r is the point on the rigid body (in the body frame), n is the direction of the correction (in the body frame), and I is the moment of inertia matrix
     * in the rest state.
     */
    virtual double weight() const override
    {
        // calculate the point on the rigid body in the body frame
        const Eigen::Vector3d r_body = _rigid_obj->globalToBody(_point_on_body);
        // calculate the direction of the correction in the body frame
        const Eigen::Vector3d n_body = GeometryUtils::rotateVectorByQuat(_direction, GeometryUtils::inverseQuat(_rigid_obj->orientation()));
        
        // use the defined formula to get the weight
        const Eigen::Vector3d r_cross_n = r_body.cross(n_body);
        return 1.0/_rigid_obj->mass() + r_cross_n.transpose() * _rigid_obj->invI() * r_cross_n;
    }

    /** Computes the global rigid body update (position and orientation) given the delta lambda computed with the XPBD formula.
     * For a rigid body in a positional constraint, this is given by:
     * 
     * position_update = dlam * n / m
     * orientation_update = 0.5 * [I^-1 * (r x (dlam * n_body)), 0] * q
     * 
     * where m is the mass, r is the point on the rigid body (in the body frame), n is the direction of the correction (in the global frame), I is the moment of inertia
     * in the rest state, n_body is the direction of the correction (in the body frame), and q is the current orientation of the body.
     * 
     * @param dlam : the delta lambda
     * @param update_vec (OUTPUT) : a (currently empty) pointer to a 7x1 vector. The first 3 numbers are the position update, the next 4 are the orientation update (as a quaternion).
     */
    virtual void update(double dlam, double* update_vec) const override
    {
        // calculate the point on the rigid body in the body frame
        const Eigen::Vector3d r_body = _rigid_obj->globalToBody(_point_on_body);
        // calculate the direction of the correction in the body frame
        const Eigen::Vector3d n_body = GeometryUtils::rotateVectorByQuat(_direction, GeometryUtils::inverseQuat(_rigid_obj->orientation()));
        
        // compute the position update (in the global frame)
        const Eigen::Vector3d position_update = dlam * _direction / _rigid_obj->mass();

        // compute the body angular velocity (with quantities in the body frame, since I is in the rest state i.e. body frame)
        const Eigen::Vector3d omega_body = 0.5 * _rigid_obj->invI() * (r_body.cross(dlam * n_body));
        // convert body anuglar velocity to spatial angular velocity
        const Eigen::Vector3d omega_spatial = GeometryUtils::rotateVectorByQuat(omega_body, _rigid_obj->orientation());
        // compute the orientation update (in the global frame)
        const Eigen::Vector4d orientation_update = GeometryUtils::quatMult(Eigen::Vector4d(omega_spatial[0], omega_spatial[1], omega_spatial[2], 0), _rigid_obj->orientation());
    
        // populate the update vector
        update_vec[0] = position_update[0];
        update_vec[1] = position_update[1];
        update_vec[2] = position_update[2];

        update_vec[3] = orientation_update[0];
        update_vec[4] = orientation_update[1];
        update_vec[5] = orientation_update[2];
        update_vec[6] = orientation_update[3];

    }

    protected:
    Eigen::Vector3d _direction;     // direction of correction in the global frame
    Eigen::Vector3d _point_on_body; // the point on the rigid body in the global frame
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/** A helper class for evaluating the intertial weight and rigid body update of a rigid body involved in a "angular constraint" (as defined in the paper 
 * "Detailed Rigid Body Simulation with Extended Position Based Dynamics (2020)").
 */
class AngularRigidBodyXPBDHelper : public RigidBodyXPBDHelper
{
    public:
    /** Takes in the rotation axis of the angular constraint in the global frame. */
    AngularRigidBodyXPBDHelper(const Sim::RigidObject* rigid_obj, const Eigen::Vector3d& rot_axis)
        : RigidBodyXPBDHelper(rigid_obj), _rot_axis(rot_axis)
    {}

    /** The inertial "weight" of the rigid body in the update formula.
     * For a rigid body in an angular constraint, this is given by:
     * 
     * w = n^T * I^-1 * n
     * 
     * where n is the rotation axis (in the body frame), and I is the moment of inertia matrix in the rest state (i.e. body frame).
     */
    virtual double weight() const override
    {
        // compute rotation axis in the body frame
        const Eigen::Vector3d rot_axis_body = GeometryUtils::rotateVectorByQuat(_rot_axis, GeometryUtils::inverseQuat(_rigid_obj->orientation()));
        // compute weight based on above formula
        return rot_axis_body.transpose() * _rigid_obj->invI() * rot_axis_body;
    }

    /** Computes the global rigid body update (position and orientation) given the delta lambda computed with the XPBD formula.
     * For a rigid body in an angular constraint, this is given by:
     * 
     * position_update = 0
     * orientation_update = 0.5 * [I^-1 * (dlam * n), 0] * q
     * 
     * where n is the rotation axis (in the body frame), and I is the moment of inertia matrix in the rest state (i.e. body frame).
     * 
     * @param dlam : the delta lambda
     * @param update_vec (OUTPUT) : a (currently empty) pointer to a 7x1 vector. The first 3 numbers are the position update, the next 4 are the orientation update (as a quaternion).
     */
    virtual void update(double dlam, double* update_vec) const override
    {
        // compute rotation axis in the body frame
        const Eigen::Vector3d rot_axis_body = GeometryUtils::rotateVectorByQuat(_rot_axis, GeometryUtils::inverseQuat(_rigid_obj->orientation()));
        // compute the body angular velocity (with quantities in the body frame, since I is in the rest state i.e. body frame)
        const Eigen::Vector3d omega_body = 0.5 * _rigid_obj->invI() * (dlam * rot_axis_body);
        // convert to global angular velocity of the body
        const Eigen::Vector3d omega_spatial = GeometryUtils::rotateVectorByQuat(omega_body, _rigid_obj->orientation());
        // compute orientation update
        const Eigen::Vector4d orientation_update = GeometryUtils::quatMult(Eigen::Vector4d(omega_spatial[0], omega_spatial[1], omega_spatial[2], 0), _rigid_obj->orientation());

        // populate the vector (position update is 0)
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


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/** Represents a constraint that involves one or more rigid bodies.
 * 
 * All constraints that involve a rigid body should inherit from this class.
 * 
 * The inclusion of rigid bodies in XPBD needs to be handled specially - based on "Detailed rigid Body Simulation with Extended Position Based Dynamics".
 * In two main ways: (1) the calculation of the XPBD Lagrange multiplier update formula and (2) the computation of the rigid body update.
 * 
 * For both of these computations, the RigidBodyConstraint class provides a RigidBodyXPBDHelper object for each rigid body involved in the constraint that
 * will provide the necessary quantities to perform these updates. It is the responsibility of the derived class to create the Helper classes.
*/
class RigidBodyConstraint// : public Constraint
{
    public:
    RigidBodyConstraint(/*const std::vector<PositionReference>& positions,*/ const std::vector<Sim::RigidObject*>& rigid_bodies, double alpha=0)
        : /*Constraint(positions, alpha),*/ _rigid_bodies(rigid_bodies), _rigid_body_helpers()
    {
    }

    const std::vector<Sim::RigidObject*> rigidBodies() const { return _rigid_bodies; }
    int numRigidBodies() const { return _rigid_bodies.size(); }

    const std::vector<std::unique_ptr<RigidBodyXPBDHelper>>& rigidBodyHelpers() const { return _rigid_body_helpers; }

    protected:
    std::vector<Sim::RigidObject*> _rigid_bodies;   // the rigid bodies involved in the constraint
    std::vector<std::unique_ptr<RigidBodyXPBDHelper> > _rigid_body_helpers; // the Helper objects that correspond to each rigid body
};

} // namespace Solver

#endif // __RIGID_BODY_CONSTRAINT_HPP