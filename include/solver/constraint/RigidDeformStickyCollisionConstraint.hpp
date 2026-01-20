#ifndef __RIGID_DEFORM_STICKY_COLLISION_CONSTRAINT_HPP
#define __RIGID_DEFORM_STICKY_COLLISION_CONSTRAINT_HPP

#include "solver/constraint/RigidDeformableCollisionConstraint.hpp"

namespace Solver
{

/**
 * A combined collision and adhesion constraint.
 * Instead of fighting between "Push out" (Collision) and "Pull in" (Adhesion),
 * this constraint manages a single distance target logic.
 * 
 * Logic:
 * 1. Checks current distance d with SDF.
 * 2. If adhering (is_broken=false):
 *      Constraint C(x) = d - rest_gap (Equality)
 *      This maintains adhesive gap at rest_gap distance.
 * 3. If broken (is_broken=true):
 *      Constraint C(x) = d >= 0 (Inequality)
 *      This acts as standard collision (separating contact).
 * 
 * Breaking mechanism:
 * - Strain = current_distance / rest_gap
 * - Breaks when strain > break_ratio (e.g., 1.5 = 150% of rest length)
 * 
 * Used to solve fighting between separate collision and adhesion constraints.
 */
class RigidDeformStickyCollisionConstraint : public RigidDeformableCollisionConstraint
{
public:
    RigidDeformStickyCollisionConstraint(const Geometry::SDF* sdf, Sim::RigidObject* rigid_obj, 
                                         const Vec3r& rigid_body_point, const Vec3r& collision_normal,
                                         int v1, Real* p1, Real m1,
                                         int v2, Real* p2, Real m2,
                                         int v3, Real* p3, Real m3,
                                         Real u, Real v, Real w,
                                         Real rest_gap,
                                         Real break_ratio = 1.5);

    virtual ~RigidDeformStickyCollisionConstraint() = default;

    // Enable Move Semantics (Required for std::vector resize)
    RigidDeformStickyCollisionConstraint(RigidDeformStickyCollisionConstraint&&) = default;
    RigidDeformStickyCollisionConstraint& operator=(RigidDeformStickyCollisionConstraint&&) = default;

    // Disable Copy (Inherited from RigidBodyConstraint)
    RigidDeformStickyCollisionConstraint(const RigidDeformStickyCollisionConstraint&) = delete;
    RigidDeformStickyCollisionConstraint& operator=(const RigidDeformStickyCollisionConstraint&) = delete;

    /** Returns false (Equality) when sticky, true (Inequality) when broken */
    bool isInequality() const override;

    /** Override evaluate to implement rest gap logic */
    void evaluate(Real* C) const override;

    /** Override gradient to support dynamic normal update */
    void gradient(Real* delC) const override;

    /** 
     * Mark the bond as broken. 
     * This moves the constraint from "Sticky" (Equality) mode 
     * to "Sliding/Repulsion" (Inequality) mode.
     */
    void setBroken(bool broken) { _is_broken = broken; }
    
    bool isBroken() const { return _is_broken; }
    
    /** Get rest gap distance (adhesive gap at equilibrium) */
    Real getRestGap() const { return _rest_gap; }
    
    /** Get break ratio (strain threshold for breaking, e.g., 1.5 = 150%) */
    Real getBreakRatio() const { return _break_ratio; }

    /** Sets the compliance (alpha) for this constraint */
    void setCompliance(Real compliance) { _alpha = compliance; }

    /** Enable/disable dynamic normal updates */
    void setDynamicNormalUpdate(bool enable) { _use_dynamic_normal = enable; }

    /** Get statistics for debugging */
    Real getMaxConstraintError() const { return _max_constraint_error_seen; }
    int getErrorCount() const { return _large_error_count; }
    
    /** Get current distance for breakage checking */
    Real getCurrentDistance() const;
    
    /** Get current strain ratio (current_distance / rest_gap) */
    Real getStrainRatio() const;

protected:
    bool _is_broken;
    Real _rest_gap;      // Equilibrium adhesive gap distance
    Real _break_ratio;   // Strain ratio threshold for breaking
    bool _use_dynamic_normal = false;  // Whether to update normal each frame
    
    // Debugging statistics
    mutable Real _max_constraint_error_seen = 0.0;
    mutable int _large_error_count = 0;
    mutable Vec3r _cached_normal;  // Cached dynamic normal
};

}

#endif
