#include "solver/constraint/RigidDeformStickyCollisionConstraint.hpp"

namespace Solver
{

RigidDeformStickyCollisionConstraint::RigidDeformStickyCollisionConstraint(
    const Geometry::SDF* sdf, Sim::RigidObject* rigid_obj, 
    const Vec3r& rigid_body_point, const Vec3r& collision_normal,
    int v1, Real* p1, Real m1,
    int v2, Real* p2, Real m2,
    int v3, Real* p3, Real m3,
    Real u, Real v, Real w,
    Real rest_gap,
    Real break_ratio)
    : RigidDeformableCollisionConstraint(sdf, rigid_obj, rigid_body_point, collision_normal, 
                                         v1, p1, m1, 
                                         v2, p2, m2, 
                                         v3, p3, m3, 
                                         u, v, w),
      _is_broken(false), 
      _rest_gap(rest_gap),
      _break_ratio(break_ratio)
{
}

bool RigidDeformStickyCollisionConstraint::isInequality() const
{
    // ALWAYS use inequality constraint (C >= 0)
    // In sticky mode: Only resists stretching (one-way pull constraint)
    // In broken mode: Standard collision (push out when penetrating)
    return true;  // Always inequality
}

void RigidDeformStickyCollisionConstraint::evaluate(Real* C) const
{
    // Get the point on the deformable body from barycentric coordinates
    const Vec3r a = _u*Eigen::Map<Vec3r>(_positions[0].position_ptr) 
                  + _v*Eigen::Map<Vec3r>(_positions[1].position_ptr) 
                  + _w*Eigen::Map<Vec3r>(_positions[2].position_ptr);
    
    // Get current distance from SDF
    Real current_distance = _sdf->evaluate(a);
    
    if (_is_broken) {
        // Broken mode: Standard collision constraint C = d (inequality: C >= 0)
        // Pushes out when penetrating (d < 0)
        *C = current_distance;
    } else {
        // ========== Sticky mode: ONE-WAY PULL CONSTRAINT ==========
        // Key insight: Adhesion should ONLY resist stretching, not compression
        // - When stretched (d > rest_gap): Apply resistance (spring-like)
        // - When compressed (d <= rest_gap): Return C=0 (no constraint force)
        
        Real stretch = current_distance - _rest_gap;
        
        if (stretch <= 0.0) {
            // Compressed or at rest: NO constraint force
            // This allows the tissue to freely move closer to the bone
            // Separate collision constraint will handle penetration
            *C = 0.0;  // Inactive constraint (inequality: C >= 0 satisfied)
            
            // Keep compliance soft for inactive state
            const_cast<RigidDeformStickyCollisionConstraint*>(this)->_alpha = 1e-3;
            return;
        }
        
        // Stretching: Apply resistance proportional to stretch amount
        // This is the core adhesion behavior
        
        // Track maximum stretch for debugging
        if (stretch > _max_constraint_error_seen) {
            _max_constraint_error_seen = stretch;
        }
        
        // ========== Non-linear Compliance (Progressive Stiffening) ==========
        // Calculate strain ratio: how much is the adhesion stretched?
        Real strain_ratio = current_distance / _rest_gap;
        
        // Adjust compliance based on strain - stiffer as it stretches
        // VERY STIFF values (1e-5 level) to provide REAL adhesion resistance
        if (strain_ratio < 1.05) {
            // Small stretch (<5%): Moderate resistance
            const_cast<RigidDeformStickyCollisionConstraint*>(this)->_alpha = 1e-4;
        } else if (strain_ratio < 1.15) {
            // Medium stretch (5-15%): Strong resistance
            const_cast<RigidDeformStickyCollisionConstraint*>(this)->_alpha = 5e-5;
        } else if (strain_ratio < 1.3) {
            // Large stretch (15-30%): Very strong resistance
            const_cast<RigidDeformStickyCollisionConstraint*>(this)->_alpha = 3e-5;
        } else if (strain_ratio < 1.6) {
            // Near breaking (30-60%): Extreme resistance
            const_cast<RigidDeformStickyCollisionConstraint*>(this)->_alpha = 2e-5;
        } else {
            // About to break (>60%): Maximum stiffness
            const_cast<RigidDeformStickyCollisionConstraint*>(this)->_alpha = 1e-5;
        }
        
        // Return the stretch amount as constraint value (must be > 0)
        // XPBD will apply correction to reduce this (compliance determines strength)
        *C = stretch;
    }
}

void RigidDeformStickyCollisionConstraint::gradient(Real* delC) const
{
    // If dynamic normal update is enabled, recompute normal based on current position
    const Vec3r* normal_to_use = &_collision_normal;
    
    if (_use_dynamic_normal && !_is_broken) {
        // Compute current point position
        const Vec3r a = _u*Eigen::Map<Vec3r>(_positions[0].position_ptr) 
                      + _v*Eigen::Map<Vec3r>(_positions[1].position_ptr) 
                      + _w*Eigen::Map<Vec3r>(_positions[2].position_ptr);
        
        // Get current normal from SDF gradient
        _cached_normal = _sdf->gradient(a);
        
        // Normalize if needed
        Real norm = _cached_normal.norm();
        if (norm > 1e-8) {
            _cached_normal /= norm;
            normal_to_use = &_cached_normal;
        }
        // else: keep using original normal if current one is invalid
    }
    
    // Compute gradient: dC/dv_i = barycentric_weight_i * normal
    delC[0] = _u * (*normal_to_use)[0];
    delC[1] = _u * (*normal_to_use)[1];
    delC[2] = _u * (*normal_to_use)[2];

    delC[3] = _v * (*normal_to_use)[0];
    delC[4] = _v * (*normal_to_use)[1];
    delC[5] = _v * (*normal_to_use)[2];
    
    delC[6] = _w * (*normal_to_use)[0];
    delC[7] = _w * (*normal_to_use)[1];
    delC[8] = _w * (*normal_to_use)[2];
}

Real RigidDeformStickyCollisionConstraint::getCurrentDistance() const
{
    // Get the point on the deformable body from barycentric coordinates
    const Vec3r a = _u*Eigen::Map<Vec3r>(_positions[0].position_ptr) 
                  + _v*Eigen::Map<Vec3r>(_positions[1].position_ptr) 
                  + _w*Eigen::Map<Vec3r>(_positions[2].position_ptr);
    
    // Get current distance from SDF
    return _sdf->evaluate(a);
}

Real RigidDeformStickyCollisionConstraint::getStrainRatio() const
{
    Real current_distance = getCurrentDistance();
    return (_rest_gap > 1e-8) ? (current_distance / _rest_gap) : 1.0;
}

// Note: gradient() now supports dynamic normal updates for improved accuracy
// It correctly returns dC/dx = barycentric_weight * collision_normal
// This works for both modes since gradient structure is the same

}
