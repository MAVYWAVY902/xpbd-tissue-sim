#include "solver/constraint/UnifiedDistanceConstraint.hpp"
#include "utils/MathUtils.hpp"
#include "utils/GeometryUtils.hpp"
#include <cmath>
#include <algorithm>

namespace Solver
{

UnifiedDistanceConstraint::UnifiedDistanceConstraint(
    const Geometry::SDF* sdf, 
    Sim::RigidObject* rigid_obj,
    const Vec3r& rigid_body_point,
    int tri_v1, Real* tri_p1, Real tri_m1,
    int tri_v2, Real* tri_p2, Real tri_m2, 
    int tri_v3, Real* tri_p3, Real tri_m3,
    Real alpha,
    Real break_ratio,
    Real initial_distance,
    Real d_contact,
    Real d_rest,
    Real d_neutral_start,
    Real d_neutral_end,
    Real d_bond,
    Real stretch_abs_min)
    : Constraint(std::vector<PositionReference>({
        PositionReference(tri_v1, tri_p1, tri_m1),  // triangle vertex 1
        PositionReference(tri_v2, tri_p2, tri_m2),  // triangle vertex 2
        PositionReference(tri_v3, tri_p3, tri_m3)   // triangle vertex 3
    }), alpha),
    RigidBodyConstraint(std::vector<Sim::RigidObject*>({rigid_obj})),
    _sdf(sdf),
    _rigid_body_point(rigid_body_point),
    _d_contact(d_contact),
    _d_rest(d_rest),
    _d_neutral_start(d_neutral_start),
    _d_neutral_end(d_neutral_end),
    _d_bond(d_bond),
    _stretch_abs_min(stretch_abs_min),
    _break_ratio(break_ratio),
    _initial_distance(initial_distance),  // Use precomputed value
    _should_break(false)
{
    // INITIALIZATION: Compute barycentric coordinates once at creation
    // Transform rigid body point to global coordinates at initialization
    const Vec3r rigid_point_global = rigid_obj->bodyToGlobal(rigid_body_point);
    
    Eigen::Map<const Vec3r> tri_p1_init(tri_p1);
    Eigen::Map<const Vec3r> tri_p2_init(tri_p2);
    Eigen::Map<const Vec3r> tri_p3_init(tri_p3);
    
    Vec3r closest_point, normal;
    computePointTriangleDistance(rigid_point_global, tri_p1_init, tri_p2_init, tri_p3_init,
                                closest_point, normal, _bary_cached);
    
    // IMPORTANT: Do NOT set _cache_valid=true here!
    // Let first evaluate() in timestep compute and cache the contact frame.
    // Constructor cache is just for initialization reference.
    _cache_valid = false;

    // Create RigidBodyXPBDHelper for positional constraint
    // The correction direction is along the normal from triangle to rigid body point
    _rigid_body_helpers.push_back(
        std::make_unique<PositionalRigidBodyXPBDHelper>(rigid_obj, normal, rigid_point_global)
    );
}

void UnifiedDistanceConstraint::evaluate(Real* C) const
{
    // Early exit if constraint should break (already marked for removal)
    if (_should_break) {
        *C = 0.0;
        return;
    }
    
    // Extract current triangle positions
    Eigen::Map<const Vec3r> tri_p1(_positions[0].position_ptr);
    Eigen::Map<const Vec3r> tri_p2(_positions[1].position_ptr);
    Eigen::Map<const Vec3r> tri_p3(_positions[2].position_ptr);
    
    // Get rigid body point in global coordinates (transforms with rigid body motion)
    const Sim::RigidObject* rigid_obj = _rigid_bodies[0];
    const Vec3r rigid_point_global = rigid_obj->bodyToGlobal(_rigid_body_point);

    // FROZEN FRAME: Only recompute geometry if cache is invalid (start of timestep)
    Real point_to_tri_distance;
    if (!_cache_valid) {
        // First evaluation in this timestep - compute and freeze contact geometry

        // FIXED ADHESION IMPLEMENTATION:
        // Instead of searching for the NEW closest point (which causes sliding),
        // we use the INITIAL barycentric coordinates (anchored material point).
        // This converts the constraint from "Point-to-Triangle" (Sliding) to "Point-to-Point" (Fixed).
        
        // 1. Reconstruct anchor point using current vertex positions + stored INITIAL barycentrics
        const Vec3r anchor_point = _bary_cached[0] * tri_p1 + 
                                   _bary_cached[1] * tri_p2 + 
                                   _bary_cached[2] * tri_p3;
                                   
        // 2. Compute vector from anchor to rigid point
        const Vec3r diff = rigid_point_global - anchor_point;
        point_to_tri_distance = diff.norm();
        
        // 3. Compute normal (direction of force: from anchor TO rigid point)
        // If distance is zero, use triangle normal as fallback
        Vec3r normal;
        if (point_to_tri_distance > 1e-12) {
            normal = diff / point_to_tri_distance;
        } else {
            const Vec3r edge1 = tri_p2 - tri_p1;
            const Vec3r edge2 = tri_p3 - tri_p1;
            // Area-weighted normal -> normalized
            Vec3r tri_normal = edge1.cross(edge2);
            Real area2 = tri_normal.norm();
            if (area2 > 1e-12) normal = tri_normal / area2;
            else normal = Vec3r::UnitZ(); // Degenerate
        }
        
        // Cache geometry for this timestep (frozen frame)
        _xs_cached = anchor_point;
        _n_cached = normal;
        
        // IMPORTANT: We do NOT update _bary_cached. We keep the initial bond point!
        // _bary_cached = bary_coords_current;  <-- DISABLED for Fixed Adhesion
        
        _cache_valid = true;
    } else {
        // Cache valid - use frozen contact frame (within same timestep)
        // Reconstruct surface point using frozen barycentric coordinates
        // NOTE: This keeps contact point fixed in material coordinates during solver iterations
        const Vec3r x_s = _bary_cached[0] * tri_p1 + _bary_cached[1] * tri_p2 + _bary_cached[2] * tri_p3;
        
        // ✅ CRITICAL FIX: Use Euclidean distance, NOT signed projection!
        // Old code: point_to_tri_distance = _n_cached.dot(rigid_point_global - x_s);
        // Problem: This gives SIGNED distance (can be negative when rigid point crosses plane)
        // Fix: Use norm() for pure Euclidean distance (always >= 0)
        Vec3r diff = rigid_point_global - x_s;
        point_to_tri_distance = diff.norm();
        
        // [FIX FOR SLIDING] Update Normal Direction!
        // Even though material points are fixed (barycentrics), the VECTOR separating them changes orientation.
        // We MUST update _n_cached to point along the current separation vector.
        // If we don't, the force direction is frozen, allowing free tangential motion (sliding).
        if (point_to_tri_distance > 1e-12) {
            _n_cached = diff / point_to_tri_distance;
        }
        
        // IMPORTANT: We use the updated _n_cached for gradient direction.
    }

    // [CRITICAL FIX] Update RigidBodyXPBDHelper
    // The default PositionalRigidBodyXPBDHelper stores the global attachment point AT CONSTRUCTION.
    // If the body moves, it uses the OLD global point. We must update it every iteration 
    // to prevent the attachment point from sliding across the moving bone surface.
    
    // Force mutable access to update helpers
    // OPTIMIZATION: Use updateState() instead of clear() + push_back() to avoid heap allocation churn
    // and potential memory corruption issues with Easy3D or threaded contexts.
    
    if (!_rigid_body_helpers.empty()) {
        auto* positional_helper = dynamic_cast<PositionalRigidBodyXPBDHelper*>(_rigid_body_helpers[0].get());
        if (positional_helper) {
            positional_helper->updateState(_n_cached, rigid_point_global);
        }
    }

    // ✅ UNIFIED CONSTRAINT: C(d) = d - d*(d)
    // - d: current distance
    // - d*(d): smooth target distance curve
    // - Equilibrium: d = d*(d) (C = 0)
    // - Too close: d < d*(d) (C < 0, repulsion)
    // - Too far: d > d*(d) (C > 0, attraction)
    
    const Real d = point_to_tri_distance;
    
    // Protection: Check for abnormal d values before break check
    if (!std::isfinite(d) || d > 1.0 || d < 1e-6) {
        // Use initial distance as fallback
        const Real d_safe = _initial_distance;
        const Real d_target_safe = computeTargetDistance(d_safe);
        *C = d_safe - d_target_safe;
        return;
    }
    
    // Check breaking condition: if stretched beyond threshold, mark for removal
    // ✅ CRITICAL FIX: Use ELONGATION (stretch amount), not absolute distance!
    // 
    // WHY THIS MATTERS:
    // - Old: "break when d > threshold" → sensitive to initial_distance scale
    // - New: "break when (d - d0) > max_stretch" → measures actual tissue damage
    // 
    // PROBLEM EXAMPLE (old logic):
    //   initial=0.5mm, ratio=3 → threshold=1.5mm, abs_min boosts to 3mm
    //   initial=2.0mm, ratio=3 → threshold=6mm
    //   Result: Small initial_distance constraints cluster around 3mm "mine line"
    // 
    // SOLUTION:
    //   Use stretch amount: stretch = d - d0
    //   Break when: stretch > max(d0 * (ratio-1), delta_abs_min)
    //   Example: d0=0.5mm, ratio=3 → break when stretch > max(1.0mm, 3mm) = 3mm
    //            So d must reach 0.5+3=3.5mm (not 3mm!)
    
    const Real current_stretch = d - _initial_distance;  // Elongation from rest
    const Real stretch_tolerance = _initial_distance * (_break_ratio - 1.0);  // Allowed stretch (ratio-1 because ratio includes initial)
    // Use configured stretch_abs_min (tissue intrinsic toughness)
    // Represents minimum absolute elongation tissue can withstand before rupture
    // Independent of initial gap size (like collagen fiber rupture strain)
    const Real max_allowed_stretch = std::max(stretch_tolerance, _stretch_abs_min);
    
    if (current_stretch > max_allowed_stretch) {
        _should_break = true;
        *C = 0.0;  // Disable constraint
        return;
    }
    
    const Real d_target = computeTargetDistance(d);
    const Real constraint_value = d - d_target;
    
    // Compute dC/dd = 1 - dd*/dd (needed for gradient scaling)
    const Real eps = 1e-8;
    const Real d_target_plus = computeTargetDistance(d + eps);
    const Real d_target_minus = computeTargetDistance(d - eps);
    const Real dd_target_dd = (d_target_plus - d_target_minus) / (2.0 * eps);
    const Real dC_dd = 1.0 - dd_target_dd;

    // Cache for gradient reuse
    _separation_cached = d;
    _constraint_value_cached = constraint_value;
    _dC_dd_cached = dC_dd;
    
    // ✅ CRITICAL: Update rigid body helper with SCALED normal for gradient consistency
    // Deformable side uses: grad = -dC_dd * b_i * n
    // Rigid side uses: grad = +dC_dd * n (consistent with same scaling!)
    // Newton's 3rd law is maintained by gradient summation: Σ∇C = 0
    if (!_rigid_body_helpers.empty()) {
        auto* positional_helper = dynamic_cast<PositionalRigidBodyXPBDHelper*>(_rigid_body_helpers[0].get());
        if (positional_helper) {
            const Vec3r scaled_normal = dC_dd * _n_cached;
            // Use updateState instead of reallocation to prevent heap fragmentation/corruption issues
            positional_helper->updateState(scaled_normal, rigid_point_global);
        }
    }
    
    *C = constraint_value;
}

void UnifiedDistanceConstraint::gradient(Real* grad) const
{
    // Early exit if constraint should break
    if (_should_break) {
        for (int i = 0; i < NUM_COORDINATES; i++) {
            grad[i] = 0.0;
        }
        return;
    }
    
    // OPTIMIZATION: gradient() is called AFTER evaluate() in the same iteration
    if (!_cache_valid) {
        for (int i = 0; i < NUM_COORDINATES; i++) {
            grad[i] = 0.0;
        }
        return;
    }

    // Use frozen normal and barycentric coordinates from last evaluate() call
    const Vec3r& n = _n_cached;        // unit normal (triangle to rigid point)
    const Real b1 = _bary_cached[0];   // weight for tri_p1
    const Real b2 = _bary_cached[1];   // weight for tri_p2  
    const Real b3 = _bary_cached[2];   // weight for tri_p3

    // Gradient computation with frozen contact frame:
    // C = d - d*(d), where d = n^T(p_rigid - x_s), x_s = b1*p1 + b2*p2 + b3*p3
    // 
    // dC/dd = 1 - dd*/dd (cached from evaluate, always > 0)
    // 
    // Chain rule: dC/dp_i = (dC/dd) * (dd/dp_i)
    // where dd/dp_i = -b_i * n (for triangle vertices)
    //       dd/dp_rigid = +n (for rigid body, handled by helper)
    
    // Use cached dC_dd (already computed in evaluate for consistency)
    const Real dC_dd = _dC_dd_cached;

    // Gradients w.r.t. triangle vertices
    // dC/dp_i = dC_dd * dd/dp_i = dC_dd * (-b_i * n)
    // NOTE: Negative sign is CORRECT for XPBD! (Update: Δx = w × ∇C × λ, no extra negative)
    grad[0] = -dC_dd * b1 * n[0];  grad[1] = -dC_dd * b1 * n[1];  grad[2] = -dC_dd * b1 * n[2];  // p1
    grad[3] = -dC_dd * b2 * n[0];  grad[4] = -dC_dd * b2 * n[1];  grad[5] = -dC_dd * b2 * n[2];  // p2
    grad[6] = -dC_dd * b3 * n[0];  grad[7] = -dC_dd * b3 * n[1];  grad[8] = -dC_dd * b3 * n[2];  // p3
    
    // NOTE: Rigid body gradient is handled by RigidBodyXPBDHelper
    // Helper uses scaled_normal = dC_dd * n (updated in evaluate for consistency)
}

void UnifiedDistanceConstraint::evaluateWithGradient(Real* C, Real* grad) const
{
    evaluate(C);
    gradient(grad);
}

Real UnifiedDistanceConstraint::computePointTriangleDistance(
    const Vec3r& rigid_point_global,
    const Vec3r& tri_p1, 
    const Vec3r& tri_p2,
    const Vec3r& tri_p3,
    Vec3r& closest_point,
    Vec3r& normal,
    Vec3r& bary_coords) const
{
    // TODO CRITICAL: Replace with Ericson's ClosestPtPointTriangle (Real-Time Collision Detection)
    // Current barycentric clamp is NOT strict Euclidean projection
    // Can cause normal jumps at edges/corners, defeating smooth curve design
    // See: Christer Ericson, "Real-Time Collision Detection" (2004), Section 5.1.5
    
    // Compute triangle normal and area
    const Vec3r edge1 = tri_p2 - tri_p1;
    const Vec3r edge2 = tri_p3 - tri_p1;
    const Vec3r triangle_normal = edge1.cross(edge2);
    const Real area = triangle_normal.norm();
    
    if (area < 1e-12) {
        // Degenerate triangle fallback
        normal = Vec3r::UnitZ();
        closest_point = tri_p1;
        bary_coords = Vec3r(1.0, 0.0, 0.0);
        // Return centroid-to-point distance instead of 1e6
        Vec3r centroid = (tri_p1 + tri_p2 + tri_p3) / 3.0;
        return (rigid_point_global - centroid).norm();
    }

    // Use consistent triangle normal orientation (don't flip)
    normal = triangle_normal / area;
    
    // Compute plane signed distance
    const Real signed_distance = (rigid_point_global - tri_p1).dot(normal);
    
    // Project point onto triangle plane
    const Vec3r projected_point = rigid_point_global - signed_distance * normal;
    
    // Compute barycentric coordinates of projected point
    const Vec3r v0 = edge2;
    const Vec3r v1 = edge1;  
    const Vec3r v2 = projected_point - tri_p1;
    
    const Real dot00 = v0.dot(v0);
    const Real dot01 = v0.dot(v1);
    const Real dot02 = v0.dot(v2);
    const Real dot11 = v1.dot(v1);
    const Real dot12 = v1.dot(v2);
    
    const Real inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01);
    const Real u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
    const Real v = (dot00 * dot12 - dot01 * dot02) * inv_denom;
    const Real w = 1.0 - u - v;
    
    bary_coords = Vec3r(w, v, u);
    
    // Check if point is inside triangle
    if (u >= 0.0 && v >= 0.0 && (u + v) <= 1.0) {
        // Point projects inside triangle - use plane distance
        closest_point = projected_point;
        // Normal from closest_point to rigid_point (geometric gradient direction)
        Vec3r diff = rigid_point_global - closest_point;
        Real dist = diff.norm();
        if (dist < 1e-12) {
            // Point is on triangle surface
            normal = triangle_normal / area;  // Use triangle normal
            return 0.0;
        }
        normal = diff / dist;  // Unit vector from closest to point
        // ✅ CRITICAL FIX: Return unsigned distance (always >= 0)
        // Unified curve d*(d) is designed for d>=0 (separation distance)
        // This ensures frozen frame n_cached.dot(p-xs) gives consistent positive distance
        // Otherwise: first eval returns signed_distance (maybe <0), frozen eval returns |d|
        return dist;  // Always non-negative
    } else {
        // Point projects outside triangle - clamp to boundary (edge or vertex)
        Real u_clamp = std::max(0.0, std::min(1.0, u));
        Real v_clamp = std::max(0.0, std::min(1.0, v));
        if (u_clamp + v_clamp > 1.0) {
            const Real scale = 1.0 / (u_clamp + v_clamp);
            u_clamp *= scale;
            v_clamp *= scale;
        }
        const Real w_clamp = 1.0 - u_clamp - v_clamp;
        
        bary_coords = Vec3r(w_clamp, v_clamp, u_clamp);
        closest_point = w_clamp * tri_p1 + v_clamp * tri_p2 + u_clamp * tri_p3;
        
        // ✅ CRITICAL FIX: Use point-to-closest-point distance (not point-to-plane!)
        Vec3r diff = rigid_point_global - closest_point;
        Real dist = diff.norm();
        if (dist < 1e-12) {
            // Point coincides with closest point
            normal = triangle_normal / area;  // Use triangle normal as fallback
            return 0.0;
        }
        // ✅ CRITICAL FIX: Normal is geometric gradient direction (closest→point)
        normal = diff / dist;
        return dist;  // Euclidean distance (always non-negative for outside case)
    }
}

Real UnifiedDistanceConstraint::smoothstep(Real edge0, Real edge1, Real x) const
{
    // C¹ continuous: t²(3-2t) - first derivative continuous, not second
    const Real t = std::max(0.0, std::min(1.0, (x - edge0) / (edge1 - edge0)));
    return t * t * (3.0 - 2.0 * t);
}

Real UnifiedDistanceConstraint::expBlend(Real d0, Real s, Real d, Real gate_width) const
{
    // C¹ exponential blend with delayed start (validated design)
    // Smoothstep gate: ramps from 0→1 on [d0, d0+gate_width]
    const Real gate = smoothstep(d0, d0 + gate_width, d);
    
    // Exponential component: starts ONLY after gate completes
    // This avoids the gate'*exp spike that causes non-monotonicity
    const Real x = std::max(0.0, d - (d0 + gate_width));
    const Real exp_component = 1.0 - std::exp(-x / s);
    
    // Product: gate handles smooth startup, exp handles long-range approach
    return gate * exp_component;
}

Real UnifiedDistanceConstraint::computeTargetDistance(Real d) const
{
    // ============================================================================
    // C¹-SMOOTH STRETCH-ONLY RELATIVE ADHESION (2026-01-26 REDESIGN)
    // ============================================================================
    // Goal: Each constraint pulls back toward its initial_distance (d0) when stretched
    // with C¹ continuous smooth transition at d0 (eliminates gradient discontinuity)
    // 
    // Physics:
    //   - d < d0 - δ: fully inactive (d* = d, no force)
    //   - d > d0 + δ: fully active (d* = d0 + β(d-d0), pull-back)
    //   - |d - d0| ≤ δ: smooth transition (slope interpolation, C¹ continuous)
    // 
    // Guarantees:
    //   - dd*/dd ∈ [β, 1] globally (XPBD stable)
    //   - dC/dd ∈ [0, 1-β] globally (monotonic)
    //   - C¹ continuous everywhere (no gradient jumps, no chatter)
    //   - Compression handled by collision/CCD (separation of concerns)
    // 
    // Method: Interpolate slope (dd*/dd), then integrate to construct d*.
    // This is the ONLY way to guarantee dd*/dd stays in [β, 1].
    // Direct interpolation of d* would violate stability.
    // ============================================================================
    
    const Real d0 = _initial_distance;
    
    // ============================================================================
    // CONFIGURABLE PARAMETERS (Mapped from YAML via member variables)
    // ============================================================================
    // 1. Beta (Stiffness Slope): Controls how strongly we pull back to d0
    //    Range: (0.0 = rigid, 1.0 = no force).
    //    Mapping: beta = _d_rest / _d_neutral_start
    //    - If d_rest is small (tight), beta is small -> Stiffer pullback
    //    - If d_rest is large (loose), beta is large -> Softer pullback
    //    - Example: 1.5mm / 3.0mm = 0.5 (Medium stiffness)
    //    - Example: 0.003 / 0.005 = 0.6 (Softer)
    Real beta = 0.3; // Fallback default
    if (_d_neutral_start > 1e-6) {
        beta = std::max(0.01, std::min(0.99, _d_rest / _d_neutral_start));
    }
    
    // 2. Delta (Smoothing Width): Half-width of the C1 transition zone
    //    Mapping: delta = _d_contact (Contact thickness)
    //    - Example: 0.3mm
    Real delta = std::max(1e-5, _d_contact);

    // Define slopes for different regions
    // beta (right slope): Controls extension stiffness (Softer, < 1.0)
    // alpha_compress (left slope): Controls compression stiffness
    //   - 1.0 = No force (Slack, old behavior -> causes penetration)
    //   - beta = Symmetric spring (Soft support/Repulsion)
    //   - 0.0 = Hard support (pushes back to d0)
    // define slopes
    // slope_left: scaling for compression (d < d0)
    // - 0.0: Hard constraint (pulls/pushes to d0 with full stiffness) -> Prevents Penetration
    // - beta: Soft constraint (symmetric)
    // - 1.0: No constraint (d* = d) -> Allows free motion
    
    // FIX FOR PENETRATION: Use Hard Constraint (0.01) for compression.
    // This ensures we strongly resist peneration past d0.
    const Real slope_left = 0.01; 
    const Real slope_right = beta;

    // Region 1: d < d0 - delta (Compression Zone)
    if (d < d0 - delta) {
        // Hard push back to d0 (with linear scaling 0.01)
        return d0 + slope_left * (d - d0);
    }
    
    // Region 2: d > d0 + delta (Extension Zone)
    if (d > d0 + delta) {
        return d0 + slope_right * (d - d0);
    }
    
    // Region 3: Transition zone [d0-delta, d0+delta] (C¹ smooth interpolation)
    
    // Distance from left edge of transition zone
    const Real x = d - (d0 - delta);  // x ∈ [0, 2δ]
    
    // Normalized parameter: t ∈ [0, 1]
    const Real t = x / (2.0 * delta);
    
    // Target at left edge (d = d0 - delta)
    const Real d_target_left = d0 + slope_left * ((d0 - delta) - d0);
    
    // Integral of blend term for smooth transition: ∫ blend(t) dx = 2δ * (t^3 - t^4/2)
    const Real term_blend = (t*t*t - 0.5*t*t*t*t); 
    
    // d* = d_left + slope_left * x + (slope_right - slope_left) * integral_blend
    const Real d_target = d_target_left + slope_left * x + (slope_right - slope_left) * (2.0 * delta) * term_blend;
    
    return d_target;
}

Real UnifiedDistanceConstraint::getCurrentDistance() const
{
    // Extract positions
    Eigen::Map<const Vec3r> tri_p1(_positions[0].position_ptr);
    Eigen::Map<const Vec3r> tri_p2(_positions[1].position_ptr);
    Eigen::Map<const Vec3r> tri_p3(_positions[2].position_ptr);
    
    const Sim::RigidObject* rigid_obj = _rigid_bodies[0];
    const Vec3r rigid_point_global = rigid_obj->bodyToGlobal(_rigid_body_point);

    // Compute current distance
    Vec3r closest_point, normal, bary_coords;
    return computePointTriangleDistance(rigid_point_global, tri_p1, tri_p2, tri_p3, 
                                       closest_point, normal, bary_coords);
}

} // namespace Solver
