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
    Real d_bond)
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
    _break_ratio(break_ratio),
    _initial_distance(initial_distance),  // Use precomputed value
    _should_break(false)
{
    // 🔍 RUNTIME PARAMETER VERIFICATION (print first 3 constraints only)
    static int constraint_count = 0;
    if (constraint_count < 3) {
        std::cout << "\n🔍 [CONSTRAINT #" << constraint_count << "] Runtime Parameters:" << std::endl;
        std::cout << "  d_contact = " << _d_contact << " m (" << _d_contact*1000 << " mm)" << std::endl;
        std::cout << "  d_rest = " << _d_rest << " m (" << _d_rest*1000 << " mm)" << std::endl;
        std::cout << "  d_neutral_start = " << _d_neutral_start << " m (" << _d_neutral_start*1000 << " mm)" << std::endl;
        std::cout << "  d_neutral_end = " << _d_neutral_end << " m (" << _d_neutral_end*1000 << " mm)" << std::endl;
        std::cout << "  d_bond = " << _d_bond << " m (" << _d_bond*1000 << " mm)" << std::endl;
        std::cout << "  EXP_GATE_WIDTH = " << EXP_GATE_WIDTH << " m (" << EXP_GATE_WIDTH*1000 << " mm)" << std::endl;
        std::cout << "  EXP_SCALE_MARGIN = " << EXP_SCALE_MARGIN << std::endl;
        std::cout << "  alpha = " << alpha << std::endl;
        std::cout << "  break_ratio = " << _break_ratio << std::endl;
        std::cout << "  initial_distance = " << initial_distance << " m (" << initial_distance*1000 << " mm)" << std::endl;
        
        // 🔍 VERTEX POSITION DEBUG - AT CREATION TIME
        std::cout << "\n  📍 VERTEX POSITIONS AT CREATION:" << std::endl;
        std::cout << "    tri_p1 ptr = " << (void*)tri_p1 << std::endl;
        std::cout << "    tri_p2 ptr = " << (void*)tri_p2 << std::endl;
        std::cout << "    tri_p3 ptr = " << (void*)tri_p3 << std::endl;
        Eigen::Map<const Vec3r> p1_init(tri_p1);
        Eigen::Map<const Vec3r> p2_init(tri_p2);
        Eigen::Map<const Vec3r> p3_init(tri_p3);
        std::cout << "    tri_p1 = " << p1_init.transpose() << std::endl;
        std::cout << "    tri_p2 = " << p2_init.transpose() << std::endl;
        std::cout << "    tri_p3 = " << p3_init.transpose() << std::endl;
        std::cout << "    rigid_pt (body) = " << rigid_body_point.transpose() << std::endl;
        const Vec3r rigid_global_init = rigid_obj->bodyToGlobal(rigid_body_point);
        std::cout << "    rigid_pt (global) = " << rigid_global_init.transpose() << std::endl;
        Vec3r edge1_init = p2_init - p1_init;
        Vec3r edge2_init = p3_init - p1_init;
        Real area_init = edge1_init.cross(edge2_init).norm() / 2.0;
        std::cout << "    Triangle area = " << area_init << " m²" << std::endl;
    }
    constraint_count++;
    
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
    
    // NOTE: _initial_distance is set from constructor parameter (precomputed by caller)
    // Do NOT try to compute it here - vertex pointers may not be initialized yet
    
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
    
    // 🔍 VERTEX POSITION DEBUG - AT EVALUATION TIME (first 3 constraints only)
    static int eval_debug_count = 0;
    if (eval_debug_count < 3) {
        std::cout << "\n🔍🔍 [EVAL GEOMETRY DEBUG #" << eval_debug_count << "]" << std::endl;
        std::cout << "  📍 VERTEX POSITIONS AT EVALUATION:" << std::endl;
        std::cout << "    _positions[0].position_ptr = " << (void*)_positions[0].position_ptr << std::endl;
        std::cout << "    _positions[1].position_ptr = " << (void*)_positions[1].position_ptr << std::endl;
        std::cout << "    _positions[2].position_ptr = " << (void*)_positions[2].position_ptr << std::endl;
        std::cout << "    tri_p1 = " << tri_p1.transpose() << std::endl;
        std::cout << "    tri_p2 = " << tri_p2.transpose() << std::endl;
        std::cout << "    tri_p3 = " << tri_p3.transpose() << std::endl;
        std::cout << "    rigid_pt (body) = " << _rigid_body_point.transpose() << std::endl;
        std::cout << "    rigid_pt (global) = " << rigid_point_global.transpose() << std::endl;
        Vec3r edge1 = tri_p2 - tri_p1;
        Vec3r edge2 = tri_p3 - tri_p1;
        Real area = edge1.cross(edge2).norm() / 2.0;
        std::cout << "    Triangle area = " << area << " m²" << std::endl;
        std::cout << "    _cache_valid = " << _cache_valid << std::endl;
        eval_debug_count++;
    }

    // ✅ FROZEN FRAME: Only recompute geometry if cache is invalid (start of timestep)
    Real point_to_tri_distance;
    if (!_cache_valid) {
        // First evaluation in this timestep - compute and freeze contact geometry
        Vec3r closest_point, normal;
        Vec3r bary_coords_current;
        
        point_to_tri_distance = computePointTriangleDistance(
            rigid_point_global, tri_p1, tri_p2, tri_p3,
            closest_point, normal, bary_coords_current);
        
        // Cache geometry for this timestep (frozen frame)
        _xs_cached = closest_point;
        _n_cached = normal;
        _bary_cached = bary_coords_current;
        _cache_valid = true;
    } else {
        // Cache valid - use frozen contact frame (within same timestep)
        // Reconstruct surface point using frozen barycentric coordinates
        // NOTE: This keeps contact point fixed in material coordinates during solver iterations
        // d = n_cached · (p_rigid - x_s) where x_s = b1*p1 + b2*p2 + b3*p3
        // This is NOT strict Euclidean distance, but \"separation along frozen normal\"
        // Valid linearization for small motion, prevents feature jumping (critical for stability)
        const Vec3r x_s = _bary_cached[0] * tri_p1 + _bary_cached[1] * tri_p2 + _bary_cached[2] * tri_p3;
        point_to_tri_distance = _n_cached.dot(rigid_point_global - x_s);
    }

    // ✅ UNIFIED CONSTRAINT: C(d) = d - d*(d)
    // - d: current distance
    // - d*(d): smooth target distance curve
    // - Equilibrium: d = d*(d) (C = 0)
    // - Too close: d < d*(d) (C < 0, repulsion)
    // - Too far: d > d*(d) (C > 0, attraction)
    
    const Real d = point_to_tri_distance;
    
    // 🛡️ PROTECTION: Check for abnormal d values before break check
    if (!std::isfinite(d) || d > 1.0 || d < 1e-6) {  // d < 1μm is likely geometry error
        static int abnormal_count = 0;
        if (abnormal_count < 5) {
            std::cout << "⚠️  [ABNORMAL DISTANCE] d=" << d*1000 << "mm, using initial_distance=" 
                      << _initial_distance*1000 << "mm as fallback" << std::endl;
            abnormal_count++;
        }
        // Use initial distance as fallback instead of returning 0
        const Real d_safe = _initial_distance;
        const Real d_target_safe = computeTargetDistance(d_safe);
        *C = d_safe - d_target_safe;
        return;
    }
    
    // Check breaking condition: if stretched beyond threshold, mark for removal
    const Real break_threshold = _initial_distance * _break_ratio;
    if (d > break_threshold) {
        _should_break = true;
        // DEBUG: Print when breaking condition is triggered
        static int break_print_count = 0;
        if (break_print_count < 5) {
            std::cout << "🔴 [CONSTRAINT BREAKING] d=" << d*1000 << "mm > threshold=" 
                      << break_threshold*1000 << "mm (initial=" << _initial_distance*1000 
                      << "mm × ratio=" << _break_ratio << ")" << std::endl;
            break_print_count++;
        }
        *C = 0.0;  // Disable constraint
        return;
    }
    
    const Real d_target = computeTargetDistance(d);
    const Real constraint_value = d - d_target;
    
    // 🔍 DEBUG: Print first few constraints at first evaluation
    static int eval_count = 0;
    static bool first_eval = true;
    if (first_eval && eval_count < 5) {
        std::cout << "🔍 [EVAL #" << eval_count << "] d=" << d*1000 << "mm, d*=" << d_target*1000 
                  << "mm, C=" << constraint_value*1000 << "mm";
        if (constraint_value > 0) std::cout << " (ATTRACTION ✅)";
        else if (constraint_value < 0) std::cout << " (REPULSION ⚠️)";
        else std::cout << " (EQUILIBRIUM)";
        std::cout << std::endl;
        eval_count++;
        if (eval_count >= 5) first_eval = false;
    }
    
    // Compute dC/dd = 1 - dd*/dd (needed for gradient scaling)
    const Real eps = 1e-8;
    const Real d_target_plus = computeTargetDistance(d + eps);
    const Real d_target_minus = computeTargetDistance(d - eps);
    const Real dd_target_dd = (d_target_plus - d_target_minus) / (2.0 * eps);
    const Real dC_dd = 1.0 - dd_target_dd;  // Should be > 0 (validated)
    
    // Cache for gradient reuse
    _separation_cached = d;
    _constraint_value_cached = constraint_value;
    _dC_dd_cached = dC_dd;
    
    // ✅ CRITICAL: Update rigid body helper with SCALED normal for gradient consistency
    // Deformable side uses: grad = -dC_dd * b_i * n
    // Rigid side must use: grad = +dC_dd * n (same scaling!)
    if (!_rigid_body_helpers.empty()) {
        auto* positional_helper = dynamic_cast<PositionalRigidBodyXPBDHelper*>(_rigid_body_helpers[0].get());
        if (positional_helper) {
            const Vec3r scaled_normal = dC_dd * _n_cached;
            *positional_helper = PositionalRigidBodyXPBDHelper(rigid_obj, scaled_normal, rigid_point_global);
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
        // Degenerate triangle
        normal = Vec3r::UnitZ();
        closest_point = tri_p1;
        bary_coords = Vec3r(1.0, 0.0, 0.0);
        return 1e6;
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
    // Stage 1: contact → rest (smoothstep blend, active in [_d_contact, _d_neutral_start])
    const Real blend1 = smoothstep(_d_contact, _d_neutral_start, d);
    const Real stage1_target = _d_contact * (1.0 - blend1) + _d_rest * blend1;
    
    // Stage 2: rest → bond (C¹ exponential blend, active AFTER _d_neutral_end)
    // Use 1.2x margin to ensure max(dd*/dd) < 1 with numerical safety
    const Real s = EXP_SCALE_MARGIN * (_d_bond - _d_rest);
    const Real blend2 = expBlend(_d_neutral_end, s, d, EXP_GATE_WIDTH);
    
    // Combine: use stage1 result + add stage2 contribution
    // When d < _d_neutral_end: blend2 ≈ 0, d_target ≈ stage1_target
    // When d >> _d_neutral_end: blend2 → 1, d_target → _d_bond
    const Real d_target = stage1_target * (1.0 - blend2) + _d_bond * blend2;
    
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
