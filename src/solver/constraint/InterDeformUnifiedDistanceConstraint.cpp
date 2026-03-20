#include "solver/constraint/InterDeformUnifiedDistanceConstraint.hpp"
#include "utils/MathUtils.hpp"
#include "utils/GeometryUtils.hpp"
#include <cmath>
#include <algorithm>

namespace Solver
{

InterDeformUnifiedDistanceConstraint::InterDeformUnifiedDistanceConstraint(
    int vertex_v, Real* vertex_p, Real vertex_m,
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
        PositionReference(vertex_v, vertex_p, vertex_m),  // vertex from object A
        PositionReference(tri_v1, tri_p1, tri_m1),         // triangle vertex 1 from object B
        PositionReference(tri_v2, tri_p2, tri_m2),         // triangle vertex 2 from object B
        PositionReference(tri_v3, tri_p3, tri_m3)          // triangle vertex 3 from object B
    }), alpha),
    _d_contact(d_contact),
    _d_rest(d_rest),
    _d_neutral_start(d_neutral_start),
    _d_neutral_end(d_neutral_end),
    _d_bond(d_bond),
    _stretch_abs_min(stretch_abs_min),
    _break_ratio(break_ratio),
    _original_break_ratio(break_ratio),
    _original_stretch_abs_min(stretch_abs_min),
    _initial_distance(initial_distance),  // Use precomputed value
    _default_alpha(alpha),              // Store original compliance
    _should_break(false)
{
    // INITIALIZATION: Compute barycentric coordinates once at creation to establish FIXED anchor point
    Eigen::Map<const Vec3r> vertex_pos_init(vertex_p);
    Eigen::Map<const Vec3r> tri_p1_init(tri_p1);
    Eigen::Map<const Vec3r> tri_p2_init(tri_p2);
    Eigen::Map<const Vec3r> tri_p3_init(tri_p3);
    
    Vec3r closest_point, normal;
    computePointTriangleDistance(vertex_pos_init, tri_p1_init, tri_p2_init, tri_p3_init,
                                closest_point, normal, _bary_cached);
    
    // IMPORTANT: Do NOT set _cache_valid=true here!
    // Let first evaluate() in timestep compute and cache the contact frame.
    _cache_valid = false;
}

void InterDeformUnifiedDistanceConstraint::evaluate(Real* C) const
{
    // Early exit if constraint should break (already marked for removal)
    if (_should_break) {
        *C = 0.0;
        return;
    }
    
    // Extract current positions (vertex from object A, triangle from object B)
    Eigen::Map<const Vec3r> vertex_pos(_positions[0].position_ptr);
    Eigen::Map<const Vec3r> tri_p1(_positions[1].position_ptr);
    Eigen::Map<const Vec3r> tri_p2(_positions[2].position_ptr);
    Eigen::Map<const Vec3r> tri_p3(_positions[3].position_ptr);
    
    // ✅ FROZEN FRAME: Only recompute geometry if cache is invalid (start of timestep)
    Real point_to_tri_distance;
    if (!_cache_valid) {
        // First evaluation in this timestep - compute and freeze contact geometry

        // FIXED ADHESION IMPLEMENTATION:
        // Use INITIAL barycentric coordinates (anchored material point).
        // This converts the constraint from "Point-to-Triangle" (Sliding) to "Point-to-Point" (Fixed).
        
        // 1. Reconstruct anchor point using current vertex positions + stored INITIAL barycentrics
        const Vec3r anchor_point = _bary_cached[0] * tri_p1 + 
                                   _bary_cached[1] * tri_p2 + 
                                   _bary_cached[2] * tri_p3;
                                   
        // 2. Compute vector from anchor to vertex
        const Vec3r diff = vertex_pos - anchor_point;
        point_to_tri_distance = diff.norm();
        
        // 3. Compute normal (direction of force)
        Vec3r normal;
        
        // COLLISION ROBUSTNESS: 
        // If we are very close (collision zone), prefer the Triangle Face Normal.
        // This prevents the "vector flip" issue if a vertex slightly penetrates/tunnels.
        // It ensures we always push OUT of the volume.
        bool use_face_normal = (point_to_tri_distance < _d_contact);
        
        if (use_face_normal) {
            const Vec3r edge1 = tri_p2 - tri_p1;
            const Vec3r edge2 = tri_p3 - tri_p1;
            Vec3r tri_normal = edge1.cross(edge2);
            Real area2 = tri_normal.norm();
            if (area2 > 1e-12) {
                 normal = tri_normal / area2;
                 
                 // Ensure normal points towards vertex (if vertex is on "front" side)
                 // But for collision, we usually implicitly trust Face Normal is "Out".
                 // Let's dot with diff to be consistent with current side constraint
                 if (diff.dot(normal) < 0) {
                     // Vertex is behind? If so, push it OUT (along normal)
                     // But diff points In.
                     // We want to increase signed distance.
                 }
            } else {
                 normal = Vec3r::UnitZ(); // Degenerate fallback
            }
        } else if (point_to_tri_distance > 1e-12) {
            // Adhesion zone: Point-to-Point direction is stable
            normal = diff / point_to_tri_distance;
        } else {
            normal = Vec3r::UnitZ();
        }
        
        // Cache geometry for this timestep (frozen frame)
        _xs_cached = anchor_point;
        _n_cached = normal;
        
        // IMPORTANT: We do NOT update _bary_cached. We keep the initial bond point!
        
        _cache_valid = true;
    } else {
        // Cache valid - use frozen contact frame (within same timestep)
        // Reconstruct surface point using frozen barycentric coordinates
        const Vec3r x_s = _bary_cached[0] * tri_p1 + _bary_cached[1] * tri_p2 + _bary_cached[2] * tri_p3;
        
        // Compute current separation vector and distance
        Vec3r diff = vertex_pos - x_s;
        point_to_tri_distance = diff.norm();
        
        // [FIX FOR SLIDING] Update Normal Direction!
        // Even though material points are fixed (barycentrics), the VECTOR separating them changes orientation.
        if (point_to_tri_distance > 1e-12) {
            _n_cached = diff / point_to_tri_distance;
        }
    }

    // ✅ UNIFIED CONSTRAINT: C(d) = d - d*(d)
    const Real d = point_to_tri_distance;
    
    // =========================================================================
    // DYNAMIC COMPLIANCE ADJUSTMENT
    // Key Concept: "Soft for Adhesion, Hard for Collision"
    // - If d <= d_contact: Act as a Collision Constraint (Hard, Alpha ≈ 0)
    // - If d > d_contact: Act as an Adhesion Constraint (Soft, Alpha = Config)
    // 
    // This allows InterDeformUnifiedDistanceConstraint to serve double duty
    // when inter-object-collisions are disabled for performance.
    // =========================================================================
    
    // We use const_cast because evaluate() is const but we need to update state
    // This is safe because _alpha is used by the Solver *after* evaluate()
    Real& mutable_alpha = const_cast<Real&>(_alpha);
    
    if (d <= _d_contact) {
        mutable_alpha = 1e-9; // Almost zero compliance = Hard constraint
    } else {
        mutable_alpha = _default_alpha; // Restore configured soft compliance
    }
    
    // Protection: Check for abnormal d values before break check
    if (!std::isfinite(d) || d > 1.0) {
        const Real d_safe = _initial_distance;
        const Real d_target_safe = computeTargetDistance(d_safe);
        *C = d_safe - d_target_safe;
        return;
    }
    
    // Check breaking condition: use ELONGATION (stretch amount), not absolute distance
    const Real current_stretch = d - _initial_distance;
    const Real stretch_tolerance = _initial_distance * (_break_ratio - 1.0);
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
    
    *C = constraint_value;
}

void InterDeformUnifiedDistanceConstraint::gradient(Real* grad) const
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

    // Use frozen normal and barycentric coordinates
    const Vec3r& n = _n_cached;
    const Real b1 = _bary_cached[0];  // weight for tri_p1
    const Real b2 = _bary_cached[1];  // weight for tri_p2
    const Real b3 = _bary_cached[2];  // weight for tri_p3

    // Use cached dC_dd
    const Real dC_dd = _dC_dd_cached;

    // Gradients:
    // For vertex: dC/dp_vertex = dC_dd * n (moves away from triangle → increases d)
    // For triangle: dC/dp_tri = -dC_dd * b_i * n (moves away from vertex → increases d)
    
    // Vertex gradient (position 0)
    grad[0] = dC_dd * n[0];  grad[1] = dC_dd * n[1];  grad[2] = dC_dd * n[2];
    
    // Triangle vertex gradients (positions 1, 2, 3)
    grad[3] = -dC_dd * b1 * n[0];  grad[4] = -dC_dd * b1 * n[1];  grad[5] = -dC_dd * b1 * n[2];  // tri_p1
    grad[6] = -dC_dd * b2 * n[0];  grad[7] = -dC_dd * b2 * n[1];  grad[8] = -dC_dd * b2 * n[2];  // tri_p2
    grad[9] = -dC_dd * b3 * n[0];  grad[10] = -dC_dd * b3 * n[1]; grad[11] = -dC_dd * b3 * n[2]; // tri_p3
}

void InterDeformUnifiedDistanceConstraint::evaluateWithGradient(Real* C, Real* grad) const
{
    evaluate(C);
    gradient(grad);
}

Real InterDeformUnifiedDistanceConstraint::computeTargetDistance(Real d) const
{
    // ============================================================================
    // C¹-SMOOTH SLOPE-INTERPOLATION DESIGN (ported from UnifiedDistanceConstraint)
    // ============================================================================
    // Each constraint pulls back toward its initial_distance (d0) when stretched
    // with C¹ continuous smooth transition at d0 (eliminates gradient discontinuity)
    //
    // Physics:
    //   - d < d0 - δ: compression zone (d* ≈ d0, hard push-back, slope=0.01)
    //   - d > d0 + δ: extension zone (d* = d0 + β(d-d0), pull-back)
    //   - |d - d0| ≤ δ: smooth transition (slope interpolation, C¹ continuous)
    //
    // Guarantees:
    //   - dd*/dd ∈ [0.01, β] globally (XPBD stable)
    //   - dC/dd ∈ [1-β, 0.99] globally (monotonic, always positive)
    //   - C¹ continuous everywhere (no gradient jumps, no chatter)
    // ============================================================================

    const Real d0 = _initial_distance;

    // Beta (Stiffness Slope): Controls how strongly we pull back to d0
    // Mapping: beta = _d_rest / _d_neutral_start
    Real beta = 0.3; // Fallback default
    if (_d_neutral_start > 1e-6) {
        beta = std::max(0.01, std::min(0.99, _d_rest / _d_neutral_start));
    }

    // Delta (Smoothing Width): Half-width of the C1 transition zone
    // Mapping: delta = _d_contact
    Real delta = std::max(1e-5, _d_contact);

    // Define slopes
    const Real slope_left = 0.01;  // Hard push-back for compression
    const Real slope_right = beta; // Soft pull-back for extension

    // Region 1: d < d0 - delta (Compression Zone)
    if (d < d0 - delta) {
        return d0 + slope_left * (d - d0);
    }

    // Region 2: d > d0 + delta (Extension Zone)
    if (d > d0 + delta) {
        return d0 + slope_right * (d - d0);
    }

    // Region 3: Transition zone [d0-delta, d0+delta] (C¹ smooth interpolation)
    // Method: Interpolate slope (dd*/dd), then integrate to construct d*.

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

Real InterDeformUnifiedDistanceConstraint::computePointTriangleDistance(
    const Vec3r& vertex_pos,
    const Vec3r& tri_p1, 
    const Vec3r& tri_p2,
    const Vec3r& tri_p3,
    Vec3r& closest_point,
    Vec3r& normal,
    Vec3r& bary_coords) const
{
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
        Vec3r centroid = (tri_p1 + tri_p2 + tri_p3) / 3.0;
        return (vertex_pos - centroid).norm();
    }

    // Use consistent triangle normal orientation
    normal = triangle_normal / area;
    
    // Compute plane signed distance
    Real signed_distance = (vertex_pos - tri_p1).dot(normal);
    
    // Flip normal to point toward vertex if needed
    if (signed_distance < 0) {
        normal = -normal;
        signed_distance = -signed_distance;
    }
    
    // Project point onto triangle plane
    const Vec3r projected_point = vertex_pos - signed_distance * normal;
    
    // Compute barycentric coordinates
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
        closest_point = projected_point;
        return signed_distance;
    } else {
        // Outside - clamp to boundary
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
        return (vertex_pos - closest_point).norm();
    }
}

Real InterDeformUnifiedDistanceConstraint::getCurrentDistance() const
{
    Eigen::Map<const Vec3r> vertex_pos(_positions[0].position_ptr);
    Eigen::Map<const Vec3r> tri_p1(_positions[1].position_ptr);
    Eigen::Map<const Vec3r> tri_p2(_positions[2].position_ptr);
    Eigen::Map<const Vec3r> tri_p3(_positions[3].position_ptr);

    Vec3r closest_point, normal, bary_coords;
    return computePointTriangleDistance(vertex_pos, tri_p1, tri_p2, tri_p3, 
                                       closest_point, normal, bary_coords);
}

Real InterDeformUnifiedDistanceConstraint::getBreakThreshold() const
{
    const Real stretch_tolerance = _initial_distance * (_break_ratio - 1.0);
    const Real max_allowed_stretch = std::max(stretch_tolerance, _stretch_abs_min);
    return _initial_distance + max_allowed_stretch;
}

} // namespace Solver
