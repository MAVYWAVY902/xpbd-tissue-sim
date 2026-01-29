#include "solver/constraint/InterDeformUnifiedDistanceConstraint.hpp"
#include "utils/MathUtils.hpp"
#include "utils/GeometryUtils.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

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
    _initial_distance(initial_distance),  // Use precomputed value
    _should_break(false)
{
    // 🔍 RUNTIME PARAMETER VERIFICATION (print first 3 constraints only)
    static int constraint_count = 0;
    if (constraint_count < 3) {
        std::cout << "\n🔍 [INTER-DEFORM UNIFIED #" << constraint_count << "] Runtime Parameters:" << std::endl;
        std::cout << "  d_contact = " << _d_contact << " m (" << _d_contact*1000 << " mm)" << std::endl;
        std::cout << "  d_rest = " << _d_rest << " m (" << _d_rest*1000 << " mm)" << std::endl;
        std::cout << "  d_neutral_start = " << _d_neutral_start << " m (" << _d_neutral_start*1000 << " mm)" << std::endl;
        std::cout << "  d_neutral_end = " << _d_neutral_end << " m (" << _d_neutral_end*1000 << " mm)" << std::endl;
        std::cout << "  d_bond = " << _d_bond << " m (" << _d_bond*1000 << " mm)" << std::endl;
        std::cout << "  alpha = " << alpha << std::endl;
        std::cout << "  break_ratio = " << _break_ratio << std::endl;
        std::cout << "  initial_distance (PRECOMPUTED) = " << initial_distance << " m (" << initial_distance*1000 << " mm)" << std::endl;
        std::cout << "  break_threshold = " << (initial_distance * _break_ratio)*1000 << " mm" << std::endl;
    }
    constraint_count++;
    
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
        static int cache_recompute_count = 0;
        if (cache_recompute_count < 10) {
            std::cout << "🔄 [INTER-DEFORM CACHE RECOMPUTE #" << cache_recompute_count << "] Computing fresh contact frame" << std::endl;
            cache_recompute_count++;
        }
        
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
        
        // 3. Compute normal (direction of force: from anchor TO vertex)
        Vec3r normal;
        if (point_to_tri_distance > 1e-12) {
            normal = diff / point_to_tri_distance;
        } else {
            const Vec3r edge1 = tri_p2 - tri_p1;
            const Vec3r edge2 = tri_p3 - tri_p1;
            Vec3r tri_normal = edge1.cross(edge2);
            Real area2 = tri_normal.norm();
            if (area2 > 1e-12) normal = tri_normal / area2;
            else normal = Vec3r::UnitZ(); // Degenerate
        }
        
        // Cache geometry for this timestep (frozen frame)
        _xs_cached = anchor_point;
        _n_cached = normal;
        
        // IMPORTANT: We do NOT update _bary_cached. We keep the initial bond point!
        
        _cache_valid = true;
        
        // 🔍 DEBUG: Print contact geometry for first few constraints
        static int contact_debug_count = 0;
        if (contact_debug_count < 5) {
            std::cout << "\n🔍🎯 [INTER-DEFORM CONTACT GEOMETRY #" << contact_debug_count << "] (Fixed Adhesion Mode)" << std::endl;
            std::cout << "  📌 Vertex position: " << vertex_pos.transpose() << std::endl;
            std::cout << "  📐 Triangle vertices:" << std::endl;
            std::cout << "      tri_p1: " << tri_p1.transpose() << std::endl;
            std::cout << "      tri_p2: " << tri_p2.transpose() << std::endl;
            std::cout << "      tri_p3: " << tri_p3.transpose() << std::endl;
            std::cout << "  🎯 Anchor point on triangle: " << anchor_point.transpose() << std::endl;
            std::cout << "  📏 Distance: " << point_to_tri_distance*1000 << " mm" << std::endl;
            std::cout << "  🧮 Fixed Barycentric coords: [" << _bary_cached[0] 
                      << ", " << _bary_cached[1] 
                      << ", " << _bary_cached[2] << "]" << std::endl;
            std::cout << "  ➡️  Normal: " << normal.transpose() << std::endl;
            contact_debug_count++;
        }
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
    
    // 🛡️ PROTECTION: Check for abnormal d values before break check
    if (!std::isfinite(d) || d > 1.0 || d < 1e-6) {
        static int abnormal_count = 0;
        if (abnormal_count < 5) {
            std::cout << "⚠️  [INTER-DEFORM ABNORMAL DISTANCE] d=" << d*1000 << "mm, using initial_distance=" 
                      << _initial_distance*1000 << "mm as fallback" << std::endl;
            abnormal_count++;
        }
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
        static int break_print_count = 0;
        if (break_print_count < 5) {
            std::cout << "🔴 [INTER-DEFORM CONSTRAINT BREAKING] stretch=" << current_stretch*1000 
                      << "mm > max_stretch=" << max_allowed_stretch*1000 
                      << "mm (d=" << d*1000 << "mm, d0=" << _initial_distance*1000 
                      << "mm, ratio=" << _break_ratio << ")" << std::endl;
            break_print_count++;
        }
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
    // This is IDENTICAL to UnifiedDistanceConstraint::computeTargetDistance
    // Stage 1: Contact → Rest (smoothstep)
    // Stage 2: Rest → Bond (exponential blend with gate)
    
    // Hard constraint zone (d < d_contact): Strong repulsion
    if (d <= _d_contact) {
        // FIX: Creating a hard wall at _d_contact
        // We want C = d - d_contact (approx), so d_target should be near d_contact.
        // Using slope_hard = 0.01 means d_target varies little from _d_contact.
        // Formula: d_target = d_contact + slope * (d - d_contact)
        // Resulting C = d - d_target = (1 - slope) * (d - d_contact)
        // If slope=0.01, C = 0.99 * (d - d_contact) -> STRONG repulsion
        
        const Real slope_wall = 0.01; 
        return _d_contact + slope_wall * (d - _d_contact);
    }
    
    // Stage 1: Contact → Rest (smoothstep blend)
    if (d <= _d_rest) {
        const Real t = smoothstep(_d_contact, _d_rest, d);
        return (1.0 - t) * _d_contact + t * _d_rest;
    }
    
    // Stage 2: Rest → Bond (exponential approach with gate)
    // Delayed exponential: stays at d_rest until d_neutral_start, then rises
    const Real gate = smoothstep(_d_neutral_start, _d_neutral_end, d);
    const Real s_max = (_d_bond - _d_rest) * EXP_SCALE_MARGIN / EXP_GATE_WIDTH;
    const Real exp_contrib = expBlend(_d_rest, s_max, d, EXP_GATE_WIDTH);
    const Real d_target_stage2 = _d_rest + gate * exp_contrib;
    
    return std::min(d_target_stage2, _d_bond);  // Cap at saturation
}

Real InterDeformUnifiedDistanceConstraint::smoothstep(Real edge0, Real edge1, Real x) const
{
    if (x <= edge0) return 0.0;
    if (x >= edge1) return 1.0;
    const Real t = (x - edge0) / (edge1 - edge0);
    return t * t * (3.0 - 2.0 * t);  // C¹ continuous: f(t) = 3t² - 2t³
}

Real InterDeformUnifiedDistanceConstraint::expBlend(Real d0, Real s, Real d, Real gate_width) const
{
    if (d <= d0) return 0.0;
    const Real exponent = -(d - d0) / gate_width;
    return s * gate_width * (1.0 - std::exp(exponent));
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
