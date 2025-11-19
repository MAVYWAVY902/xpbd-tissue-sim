#include "solver/constraint/NerveTumorAdhesionConstraint.hpp"
#include "utils/MathUtils.hpp"
#include <iostream>

namespace Solver
{

NerveTumorAdhesionConstraint::NerveTumorAdhesionConstraint(int nerve_v, Real* nerve_p, Real nerve_m,
                                                         int tri_v1, Real* tri_p1, Real tri_m1,
                                                         int tri_v2, Real* tri_p2, Real tri_m2, 
                                                         int tri_v3, Real* tri_p3, Real tri_m3,
                                                         Real target_gap,
                                                         Real alpha)
    : Constraint(std::vector<PositionReference>({
        PositionReference(nerve_v, nerve_p, nerve_m),  // nerve vertex
        PositionReference(tri_v1, tri_p1, tri_m1),     // triangle vertex 1
        PositionReference(tri_v2, tri_p2, tri_m2),     // triangle vertex 2  
        PositionReference(tri_v3, tri_p3, tri_m3)      // triangle vertex 3
    }), alpha), _target_gap(target_gap)
{
}

void NerveTumorAdhesionConstraint::evaluate(Real* C) const
{
    // Extract positions
    Eigen::Map<const Vec3r> nerve_pos(_positions[0].position_ptr);
    Eigen::Map<const Vec3r> tri_p1(_positions[1].position_ptr);
    Eigen::Map<const Vec3r> tri_p2(_positions[2].position_ptr);
    Eigen::Map<const Vec3r> tri_p3(_positions[3].position_ptr);

    // Compute closest point, normal, and barycentric coordinates
    Vec3r closest_point, normal, bary_coords;
    const Real distance = computePointTriangleDistance(nerve_pos, tri_p1, tri_p2, tri_p3, 
                                                      closest_point, normal, bary_coords);
    
    // Check for valid computation
    if (!std::isfinite(distance) || distance >= 1e6) {
        *C = 0.0; // constraint inactive for degenerate cases
        _cache_valid = false;
        return;
    }

    // Cache the contact frame for use in gradient computation
    _xs_cached = closest_point;
    _n_cached = normal;
    _bary_cached = bary_coords;
    _cache_valid = true;

    // ✅ FIXED: Single-sided adhesion constraint (only attractive, no repulsion)
    // C = max(0, n^T(q - x_s) - d_0)
    // This creates a one-way spring that only pulls when separated beyond target gap
    Real separation_distance = _n_cached.dot(nerve_pos - _xs_cached);
    Real constraint_violation = separation_distance - _target_gap;
    
    // Only activate constraint when separated beyond target gap (adhesive pull)
    *C = std::max(0.0, constraint_violation);
    
    // DEBUG: Print constraint evaluation details (limit output frequency)
    static int debug_count = 0;
    debug_count++;
    if (debug_count % 9000 == 0) {  // Print every 9000 evaluations (10x less frequent)
        std::cout << "[adhesion DEBUG] Constraint eval #" << debug_count 
                  << ": nerve=(" << nerve_pos.transpose() << ")"
                  << " separation=" << separation_distance 
                  << " constraint_violation=" << constraint_violation
                  << " C=" << *C 
                  << " target_gap=" << _target_gap << "\n";
    }
}

void NerveTumorAdhesionConstraint::gradient(Real* grad) const
{
    // Ensure evaluate() has been called to populate cache
    if (!_cache_valid) {
        // Call evaluate to compute and cache contact frame
        Real dummy_C;
        evaluate(&dummy_C);
        
        // If still invalid after evaluate, return zero gradient
        if (!_cache_valid) {
            for (int i = 0; i < NUM_COORDINATES; i++) {
                grad[i] = 0.0;
            }
            return;
        }
    }

    // ✅ FIXED: Check if constraint is active (C > 0)
    // For single-sided adhesion, gradient is zero when constraint is inactive
    Eigen::Map<const Vec3r> nerve_pos(_positions[0].position_ptr);
    Real separation_distance = _n_cached.dot(nerve_pos - _xs_cached);
    Real constraint_violation = separation_distance - _target_gap;
    
    if (constraint_violation <= 0.0) {
        // Constraint is inactive (not separated beyond target gap)
        for (int i = 0; i < NUM_COORDINATES; i++) {
            grad[i] = 0.0;
        }
        return;
    }

    // Use frozen normal and barycentric coordinates from last evaluate() call
    const Vec3r& n = _n_cached;        // unit normal (triangle to point)
    const Real b1 = _bary_cached[0];   // weight for tri_p1
    const Real b2 = _bary_cached[1];   // weight for tri_p2  
    const Real b3 = _bary_cached[2];   // weight for tri_p3

    // Gradient computation with frozen contact frame:
    // C = max(0, n^T(q - x_s) - d_0), where x_s = b1*p1 + b2*p2 + b3*p3
    // When C > 0: ∂C/∂q = n, ∂C/∂pi = -bi * n
    // When C = 0: all gradients are 0 (handled above)

    // Gradient w.r.t. nerve vertex (point q)
    grad[0] = n[0];   grad[1] = n[1];   grad[2] = n[2];

    // Gradients w.r.t. triangle vertices (via barycentric distribution)
    grad[3] = -b1 * n[0];  grad[4] = -b1 * n[1];  grad[5] = -b1 * n[2];  // p1
    grad[6] = -b2 * n[0];  grad[7] = -b2 * n[1];  grad[8] = -b2 * n[2];  // p2
    grad[9] = -b3 * n[0];  grad[10] = -b3 * n[1]; grad[11] = -b3 * n[2]; // p3
}

void NerveTumorAdhesionConstraint::evaluateWithGradient(Real* C, Real* grad) const
{
    evaluate(C);
    gradient(grad);
}

Real NerveTumorAdhesionConstraint::computePointTriangleDistance(const Vec3r& nerve_pos,
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
        // Degenerate triangle - return large distance
        normal = Vec3r::UnitZ(); // arbitrary normal
        closest_point = tri_p1; // arbitrary point on triangle
        bary_coords = Vec3r(1.0, 0.0, 0.0); // all weight on first vertex
        return 1e6; // large distance to indicate invalid
    }

    normal = triangle_normal / area;
    
    // Compute plane signed distance  
    Real signed_distance = (nerve_pos - tri_p1).dot(normal);
    
    // ✅ ORIENTATION-INVARIANT: Flip normal to point toward nerve if needed
    if (signed_distance < 0) {
        normal = -normal;
        signed_distance = -signed_distance;
    }
    
    // Project point onto triangle plane (with corrected normal)
    const Vec3r projected_point = nerve_pos - signed_distance * normal;
    
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
    const Real u = (dot11 * dot02 - dot01 * dot12) * inv_denom; // weight for tri_p3
    const Real v = (dot00 * dot12 - dot01 * dot02) * inv_denom; // weight for tri_p2
    const Real w = 1.0 - u - v; // weight for tri_p1
    
    bary_coords = Vec3r(w, v, u);
    
    // Check if point is inside triangle
    if (u >= 0.0 && v >= 0.0 && (u + v) <= 1.0) {
        // Point projects inside triangle
        closest_point = projected_point;
        return signed_distance; // Now always non-negative due to normal flip
    } else {
        // Point projects outside triangle - clamp to triangle boundary
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
        
        // For outside points: use consistent plane distance (orientation-invariant)
        return std::abs((nerve_pos - tri_p1).dot(normal));
    }
}

bool NerveTumorAdhesionConstraint::shouldBreak(Real break_distance) const
{
    // Extract positions
    Eigen::Map<const Vec3r> nerve_pos(_positions[0].position_ptr);
    Eigen::Map<const Vec3r> tri_p1(_positions[1].position_ptr);
    Eigen::Map<const Vec3r> tri_p2(_positions[2].position_ptr);
    Eigen::Map<const Vec3r> tri_p3(_positions[3].position_ptr);

    // Compute current distance
    Vec3r closest_point, normal, bary_coords;
    const Real distance = computePointTriangleDistance(nerve_pos, tri_p1, tri_p2, tri_p3, 
                                                      closest_point, normal, bary_coords);

    return (distance > break_distance);
                                                      
}

Real NerveTumorAdhesionConstraint::getCurrentDistance() const
{
    // Extract positions
    Eigen::Map<const Vec3r> nerve_pos(_positions[0].position_ptr);
    Eigen::Map<const Vec3r> tri_p1(_positions[1].position_ptr);
    Eigen::Map<const Vec3r> tri_p2(_positions[2].position_ptr);
    Eigen::Map<const Vec3r> tri_p3(_positions[3].position_ptr);

    // Compute current distance
    Vec3r closest_point, normal, bary_coords;
    return computePointTriangleDistance(nerve_pos, tri_p1, tri_p2, tri_p3, 
                                       closest_point, normal, bary_coords);
}

} // namespace Solver