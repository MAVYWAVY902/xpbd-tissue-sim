#include "solver/constraint/InterDeformDeformAdhesionConstraint.hpp"
#include "utils/MathUtils.hpp"
#include <iostream>

namespace Solver
{

InterDeformDeformAdhesionConstraint::InterDeformDeformAdhesionConstraint(int vertex_v, Real* vertex_p, Real vertex_m,
                                                         int tri_v1, Real* tri_p1, Real tri_m1,
                                                         int tri_v2, Real* tri_p2, Real tri_m2, 
                                                         int tri_v3, Real* tri_p3, Real tri_m3,
                                                         Real rest_gap,
                                                         Real break_ratio,
                                                         Real alpha)
    : Constraint(std::vector<PositionReference>({
        PositionReference(vertex_v, vertex_p, vertex_m),  // vertex from object A
        PositionReference(tri_v1, tri_p1, tri_m1),         // triangle vertex 1 from object B
        PositionReference(tri_v2, tri_p2, tri_m2),         // triangle vertex 2 from object B
        PositionReference(tri_v3, tri_p3, tri_m3)          // triangle vertex 3 from object B
    }), alpha), _rest_gap(rest_gap), _break_ratio(break_ratio)
{
    // INITIALIZATION: Compute barycentric coordinates once at creation
    // These will be reused in evaluate() to reconstruct the closest point on the deforming triangle
    Eigen::Map<const Vec3r> vertex_pos_init(vertex_p);
    Eigen::Map<const Vec3r> tri_p1_init(tri_p1);
    Eigen::Map<const Vec3r> tri_p2_init(tri_p2);
    Eigen::Map<const Vec3r> tri_p3_init(tri_p3);
    
    Vec3r closest_point, normal;
    computePointTriangleDistance(vertex_pos_init, tri_p1_init, tri_p2_init, tri_p3_init,
                                closest_point, normal, _bary_cached);
    
    _cache_valid = true;  // Barycentric coords are now valid
    
    // std::cout << "[inter-deform adhesion INIT] Created constraint: rest_gap=" << _rest_gap 
    //           << ", break_ratio=" << _break_ratio << ", alpha=" << alpha << "\n";
}

void InterDeformDeformAdhesionConstraint::evaluate(Real* C) const
{
    static int eval_count = 0;
    const bool debug = (eval_count % 1000 == 0);  // Print every 1000th evaluation
    
    // Extract current positions (these change during simulation as triangle deforms)
    Eigen::Map<const Vec3r> vertex_pos(_positions[0].position_ptr);
    Eigen::Map<const Vec3r> tri_p1(_positions[1].position_ptr);
    Eigen::Map<const Vec3r> tri_p2(_positions[2].position_ptr);
    Eigen::Map<const Vec3r> tri_p3(_positions[3].position_ptr);

    // PERFORMANCE OPTIMIZATION: Use cached barycentric coordinates!
    // Instead of recomputing expensive point-to-triangle distance every iteration,
    // we reconstruct the closest point using the CACHED barycentric coordinates.
    // This assumes the closest point stays roughly at the same barycentric location
    // on the triangle as it deforms (valid for small deformations).
    
    // Reconstruct closest point on deformed triangle using cached barycentric coords
    const Vec3r xs_current = _bary_cached[0] * tri_p1 + 
                             _bary_cached[1] * tri_p2 + 
                             _bary_cached[2] * tri_p3;
    
    // Recompute normal (must be updated as triangle deforms)
    const Vec3r edge1 = tri_p2 - tri_p1;
    const Vec3r edge2 = tri_p3 - tri_p1;
    Vec3r normal = edge1.cross(edge2);
    const Real normal_length = normal.norm();
    
    if (normal_length < 1e-12) {
        *C = 0.0;  // Degenerate triangle
        return;
    }
    normal /= normal_length;  // Normalize
    
    // Compute signed distance along normal direction
    const Vec3r separation_vec = vertex_pos - xs_current;
    const Real separation_distance = normal.dot(separation_vec);
    
    // Track maximum distance during this step's solver iterations (for breaking detection)
    _max_distance_this_step = std::max(_max_distance_this_step, std::abs(separation_distance));
    
    // Cache the updated contact frame for gradient computation
    _xs_cached = xs_current;
    _n_cached = normal;
    // _bary_cached stays unchanged - we reuse the initial barycentric coords
    
    // if (debug) {
    //     std::cout << "[inter-deform EVAL #" << eval_count << "] separation=" << separation_distance 
    //               << ", rest_gap=" << _rest_gap << ", C=" << *C << "\n";
    // }
    eval_count++;
    _cache_valid = true;  // ✅ CRITICAL FIX: Mark cache as valid after updating!

    // ✅ FIXED: Single-sided adhesion constraint (only attractive, no repulsion)
    // C = max(0, separation - d_0)
    // This creates a one-way spring that only pulls when separated beyond rest gap
    const Real constraint_violation = separation_distance - _rest_gap;
    
    // Cache separation and constraint value for gradient() to reuse
    _separation_cached = separation_distance;
    _constraint_value_cached = std::max(0.0, constraint_violation);
    
    // Only activate constraint when separated beyond target gap (adhesive pull)
    *C = _constraint_value_cached;
    
    // DEBUG: Print constraint evaluation details for first few evaluations
    // NOTE: Disable this in production for performance!
    #ifdef ADHESION_DEBUG_VERBOSE
    static int debug_count = 0;
    debug_count++;
    if (debug_count <= 10) {  // Print first 10 evaluations
        std::cout << "[inter-deform adhesion EVAL#" << debug_count << "] "
                  << "separation=" << separation_distance 
                  << " rest_gap=" << _rest_gap
                  << " violation=" << constraint_violation
                  << " C=" << *C << "\n";
    }
    #endif
    
    // ALWAYS-ON DEBUG: Print when constraint is active (C > 0)
    static int active_count = 0;
    if (*C > 0) {
        active_count++;
        if (active_count <= 20 || active_count % 2000 == 0) {
            std::cout << "[INTER-DEFORM ADHESION ACTIVE #" << active_count << "] "
                      << "vertex_v=" << _positions[0].index 
                      << " tri=[" << _positions[1].index << "," << _positions[2].index << "," << _positions[3].index << "]"
                      << " | sep=" << separation_distance << "m"
                      << " | rest=" << _rest_gap << "m"
                      << " | C=" << *C << "m"
                      << " | alpha=" << this->alpha() << "\n";
        }
    }
}

void InterDeformDeformAdhesionConstraint::gradient(Real* grad) const
{
    // OPTIMIZATION: gradient() is called AFTER evaluate() in the same iteration
    // So _cache_valid should always be true. No need to re-evaluate.
    
    // ✅ PERFORMANCE: Avoid redundant checks by assuming evaluate() was just called
    // The XPBD solver always calls evaluate() before gradient() in the same iteration.
    // If for some reason cache is invalid, we return zero gradient (safe fallback).
    if (!_cache_valid) {
        for (int i = 0; i < NUM_COORDINATES; i++) {
            grad[i] = 0.0;
        }
        return;
    }

    // ✅ CRITICAL OPTIMIZATION: Reuse cached constraint value instead of recomputing!
    // Previously we were doing: separation = n.dot(q - xs), C = max(0, separation - d0)
    // Now we just read the cached value from evaluate()
    if (_constraint_value_cached <= 0.0) {
        // Constraint is inactive (not separated beyond rest gap)
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

    // Gradient w.r.t. vertex (point q)
    grad[0] = n[0];   grad[1] = n[1];   grad[2] = n[2];

    // Gradients w.r.t. triangle vertices (via barycentric distribution)
    grad[3] = -b1 * n[0];  grad[4] = -b1 * n[1];  grad[5] = -b1 * n[2];  // p1
    grad[6] = -b2 * n[0];  grad[7] = -b2 * n[1];  grad[8] = -b2 * n[2];  // p2
    grad[9] = -b3 * n[0];  grad[10] = -b3 * n[1]; grad[11] = -b3 * n[2]; // p3
}

void InterDeformDeformAdhesionConstraint::evaluateWithGradient(Real* C, Real* grad) const
{
    evaluate(C);
    gradient(grad);
}

Real InterDeformDeformAdhesionConstraint::computePointTriangleDistance(const Vec3r& vertex_pos,
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
    Real signed_distance = (vertex_pos - tri_p1).dot(normal);
    
    // ✅ ORIENTATION-INVARIANT: Flip normal to point toward vertex if needed
    if (signed_distance < 0) {
        normal = -normal;
        signed_distance = -signed_distance;
    }
    
    // Project point onto triangle plane (with corrected normal)
    const Vec3r projected_point = vertex_pos - signed_distance * normal;
    
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
        return std::abs((vertex_pos - tri_p1).dot(normal));
    }
}

bool InterDeformDeformAdhesionConstraint::shouldBreak() const
{
    // DESIGN RATIONALE: Strain-based breaking with initial geometry as rest state
    // 
    // This constraint uses the same breaking logic as NerveTumorAdhesionConstraint:
    // - Each constraint remembers its initial distance d_0 as rest_gap
    // - Break when STRAIN RATIO exceeds threshold: (distance / d_0) > break_ratio
    // - Example: break_ratio = 1.5 means bond breaks at 50% extension (d = 1.5 * d_0)
    // - Physically correct: bonds break from relative stretch, not absolute distance
    // 
    // WHY USE _max_distance_this_step?
    // The solver performs multiple Gauss-Seidel iterations to satisfy constraints.
    // During these iterations, positions can be temporarily stretched far beyond their
    // final converged state. We track the maximum distance seen across all solver
    // iterations within this time step to detect bonds that should break.
    // 
    // CRITICAL: resetMaxDistanceThisStep() MUST be called at the start of each time step
    // (done in XPBDMeshObject::update()), otherwise this becomes "max over entire simulation"
    // and will trigger false positives forever.
    
    // PHYSICS: Break when STRAIN RATIO exceeds threshold
    // strain_ratio = max_distance_this_step / rest_gap
    // Bond breaks when stretched beyond critical extension (e.g., 1.5 = 50% strain)
    // 
    // SPECIAL CASE: When rest_gap=0 (zero rest length), use absolute distance threshold
    // instead of strain ratio to avoid division by zero.
    Real strain_ratio;
    bool should_break;
    
    if (_rest_gap > 1e-12) {
        // Normal case: strain-based breaking
        strain_ratio = _max_distance_this_step / _rest_gap;
        should_break = (strain_ratio > _break_ratio);
    } else {
        // rest_gap ≈ 0: Use absolute distance threshold
        // break_ratio is reinterpreted as absolute distance in meters
        strain_ratio = std::numeric_limits<Real>::infinity();  // for debug output
        should_break = (_max_distance_this_step > _break_ratio * 0.01);  // break_ratio * 1cm
    }
    
    // DEBUG: Print max distance info when checking breaking
    // Get vertex indices for identification
    int vertex_v = _positions[0].index;
    int tri_v1 = _positions[1].index;
    int tri_v2 = _positions[2].index;
    int tri_v3 = _positions[3].index;
    
    // Also compute current distance and ratio (not max) for comparison
    Real current_distance = getCurrentDistance();
    Real current_ratio = current_distance / _rest_gap;
    
    // If breaking, always print (important events)
    // if (should_break) {
    //     std::cout << "[INTER-DEFORM ADHESION BREAKING!] vertex_v=" << vertex_v 
    //               << " tri=[" << tri_v1 << "," << tri_v2 << "," << tri_v3 << "]"
    //               << "\n  | current_dist=" << current_distance << "m, current_ratio=" << current_ratio
    //               << "\n  | max_dist=" << _max_distance_this_step << "m, max_ratio=" << strain_ratio
    //               << "\n  | rest_gap=" << _rest_gap << "m, break_ratio=" << _break_ratio << " (EXCEEDED)\n";
    // }
    
    // NOTE: Do NOT reset _max_distance_this_step here! 
    // Multiple constraints are checked during the same breaking phase, and resetting
    // would cause all constraints after the first to see max_distance=0 and never break.
    // The reset happens correctly in resetMaxDistanceThisStep() at the start of update().
    
    return should_break;
}

Real InterDeformDeformAdhesionConstraint::getCurrentDistance() const
{
    // Extract positions
    Eigen::Map<const Vec3r> vertex_pos(_positions[0].position_ptr);
    Eigen::Map<const Vec3r> tri_p1(_positions[1].position_ptr);
    Eigen::Map<const Vec3r> tri_p2(_positions[2].position_ptr);
    Eigen::Map<const Vec3r> tri_p3(_positions[3].position_ptr);

    // Compute current distance
    Vec3r closest_point, normal, bary_coords;
    return computePointTriangleDistance(vertex_pos, tri_p1, tri_p2, tri_p3, 
                                       closest_point, normal, bary_coords);
}

} // namespace Solver
