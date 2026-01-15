#include "solver/constraint/RigidDeformAdhesionConstraint.hpp"
#include "utils/MathUtils.hpp"
#include "utils/GeometryUtils.hpp"
#include <iostream>

namespace Solver
{

RigidDeformAdhesionConstraint::RigidDeformAdhesionConstraint(
    const Geometry::SDF* sdf, 
    Sim::RigidObject* rigid_obj,
    const Vec3r& rigid_body_point,
    int tri_v1, Real* tri_p1, Real tri_m1,
    int tri_v2, Real* tri_p2, Real tri_m2, 
    int tri_v3, Real* tri_p3, Real tri_m3,
    Real rest_gap,
    Real break_ratio,
    Real alpha)
    : Constraint(std::vector<PositionReference>({
        PositionReference(tri_v1, tri_p1, tri_m1),  // triangle vertex 1
        PositionReference(tri_v2, tri_p2, tri_m2),  // triangle vertex 2
        PositionReference(tri_v3, tri_p3, tri_m3)   // triangle vertex 3
    }), alpha),
    RigidBodyConstraint(std::vector<Sim::RigidObject*>({rigid_obj})),
    _sdf(sdf),
    _rigid_body_point(rigid_body_point),
    _rest_gap(rest_gap),
    _break_ratio(break_ratio)
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
    
    _cache_valid = true;  // Barycentric coords are now valid
    
    // Create RigidBodyXPBDHelper for positional constraint
    // The correction direction is along the normal from triangle to rigid body point
    // PositionalRigidBodyXPBDHelper needs: (rigid_obj, direction, point_on_body_global)
    _rigid_body_helpers.push_back(
        std::make_unique<PositionalRigidBodyXPBDHelper>(rigid_obj, normal, rigid_point_global)
    );
    
    // std::cout << "[rigid-deform adhesion INIT] Created constraint: rest_gap=" << _rest_gap 
    //           << ", break_ratio=" << _break_ratio << ", alpha=" << alpha << "\n";
}

void RigidDeformAdhesionConstraint::evaluate(Real* C) const
{
    static int eval_count = 0;
    eval_count++;
    
    // Extract current triangle positions
    Eigen::Map<const Vec3r> tri_p1(_positions[0].position_ptr);
    Eigen::Map<const Vec3r> tri_p2(_positions[1].position_ptr);
    Eigen::Map<const Vec3r> tri_p3(_positions[2].position_ptr);
    
    // Get rigid body point in global coordinates (transforms with rigid body motion)
    const Sim::RigidObject* rigid_obj = _rigid_bodies[0];
    const Vec3r rigid_point_global = rigid_obj->bodyToGlobal(_rigid_body_point);

    // PERFORMANCE OPTIMIZATION: Use cached barycentric coordinates!
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
    // Note: direction is from triangle surface to rigid body point
    const Vec3r separation_vec = rigid_point_global - xs_current;
    const Real separation_distance = normal.dot(separation_vec);
    
    // Track maximum distance during this step's solver iterations (for breaking detection)
    _max_distance_this_step = std::max(_max_distance_this_step, std::abs(separation_distance));
    
    // Cache the updated contact frame for gradient computation
    _xs_cached = xs_current;
    _n_cached = normal;
    // _bary_cached stays unchanged - we reuse the initial barycentric coords
    _cache_valid = true;

    // ✅ FIXED: Single-sided adhesion constraint (only attractive, no repulsion)
    // C = max(0, separation - d_0)
    // This creates a one-way spring that only pulls when separated beyond rest gap
    const Real constraint_violation = separation_distance - _rest_gap;
    
    // Cache separation and constraint value for gradient() to reuse
    _separation_cached = separation_distance;
    _constraint_value_cached = std::max(0.0, constraint_violation);
    
    // Only activate constraint when separated beyond target gap (adhesive pull)
    *C = _constraint_value_cached;
    
    // DEBUG: Print every 1000 evaluations when constraint is active
    // if (*C > 0 && eval_count % 1000 == 0) {
    //     std::cout << "[RIGID-DEFORM ADHESION ACTIVE eval #" << eval_count << "] "
    //               << "rigid_body=" << rigid_obj->name()
    //               << " tri=[" << _positions[0].index << "," << _positions[1].index << "," << _positions[2].index << "]"
    //               << " | sep=" << separation_distance << "m"
    //               << " | rest=" << _rest_gap << "m"
    //               << " | C=" << *C << "m"
    //               << " | alpha=" << this->alpha() << "\n";
    // }
}

void RigidDeformAdhesionConstraint::gradient(Real* grad) const
{
    // OPTIMIZATION: gradient() is called AFTER evaluate() in the same iteration
    // So _cache_valid should always be true. No need to re-evaluate.
    
    if (!_cache_valid) {
        for (int i = 0; i < NUM_COORDINATES; i++) {
            grad[i] = 0.0;
        }
        return;
    }

    // ✅ CRITICAL OPTIMIZATION: Reuse cached constraint value instead of recomputing!
    if (_constraint_value_cached <= 0.0) {
        // Constraint is inactive (not separated beyond rest gap)
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
    // C = max(0, n^T(p_rigid - x_s) - d_0), where x_s = b1*p1 + b2*p2 + b3*p3
    // When C > 0: ∂C/∂pi = -bi * n  (for triangle vertices)
    //             ∂C/∂p_rigid = n   (for rigid body - handled by RigidBodyXPBDHelper)
    // When C = 0: all gradients are 0 (handled above)

    // Gradients w.r.t. triangle vertices (via barycentric distribution)
    // Note: Negative because increasing triangle vertex position decreases separation
    grad[0] = -b1 * n[0];  grad[1] = -b1 * n[1];  grad[2] = -b1 * n[2];  // p1
    grad[3] = -b2 * n[0];  grad[4] = -b2 * n[1];  grad[5] = -b2 * n[2];  // p2
    grad[6] = -b3 * n[0];  grad[7] = -b3 * n[1];  grad[8] = -b3 * n[2];  // p3
    
    // NOTE: Rigid body gradient is handled separately by RigidBodyXPBDHelper
    // The helper will use the normal direction _n_cached to update rigid body
    // position and orientation correctly
}

void RigidDeformAdhesionConstraint::evaluateWithGradient(Real* C, Real* grad) const
{
    evaluate(C);
    gradient(grad);
}

Real RigidDeformAdhesionConstraint::computePointTriangleDistance(
    const Vec3r& rigid_point_global,
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
    Real signed_distance = (rigid_point_global - tri_p1).dot(normal);
    
    // ✅ ORIENTATION-INVARIANT: Flip normal to point toward rigid point if needed
    if (signed_distance < 0) {
        normal = -normal;
        signed_distance = -signed_distance;
    }
    
    // Project point onto triangle plane (with corrected normal)
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
        return std::abs((rigid_point_global - tri_p1).dot(normal));
    }
}

bool RigidDeformAdhesionConstraint::shouldBreak() const
{
    // DESIGN RATIONALE: Strain-based breaking with initial geometry as rest state
    // Same logic as InterDeformDeformAdhesionConstraint
    
    Real strain_ratio;
    bool should_break;
    
    if (_rest_gap > 1e-12) {
        // Normal case: strain-based breaking
        strain_ratio = _max_distance_this_step / _rest_gap;
        should_break = (strain_ratio > _break_ratio);
    } else {
        // rest_gap ≈ 0: Use absolute distance threshold
        strain_ratio = std::numeric_limits<Real>::infinity();
        should_break = (_max_distance_this_step > _break_ratio * 0.01);  // break_ratio * 1cm
    }
    
    // // DEBUG: Print breaking info
    // if (should_break) {
    //     const Sim::RigidObject* rigid_obj = _rigid_bodies[0];
    //     Real current_distance = getCurrentDistance();
    //     Real current_ratio = current_distance / _rest_gap;
        
    //     std::cout << "[RIGID-DEFORM ADHESION BREAKING!] rigid_body=" << rigid_obj->name()
    //               << " tri=[" << _positions[0].index << "," << _positions[1].index << "," << _positions[2].index << "]"
    //               << "\n  | current_dist=" << current_distance << "m, current_ratio=" << current_ratio
    //               << "\n  | max_dist=" << _max_distance_this_step << "m, max_ratio=" << strain_ratio
    //               << "\n  | rest_gap=" << _rest_gap << "m, break_ratio=" << _break_ratio << " (EXCEEDED)\n";
    // }
    
    return should_break;
}

Real RigidDeformAdhesionConstraint::getCurrentDistance() const
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
