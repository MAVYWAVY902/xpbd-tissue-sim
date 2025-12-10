#ifndef __INTER_DEFORM_DEFORM_ADHESION_CONSTRAINT_HPP
#define __INTER_DEFORM_DEFORM_ADHESION_CONSTRAINT_HPP

#include "solver/constraint/Constraint.hpp"

namespace Solver
{

/** Adhesion constraint between a vertex on one deformable object and a face on another deformable object.
 * 
 * This is a GENERALIZED version of NerveTumorAdhesionConstraint that works between any two deformable meshes,
 * not just nerve-tumor pairs. It implements the same physics: distance-threshold adhesion with target gap d_0.
 * 
 * Constraint function: C(q) = max(0, n^T(q - p_surface) - d_0)
 * - Where p_surface is the closest point on the triangle computed via barycentric coordinates
 * - Attractive force when distance > d_0 (tension-only, one-sided adhesion)
 * - Breaking when strain ratio exceeds break_ratio threshold
 * 
 * IMPORTANT: This behaves as a "soft inequality" constraint:
 * - Returns isInequality() = false (registered as equality constraint to solver)
 * - But internally uses max(0, ...) to only activate when stretched beyond rest gap
 * - This makes it act like one-sided (tension-only) adhesion
 * - Collision constraints handle penetration prevention separately
 */
class InterDeformDeformAdhesionConstraint : public Constraint
{
    public:
    // One vertex from object A + three triangle vertices from object B = 4 positions
    static constexpr int NUM_POSITIONS = 4;
    static constexpr int NUM_COORDINATES = 12; // 4 positions × 3 coordinates each

    public:
    /** Constructor for inter-object deformable adhesion constraint.
     * @param vertex_v - vertex index (from object A)
     * @param vertex_p - vertex position pointer (from object A)
     * @param vertex_m - vertex mass (from object A)
     * @param tri_v1, tri_v2, tri_v3 - triangle vertex indices (from object B)
     * @param tri_p1, tri_p2, tri_p3 - triangle vertex position pointers (from object B)
     * @param tri_m1, tri_m2, tri_m3 - triangle vertex masses (from object B)
     * @param rest_gap - rest/initial separation distance d_0 (specific to this constraint)
     * @param break_ratio - strain ratio threshold for breaking (e.g., 1.5 = 50% extension)
     * @param alpha - compliance parameter
     */
    InterDeformDeformAdhesionConstraint(int vertex_v, Real* vertex_p, Real vertex_m,
                                        int tri_v1, Real* tri_p1, Real tri_m1,
                                        int tri_v2, Real* tri_p2, Real tri_m2, 
                                        int tri_v3, Real* tri_p3, Real tri_m3,
                                        Real rest_gap,
                                        Real break_ratio,
                                        Real alpha = 0.0);

    int numPositions() const override { return NUM_POSITIONS; }
    int numCoordinates() const override { return NUM_COORDINATES; }

    /** Evaluates the adhesion constraint: C(q) = max(0, n^T(q - p_surface) - d_0)
     * @param C (OUTPUT) - constraint value
     */
    void evaluate(Real* C) const override;

    /** Computes gradient of adhesion constraint distributed across vertex and triangle vertices
     * @param grad (OUTPUT) - gradient vector [grad_vertex, grad_tri1, grad_tri2, grad_tri3]
     */
    void gradient(Real* grad) const override;

    /** Combined evaluation and gradient computation for efficiency
     * @param C (OUTPUT) - constraint value  
     * @param grad (OUTPUT) - gradient vector
     */
    void evaluateWithGradient(Real* C, Real* grad) const override;

    /** Note: we return isInequality() = false but internally clamp C = max(0, ...).
     * This makes the adhesion act like a one-sided (tension-only) soft inequality.
     * The solver treats it as an equality constraint, but it only generates force when stretched.
     */
    inline bool isInequality() const override { return false; }

    /** Get rest separation distance (initial d_0 for this constraint) */
    Real getRestGap() const { return _rest_gap; }

    /** Get break ratio threshold */
    Real getBreakRatio() const { return _break_ratio; }
    
    /** Check if adhesion bond should break based on strain ratio.
     * Bond breaks when max_distance_this_step / rest_gap > break_ratio
     * @return true if bond should be broken and constraint removed
     */
    bool shouldBreak() const;
    
    /** Get current separation distance between vertex and triangle surface */
    Real getCurrentDistance() const;
    
    /** Reset max distance tracker at the beginning of each time step.
     * MUST be called before constraint projection to ensure _max_distance_this_step
     * represents "maximum stretch during THIS step" rather than "entire simulation history".
     * This prevents false positives where old stretch events trigger breaking forever.
     * Marked const because it modifies a mutable tracking field (cache-like behavior).
     * 
     * ✅ CRITICAL: Also invalidates cached contact frame to force recomputation.
     * This ensures gradient() doesn't use stale data from previous timestep.
     */
    void resetMaxDistanceThisStep() const { 
        _max_distance_this_step = 0.0; 
        _cache_valid = false;  // Invalidate cache at start of new timestep
    }

    protected:
    /** Compute signed distance from vertex to triangle and closest point info
     * @param vertex_pos - vertex position
     * @param tri_p1, tri_p2, tri_p3 - triangle vertex positions
     * @param closest_point (OUTPUT) - closest point on triangle surface
     * @param normal (OUTPUT) - unit normal vector (triangle to point)
     * @param bary_coords (OUTPUT) - barycentric coordinates [u, v, w] where u+v+w=1
     * @return signed distance (positive = separated, negative = penetrating)
     */
    Real computePointTriangleDistance(const Vec3r& vertex_pos,
                                     const Vec3r& tri_p1, 
                                     const Vec3r& tri_p2,
                                     const Vec3r& tri_p3,
                                     Vec3r& closest_point,
                                     Vec3r& normal,
                                     Vec3r& bary_coords) const;

    private:
    Real _rest_gap;    ///< Rest separation distance d_0 (initial distance for this constraint)
    Real _break_ratio; ///< Strain ratio threshold for breaking (e.g., 1.5 = 50% extension)
    
    // Cached values for frozen contact frame approach (mutable for const methods)
    mutable Vec3r _n_cached;      ///< unit normal (triangle to point)
    mutable Vec3r _bary_cached;   ///< barycentric coordinates [b1, b2, b3]  
    mutable Vec3r _xs_cached;     ///< closest point on triangle surface
    mutable Real  _constraint_value_cached{0.0}; ///< C = max(0, separation - rest_gap)
    mutable Real  _separation_cached{0.0}; ///< current separation distance
    mutable bool  _cache_valid{false}; ///< whether cached values are valid
    
    // Track maximum distance during projection (for breaking detection with fixed vertices)
    mutable Real _max_distance_this_step{0.0}; ///< Maximum distance reached during current step
};

} // namespace Solver

#endif // __INTER_DEFORM_DEFORM_ADHESION_CONSTRAINT_HPP
