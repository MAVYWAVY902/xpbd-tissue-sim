#ifndef __RIGID_DEFORM_ADHESION_CONSTRAINT_HPP
#define __RIGID_DEFORM_ADHESION_CONSTRAINT_HPP

#include "solver/constraint/Constraint.hpp"
#include "solver/constraint/RigidBodyConstraint.hpp"
#include "simobject/RigidObject.hpp"
#include "geometry/SDF.hpp"

namespace Solver
{

/** Adhesion constraint between a rigid body and a deformable triangle.
 * 
 * This implements distance-threshold adhesion between a fixed point on a rigid body
 * and a triangle on a deformable mesh, with the same physics as InterDeformDeformAdhesionConstraint
 * but adapted for rigid-deformable interactions.
 * 
 * Constraint function: C(q) = max(0, n^T(p_surface - p_rigid) - d_0)
 * - Where p_surface is the closest point on the deformable triangle
 * - p_rigid is a point on the rigid body (in body coordinates, transformed to global)
 * - Attractive force when distance > d_0 (tension-only, one-sided adhesion)
 * - Breaking when strain ratio exceeds break_ratio threshold
 * 
 * IMPORTANT: This behaves as a "soft inequality" constraint:
 * - Returns isInequality() = false (MUST be false to generate pull force!)
 * - But internally uses max(0, ...) to only activate when stretched beyond rest gap
 * - This makes it act like one-sided (tension-only) adhesion
 * - Same design as InterDeformDeformAdhesionConstraint
 * 
 * RIGID BODY HANDLING:
 * - Inherits from both Constraint and RigidBodyConstraint
 * - Uses RigidBodyXPBDHelper for proper rigid body updates (position + orientation)
 * - Rigid body point is stored in body coordinates and transformed during evaluation
 */
class RigidDeformAdhesionConstraint : public Constraint, public RigidBodyConstraint
{
    public:
    // Three triangle vertices (deformable) = 3 positions
    // Rigid body handled separately through RigidBodyConstraint
    static constexpr int NUM_POSITIONS = 3;
    static constexpr int NUM_COORDINATES = 9; // 3 positions × 3 coordinates each
    static constexpr int NUM_RIGID_BODIES = 1;

    public:
    /** Constructor for rigid-deformable adhesion constraint.
     * @param sdf - SDF of the rigid object (for distance queries if needed)
     * @param rigid_obj - pointer to the rigid object
     * @param rigid_body_point - attachment point on rigid body (in body coordinates)
     * @param tri_v1, tri_v2, tri_v3 - triangle vertex indices (from deformable object)
     * @param tri_p1, tri_p2, tri_p3 - triangle vertex position pointers (from deformable object)
     * @param tri_m1, tri_m2, tri_m3 - triangle vertex masses (from deformable object)
     * @param rest_gap - rest/initial separation distance d_0 (specific to this constraint)
     * @param break_ratio - strain ratio threshold for breaking (e.g., 1.5 = 50% extension)
     * @param alpha - compliance parameter
     */
    RigidDeformAdhesionConstraint(const Geometry::SDF* sdf, 
                                   Sim::RigidObject* rigid_obj,
                                   const Vec3r& rigid_body_point,
                                   int tri_v1, Real* tri_p1, Real tri_m1,
                                   int tri_v2, Real* tri_p2, Real tri_m2, 
                                   int tri_v3, Real* tri_p3, Real tri_m3,
                                   Real rest_gap,
                                   Real break_ratio,
                                   Real alpha = 0.0);

    int numPositions() const override { return NUM_POSITIONS; }
    int numCoordinates() const override { return NUM_COORDINATES; }
    int numRigidBodies() const { return NUM_RIGID_BODIES; }

    /** Evaluates the adhesion constraint: C(q) = max(0, n^T(p_surface - p_rigid) - d_0)
     * @param C (OUTPUT) - constraint value
     */
    void evaluate(Real* C) const override;

    /** Computes gradient of adhesion constraint distributed across triangle vertices and rigid body
     * @param grad (OUTPUT) - gradient vector [grad_tri1, grad_tri2, grad_tri3]
     * Note: Rigid body gradient is handled separately by RigidBodyXPBDHelper
     */
    void gradient(Real* grad) const override;

    /** Combined evaluation and gradient computation for efficiency
     * @param C (OUTPUT) - constraint value  
     * @param grad (OUTPUT) - gradient vector
     */
    void evaluateWithGradient(Real* C, Real* grad) const override;

    /** CRITICAL: Returns false - registered as EQUALITY constraint to solver.
     * We internally clamp C = max(0, ...) to create one-sided adhesion,
     * but solver must treat it as equality to generate restoring force.
     * If isInequality() = true, solver only acts when C < 0, so no pull force! */
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
    
    /** Get current separation distance between rigid body point and triangle surface */
    Real getCurrentDistance() const;
    
    /** Reset max distance tracker at the beginning of each time step.
     * MUST be called before constraint projection to ensure _max_distance_this_step
     * represents "maximum stretch during THIS step" rather than "entire simulation history".
     * Marked const because it modifies a mutable tracking field (cache-like behavior).
     * 
     * ✅ CRITICAL: Also invalidates cached contact frame to force recomputation.
     */
    void resetMaxDistanceThisStep() const { 
        _max_distance_this_step = 0.0; 
        _cache_valid = false;  // Invalidate cache at start of new timestep
    }
    
    /** Get point on rigid body in body coordinates */
    const Vec3r& rigidBodyPoint() const { return _rigid_body_point; }

    protected:
    /** Compute signed distance from rigid body point to triangle and closest point info
     * @param rigid_point_global - rigid body point in global coordinates
     * @param tri_p1, tri_p2, tri_p3 - triangle vertex positions
     * @param closest_point (OUTPUT) - closest point on triangle surface
     * @param normal (OUTPUT) - unit normal vector (triangle to point)
     * @param bary_coords (OUTPUT) - barycentric coordinates [u, v, w] where u+v+w=1
     * @return signed distance (positive = separated, negative = penetrating)
     */
    Real computePointTriangleDistance(const Vec3r& rigid_point_global,
                                     const Vec3r& tri_p1, 
                                     const Vec3r& tri_p2,
                                     const Vec3r& tri_p3,
                                     Vec3r& closest_point,
                                     Vec3r& normal,
                                     Vec3r& bary_coords) const;

    private:
    const Geometry::SDF* _sdf;  ///< SDF of rigid object (optional, for advanced queries)
    Vec3r _rigid_body_point;     ///< Point on rigid body in body coordinates
    Real _rest_gap;              ///< Rest separation distance d_0 (slack length for adhesion)
    Real _initial_distance;      ///< Initial distance at constraint creation
    Real _break_ratio;           ///< Strain ratio threshold for breaking
    
    // Cached values for frozen contact frame approach (mutable for const methods)
    mutable Vec3r _n_cached;      ///< unit normal (triangle to rigid point)
    mutable Vec3r _bary_cached;   ///< barycentric coordinates [b1, b2, b3]  
    mutable Vec3r _xs_cached;     ///< closest point on triangle surface
    mutable Real  _constraint_value_cached{0.0}; ///< C = max(0, separation - rest_gap)
    mutable Real  _separation_cached{0.0}; ///< current separation distance
    mutable bool  _cache_valid{false}; ///< whether cached values are valid
    
    // Track maximum distance during projection (for breaking detection)
    mutable Real _max_distance_this_step{0.0}; ///< Maximum distance reached during current step
};

} // namespace Solver

#endif // __RIGID_DEFORM_ADHESION_CONSTRAINT_HPP
