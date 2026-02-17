#ifndef __UNIFIED_DISTANCE_CONSTRAINT_HPP
#define __UNIFIED_DISTANCE_CONSTRAINT_HPP

#include "solver/constraint/Constraint.hpp"
#include "solver/constraint/RigidBodyConstraint.hpp"
#include "simobject/RigidObject.hpp"
#include "geometry/SDF.hpp"

namespace Solver
{

/** Unified distance constraint for rigid-deformable soft tissue adhesion.
 * 
 * DESIGN SCOPE: This is a **separation distance servo** for adhesion/attraction,
 * NOT a hard contact/penetration barrier. True collision prevention is handled
 * by RigidDeformableCollisionConstraint with inequality constraints + CCD.
 * 
 * This replaces separate collision + adhesion constraints with a single smooth constraint
 * that continuously transitions between repulsion -> neutral -> attraction.
 * 
 * Key innovation: Uses target distance formulation C(d) = d - d*(d) where:
 * - d: current separation distance (always >= 0, unsigned)
 * - d*(d): smooth target distance curve that transitions:
 *   * contact (0.1mm) -> rest (1.0mm) -> bond (5.0mm)
 * - C(d) = 0 implies d = d*(d) (equilibrium)
 * - C(d) < 0 implies too close (repulsion, but no hard barrier)
 * - C(d) > 0 implies too far (attraction)
 * 
 * Frozen Contact Frame: During XPBD solver iterations within a timestep,
 * the contact geometry (closest point, normal) is frozen to avoid feature jumping.
 * Thus d becomes the "separation along frozen normal" rather than strict Euclidean distance:
 *   d = n_cached . (p_rigid - x_s)
 * This is a linearized approximation valid for small motion, trading geometric
 * precision for numerical stability (standard practice in contact mechanics).
 * 
 * Mathematical guarantees:
 * - C¹ continuous (smooth forces, continuous gradients)
 * - Monotonic: dC/dd > 0 everywhere (unique equilibrium, XPBD stable)
 * - No dead zones (gradient always defined)
 * 
 * Design validated via scripts/test_unified_constraint_math.py:
 * - min(dC/dd) = 0.156 > 0 [PASS]
 * - max(dd_star/dd) = 0.843 < 1 [PASS]
 * - Unique zero crossing at d ~= 0.1mm [PASS]
 * 
 * Parameters (all in meters):
 * - D_CONTACT = 0.0001 (0.1mm): Contact thickness, equilibrium position
 * - D_NEUTRAL_START = 0.0017 (1.7mm): Start of neutral zone
 * - D_NEUTRAL_END = 0.002 (2.0mm): End of neutral zone
 * - D_BOND = 0.005 (5.0mm): Far-field adhesion target
 * - D_REST = 0.001 (1.0mm): Mid-range target distance
 */
class UnifiedDistanceConstraint : public Constraint, public RigidBodyConstraint
{
public:
    // Three triangle vertices (deformable) = 3 positions
    // Rigid body handled separately through RigidBodyConstraint
    static constexpr int NUM_POSITIONS = 3;
    static constexpr int NUM_COORDINATES = 9; // 3 positions × 3 coordinates each
    static constexpr int NUM_RIGID_BODIES = 1;

    // Distance parameters - NOW CONFIGURABLE VIA YAML!
    // Default values (UPDATED 2026-01-26: Extended saturation point for better large-gap behavior)
    static constexpr Real DEFAULT_D_CONTACT = 0.0003;       // 0.3mm - contact equilibrium
    static constexpr Real DEFAULT_D_REST = 0.0015;          // 1.5mm - mid-range target
    static constexpr Real DEFAULT_D_NEUTRAL_START = 0.003;  // 3mm - transition start
    static constexpr Real DEFAULT_D_NEUTRAL_END = 0.005;    // 5mm - transition end
    static constexpr Real DEFAULT_D_BOND = 0.015;           // 15mm - saturation (INCREASED from 5mm)
    static constexpr Real EXP_GATE_WIDTH = 0.008;   // 8mm - smooth startup gate (2x wider for C¹ continuity)
    static constexpr Real EXP_SCALE_MARGIN = 1.2;   // 20% margin for stability
    static constexpr Real DEFAULT_BREAK_RATIO = 3.0; // Default: break at 200% strain (3x initial)

public:
    /** Constructor for unified distance constraint.
     * @param sdf - SDF of the rigid object (for distance queries if needed)
     * @param rigid_obj - pointer to the rigid object
     * @param rigid_body_point - attachment point on rigid body (in body coordinates)
     * @param tri_v1, tri_v2, tri_v3 - triangle vertex indices (from deformable object)
     * @param tri_p1, tri_p2, tri_p3 - triangle vertex position pointers (from deformable object)
     * @param tri_m1, tri_m2, tri_m3 - triangle vertex masses (from deformable object)
     * @param alpha - compliance parameter
     * @param break_ratio - break when distance > initial_distance * break_ratio (default 3.0)
     * @param initial_distance - PRECOMPUTED initial distance (must be computed before solver modifies vertices)
     * @param d_contact - equilibrium distance (YAML configurable, default 20mm)
     * @param d_rest - mid-range target distance (YAML configurable, default 30mm)
     * @param d_neutral_start - transition zone start (YAML configurable, default 36mm)
     * @param d_neutral_end - transition zone end (YAML configurable, default 40mm)
     * @param d_bond - saturation distance (YAML configurable, default 60mm)
     */
    UnifiedDistanceConstraint(const Geometry::SDF* sdf, 
                             Sim::RigidObject* rigid_obj,
                             const Vec3r& rigid_body_point,
                             int tri_v1, Real* tri_p1, Real tri_m1,
                             int tri_v2, Real* tri_p2, Real tri_m2, 
                             int tri_v3, Real* tri_p3, Real tri_m3,
                             Real alpha = 0.0,
                             Real break_ratio = DEFAULT_BREAK_RATIO,
                             Real initial_distance = 0.0,
                             Real d_contact = DEFAULT_D_CONTACT,
                             Real d_rest = DEFAULT_D_REST,
                             Real d_neutral_start = DEFAULT_D_NEUTRAL_START,
                             Real d_neutral_end = DEFAULT_D_NEUTRAL_END,
                                         Real d_bond = DEFAULT_D_BOND,
                                         Real stretch_abs_min = 0.005);

    // Required virtual functions from Constraint base class
    int numPositions() const override { return NUM_POSITIONS; }
    int numCoordinates() const override { return NUM_COORDINATES; }
    int numRigidBodies() const { return NUM_RIGID_BODIES; }

    /** Evaluates unified constraint: C(d) = d - d*(d)
     * @param C (OUTPUT) - constraint value
     */
    void evaluate(Real* C) const override;

    /** Computes gradient of constraint distributed across triangle vertices and rigid body
     * @param grad (OUTPUT) - gradient vector [grad_tri1, grad_tri2, grad_tri3]
     * Note: Rigid body gradient is handled separately by RigidBodyXPBDHelper
     */
    void gradient(Real* grad) const override;

    /** Combined evaluation and gradient computation for efficiency
     * @param C (OUTPUT) - constraint value  
     * @param grad (OUTPUT) - gradient vector
     */
    void evaluateWithGradient(Real* C, Real* grad) const override;

    /** Returns false - unified constraint is always active (no dead zone) */
    inline bool isInequality() const override { return false; }
    
    /** Get current separation distance between rigid body point and triangle surface */
    Real getCurrentDistance() const;
    
    /** Invalidate cached contact frame at start of new timestep */
    void resetCache() const { _cache_valid = false; }
    
    /** Check if constraint should break */
    bool shouldBreak() const { return _should_break; }
    Real getInitialDistance() const { return _initial_distance; }
    Real getBreakThreshold() const { 
        // Return actual breaking distance (not old formula)
        // Breaking occurs when: d > initial_distance + max_allowed_stretch
        const Real stretch_tolerance = _initial_distance * (_break_ratio - 1.0);
        const Real max_allowed_stretch = std::max(stretch_tolerance, _stretch_abs_min);
        return _initial_distance + max_allowed_stretch;
    }
    
    /** Get point on rigid body in body coordinates */
    const Vec3r& rigidBodyPoint() const { return _rigid_body_point; }
    
    /** Mark constraint for breaking (can be called externally for geometric interference)
     * This allows external logic (e.g., cutting tools) to break adhesion constraints
     */
    void markForBreaking() { _should_break = true; }

protected:
    /** Compute signed distance from rigid body point to triangle
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

    /** Compute target distance d*(d) using validated mathematical formulation.
     * Stage 1: contact → rest (smoothstep blend)
     * Stage 2: rest → bond (C¹ exponential blend with delayed start)
     * @param d - current distance
     * @return target distance d*(d)
     */
    Real computeTargetDistance(Real d) const;

    /** C¹ continuous smoothstep function: t²(3-2t)
     * Note: C¹ continuous (first derivative continuous), not C²
     * @param edge0 - start of transition
     * @param edge1 - end of transition
     * @param x - input value
     * @return blended value in [0,1]
     */
    Real smoothstep(Real edge0, Real edge1, Real x) const;

    /** C¹ exponential blend with delayed start (avoids gate' * exp spike)
     * @param d0 - start of transition
     * @param s - scale parameter (controls max slope)
     * @param d - current distance
     * @param gate_width - width of smoothstep gate
     * @return blended value in [0,1]
     */
    Real expBlend(Real d0, Real s, Real d, Real gate_width) const;

private:
    const Geometry::SDF* _sdf;      ///< SDF of rigid object (optional)
    Vec3r _rigid_body_point;         ///< Point on rigid body in body coordinates
    
    // Curve parameters (configurable via constructor/YAML)
    const Real _d_contact;         ///< Equilibrium distance (where C=0)
    const Real _d_rest;            ///< Mid-range target distance
    const Real _d_neutral_start;   ///< Transition zone start
    const Real _d_neutral_end;     ///< Transition zone end
    const Real _d_bond;            ///< Saturation distance (far adhesion limit)
    const Real _stretch_abs_min;   ///< Absolute minimum stretch tolerance (tissue intrinsic toughness)
    
    // Cached values for frozen contact frame approach (mutable for const methods)
    mutable Vec3r _n_cached;         ///< unit normal (closest_point to rigid_point, geometric gradient direction)
    mutable Vec3r _bary_cached;      ///< barycentric coordinates [b1, b2, b3]  
    mutable Vec3r _xs_cached;        ///< closest point on triangle surface
    mutable Real  _constraint_value_cached{0.0}; ///< C = d - d*(d)
    mutable Real  _separation_cached{0.0};       ///< current separation distance (signed: + = separated, - = penetrating)
    mutable Real  _dC_dd_cached{1.0};            ///< dC/dd = 1 - dd*/dd (for gradient scaling)
    mutable bool  _cache_valid{false};           ///< whether cached values are valid
    
    // Breaking logic (mutable because computed on first evaluate())
    mutable Real _initial_distance{0.0};     ///< Distance when first evaluated (computed lazily)
    Real _break_ratio{3.0};                  ///< Break when d > _initial_distance * _break_ratio
    mutable bool _should_break{false};       ///< Flag to mark constraint for removal
    
    // Debug: Track if _rigid_body_point changes across frames (should NEVER change)
    mutable Vec3r _debug_prev_rigid_body_point{Vec3r::Zero()}; ///< Previous frame's rigid body point (body coords)
    mutable Vec3r _debug_prev_rigid_global{Vec3r::Zero()};     ///< Previous frame's rigid global point
    mutable int _debug_frame_count{0};                         ///< Frame counter for this constraint
    mutable bool _debug_prev_cache_valid{false};               ///< Previous cache_valid state (for detecting new frame)
    mutable bool _debug_initialized{false};                    ///< Whether debug tracking started
};

} // namespace Solver

#endif // __UNIFIED_DISTANCE_CONSTRAINT_HPP
