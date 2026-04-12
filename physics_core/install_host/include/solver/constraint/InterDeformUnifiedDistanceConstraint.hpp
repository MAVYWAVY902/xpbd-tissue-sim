#ifndef __INTER_DEFORM_UNIFIED_DISTANCE_CONSTRAINT_HPP
#define __INTER_DEFORM_UNIFIED_DISTANCE_CONSTRAINT_HPP

#include "solver/constraint/Constraint.hpp"
#include <algorithm>

namespace Solver
{

/** Unified distance constraint for deformable-deformable tissue adhesion.
 * 
 * This is the DEFORM-DEFORM version of UnifiedDistanceConstraint, using the same
 * smooth signed-distance curve but between a vertex on one mesh and a triangle on another.
 * 
 * DESIGN SCOPE: This is a **separation distance servo** for adhesion/attraction,
 * NOT a hard contact/penetration barrier. True collision prevention is handled
 * by collision constraints with inequality constraints + CCD.
 * 
 * Key innovation: Uses target distance formulation C(d) = d - d*(d) where:
 * - d: current separation distance (always >= 0, unsigned)
 * - d*(d): smooth target distance curve that transitions:
 *   * contact → rest → bond (saturation)
 * - C(d) = 0 implies d = d*(d) (equilibrium)
 * - C(d) < 0 implies too close (repulsion, but no hard barrier)
 * - C(d) > 0 implies too far (attraction)
 * 
 * Frozen Contact Frame: During XPBD solver iterations within a timestep,
 * the contact geometry (closest point, normal) is frozen to avoid feature jumping.
 * 
 * Mathematical guarantees:
 * - C¹ continuous (smooth forces, continuous gradients)
 * - Monotonic: dC/dd > 0 everywhere (unique equilibrium, XPBD stable)
 * - No dead zones (gradient always defined)
 */
class InterDeformUnifiedDistanceConstraint : public Constraint
{
public:
    // One vertex + three triangle vertices = 4 positions
    static constexpr int NUM_POSITIONS = 4;
    static constexpr int NUM_COORDINATES = 12; // 4 positions × 3 coordinates each

    // Distance parameters - CONFIGURABLE VIA YAML!
    static constexpr Real DEFAULT_D_CONTACT = 0.0003;       // 0.3mm - contact equilibrium
    static constexpr Real DEFAULT_D_REST = 0.0015;          // 1.5mm - mid-range target
    static constexpr Real DEFAULT_D_NEUTRAL_START = 0.003;  // 3mm - transition start
    static constexpr Real DEFAULT_D_NEUTRAL_END = 0.005;    // 5mm - transition end
    static constexpr Real DEFAULT_D_BOND = 0.015;           // 15mm - saturation
    static constexpr Real DEFAULT_BREAK_RATIO = 3.0;        // Default: break at 200% strain

public:
    /** Constructor for inter-deform unified distance constraint.
     * @param vertex_v - vertex index (from object A)
     * @param vertex_p - vertex position pointer (from object A)
     * @param vertex_m - vertex mass (from object A)
     * @param tri_v1, tri_v2, tri_v3 - triangle vertex indices (from object B)
     * @param tri_p1, tri_p2, tri_p3 - triangle vertex position pointers (from object B)
     * @param tri_m1, tri_m2, tri_m3 - triangle vertex masses (from object B)
     * @param alpha - compliance parameter
     * @param break_ratio - break when distance > initial_distance * break_ratio
     * @param initial_distance - PRECOMPUTED initial distance
     * @param d_contact - equilibrium distance (YAML configurable)
     * @param d_rest - mid-range target distance
     * @param d_neutral_start - transition zone start
     * @param d_neutral_end - transition zone end
     * @param d_bond - saturation distance
     * @param stretch_abs_min - absolute minimum stretch tolerance
     */
    InterDeformUnifiedDistanceConstraint(int vertex_v, Real* vertex_p, Real vertex_m,
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

    /** Evaluates the unified distance constraint: C(d) = d - d*(d) */
    void evaluate(Real* C) const override;

    /** Computes gradient of constraint distributed across vertex and triangle */
    void gradient(Real* grad) const override;

    /** Combined evaluation and gradient for efficiency */
    void evaluateWithGradient(Real* C, Real* grad) const override;

    /** This is an equality constraint (not inequality) */
    inline bool isInequality() const override { return false; }

    /** Check if bond should break based on stretch */
    bool shouldBreak() const { return _should_break; }

    /** Get current separation distance */
    Real getCurrentDistance() const;

    /** Get initial distance (set at creation) */
    Real getInitialDistance() const { return _initial_distance; }

    /** Get break threshold distance */
    Real getBreakThreshold() const;

    /** Reset state at start of new timestep */
    void resetMaxDistanceThisStep() const {
        _cache_valid = false;  // Force geometry recomputation
    }

    /** Mark constraint for breaking (external trigger). */
    void markForBreaking() { _should_break = true; }

    /** Weaken the break threshold based on tool proximity.
     * Reduces _break_ratio and _stretch_abs_min so the constraint is easier to break
     * via strain (mechanism 1), but still requires actual physical stretch.
     * @param factor - weakening factor in [0, 1]. 0 = no weakening, 1 = maximum weakening.
     */
    void weakenBreakThreshold(Real factor)
    {
        factor = std::max(Real(0), std::min(Real(1), factor));
        Real min_ratio = 1.0 + (_original_break_ratio - 1.0) * 0.02;
        _break_ratio = _original_break_ratio - factor * (_original_break_ratio - min_ratio);
        Real min_abs = _original_stretch_abs_min * 0.02;
        _stretch_abs_min = _original_stretch_abs_min - factor * (_original_stretch_abs_min - min_abs);
    }

    /** Reset break threshold to original values. */
    void resetBreakThreshold()
    {
        _break_ratio = _original_break_ratio;
        _stretch_abs_min = _original_stretch_abs_min;
    }

protected:
    /** Compute target distance d*(d) using slope-interpolation design */
    Real computeTargetDistance(Real d) const;

    /** Compute signed distance from vertex to triangle */
    Real computePointTriangleDistance(const Vec3r& vertex_pos,
                                     const Vec3r& tri_p1, 
                                     const Vec3r& tri_p2,
                                     const Vec3r& tri_p3,
                                     Vec3r& closest_point,
                                     Vec3r& normal,
                                     Vec3r& bary_coords) const;

private:
    // Curve parameters (configurable via YAML)
    const Real _d_contact;         ///< Equilibrium distance
    const Real _d_rest;            ///< Mid-range target distance
    const Real _d_neutral_start;   ///< Transition zone start
    const Real _d_neutral_end;     ///< Transition zone end
    const Real _d_bond;            ///< Saturation distance
    Real _stretch_abs_min;   ///< Absolute minimum stretch tolerance (modifiable by tool)
    
    // Cached values for frozen contact frame (mutable for const methods)
    mutable Vec3r _n_cached;         ///< unit normal (vertex to triangle)
    mutable Vec3r _bary_cached;      ///< barycentric coordinates (FROZEN at init)
    mutable Vec3r _xs_cached;        ///< closest point on triangle surface
    mutable Real  _constraint_value_cached{0.0}; ///< C = d - d*(d)
    mutable Real  _separation_cached{0.0};       ///< current separation distance
    mutable Real  _dC_dd_cached{1.0};            ///< dC/dd = 1 - dd*/dd
    mutable bool  _cache_valid{false};           ///< whether cached values are valid
    
    // Breaking logic
    mutable Real _initial_distance{0.0};     ///< Distance when first evaluated
    Real _break_ratio{3.0};                  ///< Break when d > initial * break_ratio
    Real _original_break_ratio{3.0};         ///< Original break ratio (for reset)
    Real _original_stretch_abs_min{0.005};   ///< Original stretch abs min (for reset)
    
    // Default Compliance (Soft) - stored to allow switching to Hard compliance for collision
    Real _default_alpha{0.0};

    mutable bool _should_break{false};       ///< Flag to mark constraint for removal
};

} // namespace Solver

#endif // __INTER_DEFORM_UNIFIED_DISTANCE_CONSTRAINT_HPP
