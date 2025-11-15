#ifndef __NERVE_TUMOR_ADHESION_CONSTRAINT_HPP
#define __NERVE_TUMOR_ADHESION_CONSTRAINT_HPP

#include "solver/constraint/Constraint.hpp"

namespace Solver
{

/** Adhesion constraint between a nerve vertex (point) and a tumor face (triangle).
 * Implements distance-threshold adhesion with target gap d_0.
 * 
 * Phase 1: Simple distance-based adhesion within window [0, d_a] pulling toward target gap d_0
 * - Constraint function: C(q) = n^T(q - p_surface) - d_0
 * - Where p_surface is the closest point on the triangle computed via barycentric coordinates
 * - Attractive force when distance < d_a, breaking when distance > d_break
 */
class NerveTumorAdhesionConstraint : public Constraint
{
    public:
    // One nerve vertex + three triangle vertices = 4 positions
    static constexpr int NUM_POSITIONS = 4;
    static constexpr int NUM_COORDINATES = 12; // 4 positions × 3 coordinates each

    public:
    /** Constructor for nerve-tumor adhesion constraint.
     * @param nerve_v - nerve vertex index  
     * @param nerve_p - nerve vertex position pointer
     * @param nerve_m - nerve vertex mass
     * @param tri_v1, tri_v2, tri_v3 - tumor triangle vertex indices
     * @param tri_p1, tri_p2, tri_p3 - tumor triangle vertex position pointers  
     * @param tri_m1, tri_m2, tri_m3 - tumor triangle vertex masses
     * @param target_gap - target separation distance d_0
     * @param alpha - compliance parameter
     */
    NerveTumorAdhesionConstraint(int nerve_v, Real* nerve_p, Real nerve_m,
                                int tri_v1, Real* tri_p1, Real tri_m1,
                                int tri_v2, Real* tri_p2, Real tri_m2, 
                                int tri_v3, Real* tri_p3, Real tri_m3,
                                Real target_gap,
                                Real alpha = 0.0);

    int numPositions() const override { return NUM_POSITIONS; }
    int numCoordinates() const override { return NUM_COORDINATES; }

    /** Evaluates the adhesion constraint: C(q) = n^T(q - p_surface) - d_0
     * @param C (OUTPUT) - constraint value
     */
    void evaluate(Real* C) const override;

    /** Computes gradient of adhesion constraint distributed across nerve vertex and triangle vertices
     * @param grad (OUTPUT) - gradient vector [grad_nerve, grad_tri1, grad_tri2, grad_tri3]
     */
    void gradient(Real* grad) const override;

    /** Combined evaluation and gradient computation for efficiency
     * @param C (OUTPUT) - constraint value  
     * @param grad (OUTPUT) - gradient vector
     */
    void evaluateWithGradient(Real* C, Real* grad) const override;

    /** Adhesion constraints are equality constraints (not inequalities like collision) */
    inline bool isInequality() const override { return false; }

    /** Get target separation distance */
    Real getTargetGap() const { return _target_gap; }

    /** Update target gap (useful for dynamic adhesion strength) */
    void setTargetGap(Real target_gap) { _target_gap = target_gap; }

    protected:
    /** Compute signed distance from nerve point to triangle and closest point info
     * @param nerve_pos - nerve vertex position
     * @param tri_p1, tri_p2, tri_p3 - triangle vertex positions
     * @param closest_point (OUTPUT) - closest point on triangle surface
     * @param normal (OUTPUT) - unit normal vector (triangle to point)
     * @param bary_coords (OUTPUT) - barycentric coordinates [u, v, w] where u+v+w=1
     * @return signed distance (positive = separated, negative = penetrating)
     */
    Real computePointTriangleDistance(const Vec3r& nerve_pos,
                                     const Vec3r& tri_p1, 
                                     const Vec3r& tri_p2,
                                     const Vec3r& tri_p3,
                                     Vec3r& closest_point,
                                     Vec3r& normal,
                                     Vec3r& bary_coords) const;

    private:
    Real _target_gap; ///< Target separation distance d_0
    
    // Cached values for frozen contact frame approach (mutable for const methods)
    mutable Vec3r _n_cached;      ///< unit normal (triangle to point)
    mutable Vec3r _bary_cached;   ///< barycentric coordinates [b1, b2, b3]  
    mutable Vec3r _xs_cached;     ///< closest point on triangle surface
    mutable bool  _cache_valid{false}; ///< whether cached values are valid
};

} // namespace Solver

#endif // __NERVE_TUMOR_ADHESION_CONSTRAINT_HPP