#ifndef __INTER_OBJECT_DEFORMABLE_COLLISION_CONSTRAINT_HPP
#define __INTER_OBJECT_DEFORMABLE_COLLISION_CONSTRAINT_HPP

#include "solver/constraint/CollisionConstraint.hpp"

// #ifdef HAVE_CUDA
// #include "gpu/constraint/GPUStaticDeformableCollisionConstraint.cuh"
// #endif

namespace Solver
{

/** 
 * Collision constraint between a vertex on one deformable object and a face on another deformable object.
 * 
 * This constraint is specifically for INTER-OBJECT collisions (collisions between two different deformable meshes).
 * It is separate from DeformableDeformableCollisionConstraint which is used for SELF-COLLISION detection
 * (vertex-face collisions within the same mesh).
 * 
 * The constraint enforces: C(x) = (q - p1) · n >= 0
 * where:
 *   - q is the vertex position from object 1
 *   - p1, p2, p3 are the triangle vertices from object 2's face
 *   - n is the triangle normal (p2-p1) × (p3-p1) normalized
 */
class InterObjectDeformableCollisionConstraint : public Constraint
{
    public:
    constexpr static int NUM_POSITIONS = 4;
    constexpr static int NUM_COORDINATES = 12;

    public:
    /** Constructor for inter-object deformable collision constraint.
     * 
     * @param v - index of the colliding vertex (from object 1)
     * @param p - pointer to vertex position (from object 1)
     * @param m - inverse mass of the vertex
     * @param fv1, fv2, fv3 - indices of the triangle vertices (from object 2)
     * @param fp1, fp2, fp3 - pointers to triangle vertex positions (from object 2)
     * @param fm1, fm2, fm3 - inverse masses of the triangle vertices
     */
    InterObjectDeformableCollisionConstraint(int v, Real* p, Real m,
                                            int fv1, Real* fp1, Real fm1,
                                            int fv2, Real* fp2, Real fm2,
                                            int fv3, Real* fp3, Real fm3);

    int numPositions() const override { return NUM_POSITIONS; }
    int numCoordinates() const override { return NUM_COORDINATES; }

    /** Evaluates the current value of this constraint with pre-allocated memory.
     * i.e. returns C(x)
     * 
     * @param C (OUTPUT) - the pointer to the (currently empty) value of the constraint
     */
    void evaluate(Real* C) const override;

    /** Computes the gradient of this constraint in vector form with pre-allocated memory.
     * i.e. returns delC(x)
     * 
     * @param grad (OUTPUT) - the pointer to the (currently empty) constraint gradient vector. Expects it to be _gradient_vector_size x 1.
     */
    void gradient(Real* delC) const override;


    /** Computes the value and gradient of this constraint with pre-allocated memory.
     * i.e. returns C(x) and delC(x) together.
     * 
     * This may be desirable when there would be duplicate work involved to evaluate constraint and its gradient separately.
     * 
     * @param C (OUTPUT) - the pointer to the (currently empty) value of the constraint
     * @param grad (OUTPUT) - the pointer to the (currently empty) constraint gradient vector. Expects it to be _gradient_vector_size x 1.
     */
    void evaluateWithGradient(Real* C, Real* grad) const override;

    #ifdef HAVE_CUDA
    typedef GPUStaticDeformableCollisionConstraint GPUConstraintType;
    GPUConstraintType createGPUConstraint() const;
    #endif

    /** Collision constraints should be implemented as inequalities, i.e. as C(x) >= 0. */
    inline virtual bool isInequality() const override { return true; }

    /** Applies a frictional force to the two colliding bodies given the coefficients of friction and the Lagrange multiplier from this constraint.
     * @param lam - the Lagrange multiplier for this constraint after the XPBD update
     * @param mu_s - the coefficient of static friction between the two bodies
     * @param mu_k - the coefficient of kinetic friction between the two bodies
     * 
     * NOTE: Friction is currently commented out but can be implemented in the future if needed for inter-object collisions.
     */
    // inline virtual void applyFriction(Real, Real, Real) const override;

};

} // namespace Solver

#endif // __INTER_OBJECT_DEFORMABLE_COLLISION_CONSTRAINT_HPP
