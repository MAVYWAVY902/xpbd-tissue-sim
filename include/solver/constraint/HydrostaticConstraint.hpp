#ifndef __HYDROSTATIC_CONSTRAINT_HPP
#define __HYDROSTATIC_CONSTRAINT_HPP

#include "solver/constraint/ElementConstraint.hpp"
#include "simobject/ElasticMaterial.hpp"

#ifdef HAVE_CUDA
#include "gpu/constraint/GPUHydrostaticConstraint.cuh"
#endif

namespace Solver
{

/** Represents the hydrostatic constraint derived from the Stable Neo-hookean strain energy, proposed by Macklin et. al: https://mmacklin.com/neohookean.pdf
 */
class HydrostaticConstraint : public ElementConstraint
{
    friend class CombinedNeohookeanConstraintProjector;
    public:
    constexpr static int NUM_POSITIONS = 4; 
    constexpr static int NUM_COORDINATES = 12;
    
    public:
    /** Creates the hydrostatic constraint from a MeshObject and the 4 vertices that make up the tetrahedral element. */
    HydrostaticConstraint(int v1, Real* p1, Real m1,
                          int v2, Real* p2, Real m2,
                          int v3, Real* p3, Real m3,
                          int v4, Real* p4, Real m4,
                          const ElasticMaterial& material);

    int numPositions() const override { return NUM_POSITIONS; }
    int numCoordinates() const override { return NUM_COORDINATES; }
    bool isInequality() const override { return false; }


    /** Evaluates the current value of this constraint with pre-allocated memory.
     * i.e. returns C(x)
     * 
     * @param C (OUTPUT) - the pointer to the (currently empty) value of the constraint
     */
    inline void evaluate(Real* C) const override;

    /** Computes the gradient of this constraint in vector form with pre-allocated memory.
     * i.e. returns delC(x)
     * 
     * @param grad (OUTPUT) - the pointer to the (currently empty) constraint gradient vector. Expects it to be _gradient_vector_size x 1.
     */
    inline void gradient(Real* grad) const override;


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
    typedef GPUHydrostaticConstraint GPUConstraintType;
    GPUConstraintType createGPUConstraint() const;
    #endif

    private:

    /** Helper method to evaluate the constraint given the deformation gradient, F, using pre-allocated memory.
     * Avoids the need to recompute F if we already have it.
     */
    inline void _evaluate(Real* C, Real* F) const;

    /** Helper method to evaluate the constraint gradient given the deformation gradient, F, useing pre-allocated memory.
     * Avoids the need to recompute F if we already have it.
     * 
     * Requires at least 9 * sizeof(Real) bytes of additional memory for intermediate quantities.
     */
    inline void _gradient(Real* delC, Real* F) const;

    /** Helper method for cross product between two 3-vectors v1 and v2, store the result in v3 */
    inline void _cross3(const Real* v1, const Real* v2, Real* v3) const;

    protected:
    Real _gamma;  // the ratio mu/lambda - part of C(x)
};

} // namespace Solver

#endif // __HYDROSTATIC_CONSTRAINT_HPP
