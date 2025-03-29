#ifndef __CONSTRAINT_HPP
#define __CONSTRAINT_HPP

#include <vector>
#include "common/types.hpp"

namespace Solver
{

/** Struct for storing references to positions in MeshObjects.
 * Used by Constraints to access node properties dynamically.
 * 
 * Direct pointers to vertex data are used to skip dereferencing multiple pointers, which is not only faster but more cache-friendly.
 */
struct PositionReference
{
    // const Sim::XPBDMeshObject_Base* obj;        // pointer to the MeshObject the position belongs to
    int index;             // the index of this position in the array of vertices
    Real* position_ptr;       // a direct pointer to the position - points to a data block owned by obj's vertices matrix
    // Real* prev_position_ptr;  // a direct pointer to the previous position - points to a data block owned by obj's previous positions matrix
    Real inv_mass;            // store the inverse mass for quick lookup
    // Real num_constraints;     // number of constraints affect this position - stored here for quick lookup

    /** Default constructor */

    PositionReference()
        : position_ptr(0), inv_mass(0)
    {}

    /** Constructor that initializes quantities from just an object pointer and index. */
    PositionReference(int index_, Real* position_ptr_, Real mass_)
        : index(index_), position_ptr(position_ptr_), inv_mass(1.0/mass_)
    {
    }

    /** Two PositionReferences are equal if they point to the same position in memory. */
    friend bool operator== (const PositionReference& lhs, const PositionReference& rhs)
    {
        return lhs.position_ptr == rhs.position_ptr;
    }
};



////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////



/** Represents a constraint in a general sense.
 * 
 * A constraint is basically some function C(x) that returns a single number, where "x" is some state.
 * Usually, the state "x" involves multiple vertices/positions in the mesh/system, and is not limited to positions from a single object (e.g. collision constraints often involve different objects).
 * The constraint does not own the position data, but rather has a reference to it via the PositionReference class.
 * 
 * The constraint class itself is abstract, and provides pure virtual methods for the evaluation of the constraint function (i.e. C(x) ) and the evaluation of the gradient of the constraint function (i.e. delC(x)).
 * There are two types of these pure virtual methods:
 *   - No pre-allocation - these methods do not work with pre-allocated memory and have return values. This is cleaner and easier to understand/implement but is slow in the critical path.
 *   - With pre-allocation - these methods work with a set block of pre-allocated memory. This memory only has to be allocated once at the beginning of the program and speeds up the solver loop.
 * 
 */
class Constraint
{
    public:

    /** Create a constraint from PositionReferences and an associated compliance (which is by default 0, indicating a hard constraint). */
    Constraint(const std::vector<PositionReference>& positions, const Real alpha = 0)
        : _alpha(alpha), _positions(positions)
    {
    }

    virtual ~Constraint() = default;


    /** Evaluates the current value of this constraint with pre-allocated memory.
     * i.e. returns C(x)
     * 
     * @param C (OUTPUT) - the pointer to the (currently empty) value of the constraint
     */
    inline virtual void evaluate(Real* C) const = 0;


    /** Computes the gradient of this constraint in vector form with pre-allocated memory.
     * i.e. returns delC(x)
     * 
     * @param grad (OUTPUT) - the pointer to the (currently empty) constraint gradient vector. Expects it to be _gradient_vector_size x 1.
     */
    inline virtual void gradient(Real* grad) const = 0;


    /** Computes the value and gradient of this constraint with pre-allocated memory.
     * i.e. returns C(x) and delC(x) together.
     * 
     * This may be desirable when there would be duplicate work involved to evaluate constraint and its gradient separately.
     * 
     * @param C (OUTPUT) - the pointer to the (currently empty) value of the constraint
     * @param grad (OUTPUT) - the pointer to the (currently empty) constraint gradient vector. Expects it to be _gradient_vector_size x 1.
     */
    inline virtual void evaluateWithGradient(Real* C, Real* grad) const = 0;


    /** Returns the number of distinct positions needed by this constraint. */
    inline virtual int numPositions() const = 0;

    /** Returns the number of coordinates (i.e. x1,y1,z1,x2,y2,z2) referenced by this constraint. In 3D, this is numPositions times 3. */
    inline virtual int numCoordinates() const = 0;
    
    /** Returns the compliance for this constraint. */
    Real alpha() const { return _alpha; }

    /** Returns true if this constraints is an inequality, false otherwise.
     * Usually, this constraints are equalities, i.e. C(x) = 0, so by default it returns false.
     * Sometimes, such as for collision constraints, we might want C(x) >= 0.
     */
    virtual bool isInequality() const = 0;

    /** Returns the positions involved in this constraint. */
    const std::vector<PositionReference>& positions() const { return _positions; }

    protected:
    Real _alpha;      // Compliance for this constraint

    std::vector<PositionReference> _positions;  // the positions associated with this constraint
};  


} // namespace Solver

#endif // __CONSTRAINT_HPP