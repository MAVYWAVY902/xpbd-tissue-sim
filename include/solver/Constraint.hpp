#ifndef __CONSTRAINT_HPP
#define __CONSTRAINT_HPP

#include <vector>
#include <Eigen/Dense>

#include "simobject/XPBDMeshObject.hpp"

namespace Solver
{

/** Struct for storing references to positions in MeshObjects.
 * Used by Constraints to access node properties dynamically.
 * 
 * Direct pointers to vertex data are used to skip dereferencing multiple pointers, which is not only faster but more cache-friendly.
 */
struct PositionReference
{
    XPBDMeshObject* obj;        // pointer to the MeshObject the position belongs to
    unsigned index;             // the index of this position in the array of vertices
    double* position_ptr;       // a direct pointer to the position - points to a data block owned by obj's vertices matrix
    double* prev_position_ptr;  // a direct pointer to the previous position - points to a data block owned by obj's previous positions matrix
    double inv_mass;            // store the inverse mass for quick lookup
    double num_constraints;     // number of constraints affect this position - stored here for quick lookup

    /** Default constructor */
    PositionReference()
        : obj(0), index(0), position_ptr(0), prev_position_ptr(0), inv_mass(0), num_constraints(0)
    {}

    /** Constructor that initializes quantities from just an object pointer and index. */
    PositionReference(XPBDMeshObject* obj_, unsigned index_)
        : obj(obj_), index(index_)
    {
        position_ptr = obj->getVertexPointer(index);
        prev_position_ptr = obj->getVertexPreviousPositionPointer(index);
        inv_mass = obj->vertexInvMass(index);
        // NOTE: this requires knowing how many constraints a position will be a part of a priori - maybe is not the case all the time
        num_constraints = obj->numConstraintsForPosition(index);
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

    typedef std::pair<double, Eigen::VectorXd> ValueAndGradient;

    /** Create a constraint from PositionReferences and an associated compliance (which is by default 0, indicating a hard constraint). */
    Constraint(std::vector<PositionReference> positions, const double alpha = 0)
        : _alpha(alpha), _positions(positions)
    {
        // default information about how the gradient vector should be formatted (see variables for more description)
        _gradient_vector_size = numCoordinates();
        _gradient_vector_index.resize(_gradient_vector_size);
        for (unsigned i = 0; i < numCoordinates(); i++)
        {
            _gradient_vector_index[i] = i;
        }
    }

    /** Evaluates the current value of this constraint.
     * i.e. returns C(x)
     */
    inline virtual double evaluate() const = 0;

    /** Returns the gradient of this constraint in vector form.
     * i.e. returns delC(x)
     */
    inline virtual Eigen::VectorXd gradient() const = 0;

    /** Returns the value and gradient of this constraint.
     * i.e. returns C(x) and delC(x) together.
     * 
     * This may be desirable when there would be duplicate work involved to evaluate constraint and its gradient separately.
     */
    inline virtual ValueAndGradient evaluateWithGradient() const = 0;


    /** Evaluates the current value of this constraint with pre-allocated memory.
     * i.e. returns C(x)
     * 
     * @param C (OUTPUT) - the pointer to the (currently empty) value of the constraint
     * @param additional_memory - a pointer to some pre-allocated memory that will be used to compute intermediate values in the constraint calculation, if necessary
     */
    inline virtual void evaluate(double* C, double* additional_memory) const = 0;


    /** Computes the gradient of this constraint in vector form with pre-allocated memory.
     * i.e. returns delC(x)
     * 
     * @param grad (OUTPUT) - the pointer to the (currently empty) constraint gradient vector. Expects it to be _gradient_vector_size x 1.
     * @param additional_memory - a pointer to some pre-allocated memory that will be used to compute intermediate values in the constraint gradient calculation, if necessary
     */
    inline virtual void gradient(double* grad, double* additional_memory) const = 0;


    /** Computes the value and gradient of this constraint with pre-allocated memory.
     * i.e. returns C(x) and delC(x) together.
     * 
     * This may be desirable when there would be duplicate work involved to evaluate constraint and its gradient separately.
     * 
     * @param C (OUTPUT) - the pointer to the (currently empty) value of the constraint
     * @param grad (OUTPUT) - the pointer to the (currently empty) constraint gradient vector. Expects it to be _gradient_vector_size x 1.
     * @param additional_memory - a pointer to some pre-allocated memory that will be used to compute intermediate values, if necessary
     */
    inline virtual void evaluateWithGradient(double* C, double* grad, double* additional_memory) const = 0;


    /** Returns the number of distinct positions needed by this constraint. */
    inline unsigned numPositions() { return _positions.size(); }

    /** Returns the number of coordinates (i.e. x1,y1,z1,x2,y2,z2) referenced by this constraint. In 3D, this is numPositions times 3. */
    inline unsigned numCoordinates() { return _positions.size()*3; }

    /** Returns the number of bytes of pre-allocated dynamic memory needed to do its computation. */
    inline virtual unsigned memoryNeeded() const = 0;
    
    /** Returns the compliance for this constraint. */
    double alpha() const { return _alpha; }

    /** Returns the positions involved in this constraint. */
    const std::vector<PositionReference>& positions() const { return _positions; }

    /** Sets the size of the gradient vector. */
    void setGradientVectorSize(const unsigned size) { _gradient_vector_size = size; }

    /** Updates the mapping of coordinate indices to their respective positions in the gradient vector.
     * @param coord_index : the coordinate index to be mapped (i.e. in 3D, coord_index of 7 would correspond to the y-coordinate of the 3rd position)
     * @param gradient_index : the index in the gradient vector that the coordinate should be mapped to
     */
    void setGradientVectorIndex(const unsigned coord_index, const unsigned gradient_index) { _gradient_vector_index[coord_index] = gradient_index; }

    protected:
    double _alpha;      // Compliance for this constraint

    /** Size of the gradient vector.
     * By default, the size of the gradient vector is simply the number of coordinates affected by this constraint.
     * 
     * The size of the gradient vector may be set larger to align with other constraints being projected simulatenously.
     */
    unsigned _gradient_vector_size;

    /** Maps coordinate indices to their respective positions in the gradient vector.
     * For example, if _gradient_vector_index[0] = 5, coordinate 0 (i.e. the x-coordinate of _positions[0]) is mapped to index 5 in the gradient vector.
     * 
     * By default, coordinate 0 will be mapped to index 0 in the gradient vector, coordinate 1 will be mapped to index 1 in the gradient vector, etc.
     * 
     * A non-default mapping exists to align with other constraints being projected simultaneously so that all constraints have the same position order in the gradient vector.
     */
    std::vector<unsigned> _gradient_vector_index;


    std::vector<PositionReference> _positions;  // the positions associated with this constraint
};  


} // namespace Solver

#endif // __CONSTRAINT_HPP