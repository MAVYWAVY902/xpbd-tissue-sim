#ifndef __POSITION_REFERENCE_HPP
#define __POSITION_REFERENCE_HPP

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

} // namespace Solver

#endif // __POSITION_REFERENCE_HPP