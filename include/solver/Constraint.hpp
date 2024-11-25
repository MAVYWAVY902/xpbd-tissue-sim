#ifndef __CONSTRAINT_HPP
#define __CONSTRAINT_HPP

#include <vector>
#include <Eigen/Dense>

#include "simobject/XPBDMeshObject.hpp"

namespace Solver
{

/** Struct for storing references to positions in MeshObjects.
 * Used by Constraints to access node properties dynamically.
 */
struct PositionReference
{
    XPBDMeshObject* obj;        // pointer to the MeshObject
    unsigned index;             // vertex index
    
    PositionReference()
        : obj(0), index(0)
    {}

    PositionReference(XPBDMeshObject* obj_, unsigned index_)
        : obj(obj_), index(index_)
    {}

    /** Helper function to get the vertex position. */
    inline Eigen::Vector3d position() const
    {
        return obj->getVertex(index);
    }

    /** Helper function to get the vertex's previous position. */
    inline Eigen::Vector3d previousPosition() const
    {
        return obj->vertexPreviousPosition(index);
    }

    /** Helper function to get the vertex velocity. */
    inline Eigen::Vector3d velocity() const
    {
        return obj->vertexVelocity(index);
    }

    /** Helper function to get the vertex mass. */
    inline double mass() const
    {
        return obj->vertexMass(index);
    }

    /** Helper function to get the vertex's inverse mass. */
    inline double invMass() const
    {
        return obj->vertexInvMass(index);
    }

    /** Helper function to get number of attached elements to the vertex. */
    inline unsigned attachedElements() const
    {
        return obj->vertexAttachedElements(index);
    }

    friend bool operator==(const PositionReference& lhs, const PositionReference& rhs)
    {
        return lhs.obj == rhs.obj && lhs.index == rhs.index;
    }
};

class Constraint
{
    friend class ConstraintDecorator;
    public:

    typedef std::pair<double, Eigen::VectorXd> ValueAndGradient;
    // typedef std::pair<Eigen::VectorXd, Eigen::MatrixXd> ValueAndGradient;

    Constraint(const double dt, std::vector<PositionReference> positions)
        : _alpha(0), _positions(positions)
    {
        _gradient_vector_position.resize(_positions.size());
        for (unsigned i = 0; i < _positions.size(); i++)
        {
            _gradient_vector_position[i] = 3*i;
            _gradient_vector_size = _positions.size() * 3;
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
     */
    inline virtual ValueAndGradient evaluateWithGradient() const = 0;
    
    /** Returns the compliance for this constraint. */
    double alpha() const { return _alpha; }

    const std::vector<PositionReference>& positions() const { return _positions; }

    void setGradientVectorSize(const unsigned size) { _gradient_vector_size = size; }

    void setGradientVectorPosition(const unsigned position_index, const unsigned gradient_index) { _gradient_vector_position[position_index] = gradient_index; }

    protected:
    double _alpha;      // Compliance for this constraint
    unsigned _gradient_vector_size; // size of the returned gradient vector
    std::vector<unsigned> _gradient_vector_position; // maps position indices to positions in gradient vector
    std::vector<PositionReference> _positions;  // the positions associated with this constraint
};  


} // namespace Solver

#endif // __CONSTRAINT_HPP