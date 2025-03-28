#ifndef __XPBD_SOLVER_HPP
#define __XPBD_SOLVER_HPP

#include "simobject/XPBDMeshObjectBase.hpp"
#include "common/XPBDEnumTypes.hpp"

#include "solver/Constraint.hpp"
#include "solver/ConstraintProjector.hpp"
#include "solver/CombinedConstraintProjector.hpp"
#include "solver/XPBDSolverUpdates.hpp"

#include "common/VariadicVectorContainer.hpp"

namespace Solver
{

template<typename ...ConstraintProjectors>
class XPBDSolver
{
    public:
    using projector_type_list = TypeList<ConstraintProjectors...>;

    explicit XPBDSolver(Sim::XPBDMeshObject_Base* obj, int num_iter, XPBDResidualPolicy residual_policy)
        : _obj(obj), _num_iter(num_iter), _residual_policy(residual_policy), _constraints_using_primary_residual(false), _num_constraints(0)
    {
        _rigid_body_updates.resize(14); // 14 doubles is enough to store 2 rigid body updates (no more than 2 rigid bodies will be involved in a single constraint projection)
    }

    ~XPBDSolver() = default;

    void setup()
    {
        // resize the primary residual vector according to the MeshObject's number of vertices
        // TODO: should there be a generalized numCoordinates() method for XPBDObjects?
        _primary_residual.resize(3*_obj->mesh()->numVertices());
    }   

    /** Project the constraints and apply the position updates.
     * Generally consists of constraint initialization, a solver loop that will run num_iter times, and optional residual calculation.
     */
    void solve()
    {
        _inertial_positions = _obj->mesh()->vertices();

        // initialize all the constraints
        _constraint_projectors.for_each_element([&](auto& proj)
        {
            if (proj.isValid())
            {
                proj.initialize();
            }
        });

        for (int i = 0; i < _num_iter; i++)
        {
            // if any of the constraint proectors require the primary residual, we need to calculate it
            if (_constraints_using_primary_residual)
            {
                _calculatePrimaryResidual();
            }

            // iterate through the constraints and solve them
            _iterateConstraints();
        }

        // calculate the residual after this step if required by the residual policy
        if (_residual_policy == XPBDResidualPolicy::EVERY_SUBSTEP)
        {
            _calculatePrimaryResidual();
            _calculateConstraintResidual();
        }
    }

    /** Returns the current primary residual (Equation (8) from XPBD paper).
     * Does NOT recalculate if stale.
     */
    const VecXr primaryResidual() const { return Eigen::Map<const VecXr>(_primary_residual.data(), _primary_residual.size()); };

    /** Returns the current constraint residual (Equation (9) from XPBD paper).
     * Does NOT recalculate if stale.
     */
    const VecXr constraintResidual() const { return Eigen::Map<const VecXr>(_constraint_residual.data(), _constraint_residual.size()); };

    /** Returns the number of solver iterations. */
    int numIterations() const { return _num_iter; }

    template<class... Constraints>
    void addConstraintProjector(Real dt, const ConstraintProjectorOptions& options, Constraints* ... constraints)
    {
        auto projector = _createConstraintProjector(dt, options, constraints...);
    
        // amount of pre-allocated memory required to perform the constraint(s) projection
        // size_t required_array_size = projector.memoryNeeded() / sizeof(Real);
        // // make sure that the data buffer of the _data vector is large enough to accomodate the new projector
        // if (required_array_size > _data.size())
        // {
        //     _data.resize(required_array_size);
        // }
    
        // make sure that the data buffer of the coordinate updates vector is large enough to accomodate the new projector
        if (static_cast<unsigned>(projector.numCoordinates()) > _coordinate_updates.size())
        {
            _coordinate_updates.resize(projector.numCoordinates());
        }
    
        // increase the total number of constraints (needed for the constraint residual size)
        _num_constraints += decltype(projector)::NUM_CONSTRAINTS;
    
        // check if primary residual is needed for constraint projection
        if (options.with_residual)
        {
            // TODO: FIX THIS for the NEW APPROACH!!!

            // _constraints_using_primary_residual = true;
            // if (WithDistributedPrimaryResidual* wpr = dynamic_cast<WithDistributedPrimaryResidual*>(projector.get()))
            // {
            //     wpr->setPrimaryResidual(_primary_residual.data());
            // }
        }
            
        _constraint_projectors.template push_back<decltype(projector)>(std::move(projector));
    }

    protected:

    /** Helper function that will perform 1 iteration of the solver. 
     * This method is pure virtual because its implementation depends on the solver type (Gauss-Seidel, Jacobi, etc.) to know what to do with the position updates given by the ConstraintProjectors.
     * @param data - the pre-allocated data block to use for evaluating the constraints and their gradients. Assumes that it is large enough to accomodate the ConstraintProjector with the largest memory requirement.
     */
    virtual void _iterateConstraints() = 0;
    
    /** Calculates the primary residual (Equation (8) from XPBD paper). */
    // TODO: Actually implement this
    void _calculatePrimaryResidual() {}

    /** Calculates the constraint residual (Equation (9) from XPBD paper). */
    // TODO: Actually implement this
    void _calculateConstraintResidual() {}

    
    template <class Constraint>
    ConstraintProjector<Constraint> _createConstraintProjector(Real dt, const ConstraintProjectorOptions& /* options */, Constraint* constraint)
    {
        ConstraintProjector<Constraint> projector = ConstraintProjector(dt, constraint);

        return projector;
    }

    template <class Constraint1, class Constraint2>
    CombinedConstraintProjector<Constraint1, Constraint2> _createConstraintProjector(Real dt, const ConstraintProjectorOptions& /* options */, Constraint1* constraint1, Constraint2* constraint2)
    {
        CombinedConstraintProjector<Constraint1, Constraint2> projector = CombinedConstraintProjector(dt, constraint1, constraint2);

        return projector;
    }

    protected:
    VariadicVectorContainer<ConstraintProjectors...> _constraint_projectors;

    Sim::XPBDMeshObject_Base* _obj;                                 // pointer to the XPBDMeshObject that owns this Solver and is updated by the solver loop
    int _num_iter;                                         // number of solver iterations per solve() call
    Geometry::Mesh::VerticesMat _inertial_positions;                // stores the positions after the inertial update - useful for primary residual calculation

    XPBDResidualPolicy _residual_policy;                        // determines how often to calculate the residuals - this varies depending on the information needs of the user

    bool _constraints_using_primary_residual;                   // whether or not any of the ConstraintProjectors require the primary residual to do the position update

    int _num_constraints;                                  // total number of constraints projected (note this may be different from number of ConstraintProjectors if some constraints are solved simultaneously)

    std::vector<Real> _primary_residual;                      // primary residual vector
    std::vector<Real> _constraint_residual;                   // constraint residual vector - use std::vector instead of VecXr to minimize dynamic reallocations as number of constraints change

    // mutable std::vector<Real> _data;                          // the vector class is used to pre-allocate data for the solver loop
    std::vector<CoordinateUpdate> _coordinate_updates;                    // stores updates to the coordinates determined by the constraint projections - also pre-allocated. Needs to be have a size >= the max number of positions affected by a single constraint projection.
    std::vector<RigidBodyUpdate> _rigid_body_updates;                    // stores updates to any rigid bodies involved in the constraint projections - also pre-allocated. Each rigid body update consists of a position update and an orientation update, which is 7 doubles.
};

} // namespace Solver

#endif // __XPBD_SOLVER_HPP