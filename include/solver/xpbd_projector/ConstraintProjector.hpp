#ifndef __CONSTRAINT_PROJECTOR_HPP
#define __CONSTRAINT_PROJECTOR_HPP

#include "solver/constraint/DeviatoricConstraint.hpp"
#include "solver/xpbd_solver/XPBDSolverUpdates.hpp"

#include "common/TypeList.hpp"

#ifdef HAVE_CUDA
#include "gpu/projector/GPUConstraintProjector.cuh"
#endif

namespace Solver
{

/** Performs a XPBD constraint projection for a single constraint.
 *  
 * IsFirstOrder template parameter indicates if the projection follows the 1st-Order XPBD algorithm
 * 
 * Constraint template parameter is the type of constraint being projected (HydrostaticConstraint, DeviatoricConstraint, etc.)
*/
template<bool IsFirstOrder, class Constraint>
class ConstraintProjector
{
    public:
    /** Number of constraints projected */
    constexpr static int NUM_CONSTRAINTS = 1;
    /** Maximum number of coordinates involved in the contsraint projection */
    constexpr static int MAX_NUM_COORDINATES = Constraint::NUM_COORDINATES;

    /** List of constraint types being projected (will be a single constraint for this projector) */
    using constraint_type_list = TypeList<Constraint>;
    /** Whether or not the 1st-Order algorithm is used */
    constexpr static bool is_first_order = IsFirstOrder;

    public:
    /** Constructor */
    explicit ConstraintProjector(Real dt, Constraint* constraint_ptr)
        : _dt(dt), _constraint(constraint_ptr), _valid(true)
    {
    }

    /** Default constructor - projector marked invalid */
    explicit ConstraintProjector()
        : _valid(false)
    {   
    }

    void setValidity(bool valid) { _valid = valid; }
    bool isValid() const { return _valid; }

    int numCoordinates() { return MAX_NUM_COORDINATES; }

    /** Initialize the Lagrange multipliers */
    void initialize()
    {
        _lambda = 0;
    }

    /** Perform the XPBD constraint projection
     * @param coordinate_updates_ptr (OUTPUT) - a (currently empty) array of CoordinateUpdate structs of numCoordinates() size. Stores the resulting state updates from the XPBD projection. 
     */
    void project(CoordinateUpdate* coordinate_updates_ptr)
    {
        // evalute the constraint's value and gradient
        Real C;
        Real delC[Constraint::NUM_COORDINATES];
        _constraint->evaluateWithGradient(&C, delC);

        // if inequality constraint, make sure that the constraint should actually be enforced
        if (C > 0 && _constraint->isInequality())
        {
            for (int i = 0; i < Constraint::NUM_COORDINATES; i++) 
            { 
                coordinate_updates_ptr[i].ptr = nullptr;
                coordinate_updates_ptr[i].update = 0;
            }
            return;
        }

        // compute alpha_tilde
        // if using 1st order XPBD method - alpha_tilde = alpha / dt
        // if using 2nd order XPBD method - alpha_tilde = alpha / dt^2
        Real alpha_tilde;
        if constexpr(IsFirstOrder)
        {
            alpha_tilde = _constraint->alpha() / _dt;
        }
        else
        {
            alpha_tilde = _constraint->alpha() / (_dt * _dt);
        }

        // calculate LHS of lambda update: delC^T * M^-1 * delC + alpha_tilde
        Real LHS = alpha_tilde;
        const std::vector<PositionReference>& positions = _constraint->positions();
        
        for (int i = 0; i < Constraint::NUM_POSITIONS; i++)
        {
            LHS += positions[i].inv_mass * (delC[3*i]*delC[3*i] + delC[3*i+1]*delC[3*i+1] + delC[3*i+2]*delC[3*i+2]);
        }

        // compute RHS of lambda update: -C - alpha_tilde*lambda
        Real RHS = -C - alpha_tilde * _lambda;

        // std::cout << "\t\tRHS: " << RHS << " LHS: " << LHS << std::endl; 

        // compute lambda update
        Real dlam = RHS / LHS;
        _lambda += dlam;

        // compute position updates
        for (int i = 0; i < Constraint::NUM_POSITIONS; i++)
        {
            Real update_x = positions[i].inv_mass * delC[3*i] * dlam;
            Real update_y = positions[i].inv_mass * delC[3*i+1] * dlam;
            Real update_z = positions[i].inv_mass * delC[3*i+2] * dlam;

            // std::cout << "update: " << update_x << ", " << update_y << ", " << update_z << std::endl;
            
            coordinate_updates_ptr[3*i].ptr = positions[i].position_ptr;
            coordinate_updates_ptr[3*i].update = update_x;
            coordinate_updates_ptr[3*i+1].ptr = positions[i].position_ptr+1;
            coordinate_updates_ptr[3*i+1].update = update_y;
            coordinate_updates_ptr[3*i+2].ptr = positions[i].position_ptr+2;
            coordinate_updates_ptr[3*i+2].update = update_z;

            // if (positions[i].index == 337)
            // {
            //     std::cout << "Index 337 position update: "  << update_x << ", " << update_y << ", " << update_z << std::endl;
            // }
        }
    }

    #ifdef HAVE_CUDA
    typedef GPUConstraintProjector<IsFirstOrder, typename Constraint::GPUConstraintType> GPUConstraintProjectorType;
    GPUConstraintProjectorType createGPUConstraintProjector() const
    {
        typename Constraint::GPUConstraintType gpu_constraint = _constraint->createGPUConstraint();
        return GPUConstraintProjectorType(std::move(gpu_constraint), _dt);
    }
    #endif

    private:
    Real _dt;       // time step of the simulation
    Real _lambda;   // Lagrange multiplier for the projection
    Constraint* _constraint;    // the constraint being projected
    bool _valid;    // whether or not this projector is valid (if invalid, projection will not occur)
    

};


} // Solver

#endif // __CONSTRAINT_PROJECTOR_HPP