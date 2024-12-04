#ifndef __CONSTRAINT_PROJECTOR_HPP
#define __CONSTRAINT_PROJECTOR_HPP

#include "solver/Constraint.hpp"

namespace Solver
{

/** Responsible for projecting constraint(s) according to the XPBD algorithm.
 * Multiple constraints can be projected and solved for simultaneously.
 * 
 * The main external interface for this class is the project() method, which will solve the constraint(s) given the current state and return updates for each position involved.
 *   - The project() method works with pre-allocated memory, meaning that the whatever calls project() (i.e. the XPBDSolver class) must pre-allocate enough memory such that the computation can be performed.
 *   - The amount of pre-allocated memory required can be found with the memoryNeeded() method.
 */
class ConstraintProjector
{
    friend class ConstraintProjectorDecorator;

    public:

    explicit ConstraintProjector(std::vector<Constraint*> constraints, const double dt)
        : _constraints(std::move(constraints)), _dt(dt)
    {
        // resize Lagrange multipliers vector
        _lambda.resize(numConstraints());

        // we need to assemble a list of mesh positions that are affected by the constraints projected by this ConstraintProjector, so we can compute position updates (Delta x) for each one.
        // if there is only one constraint projected, this is easy - this is simply just the positions affected by that single constraint.
        // however, with multiple constraints, this gets a little trickier - we need the set of all positions affected, and there may be some overlap between constraints

        for (const auto& c : _constraints)
        {
            for (const auto& pref : c->positions())
            {
                // check if position is not accounted for yet, and if not, add it to the member positions vector
                if (std::find(_positions.begin(), _positions.end(), pref) == _positions.end())
                {
                    _positions.push_back(pref);
                }
            }
        }


        // now that we have all the positions affected by the projected constraints, we have to make sure that the constraints format their constraint gradient vectors properly
        // we want all constraints projected by this ConstraintProjector to compute gradient vectors that are not only the same size but also match up (i.e. the same index corresponds to the same coordinate)
        //  - e.g. if delC1[5] corresponds to the partial of C1 w.r.t. y2, then delC2[5] should correspond to the partial of C2 also w.r.t y2.
        // to make this happen, we need to:
        //  - tell each constraint the size of the gradient vector they should return (which will be the number of coordinates affected by this ConstraintProjector's constraints)
        //  - map the constraint's position indices to this ConstraintProjector's position indices

        // update gradient vector sizes for each constraint
        for (const auto& c : _constraints)
        {
            c->setGradientVectorSize(numCoordinates());              // set the size of the gradient vector to be the number of coordinates

            for (unsigned i = 0; i < c->numPositions(); i++)     // iterate through constraint's positions
            {
                const PositionReference& pref_i = c->positions()[i];
                // get index of constraint's position in the combined _positions vector of this ConstraintProjector
                unsigned p_index = std::distance(_positions.begin(), std::find(_positions.begin(), _positions.end(), pref_i));

                // map the ith coordinate in the constraint to the correct index in the constraint gradient vector
                c->setGradientVectorIndex(3*i, 3*p_index);
                c->setGradientVectorIndex(3*i+1, 3*p_index+1);
                c->setGradientVectorIndex(3*i+2, 3*p_index+2);
            }
        }
    }

    /** Initialize the constraint projection.
     * By default, this is just setting all Lagrange multipliers to 0.
     */
    inline virtual void initialize()
    {
        for (unsigned i = 0; i < numConstraints(); i++)
        {
            setLambda(i, 0);
        }
    }

    /** Projects the constraints and computes the position updates resulting from the projection.
     * Uses a pre-allocated data block to perform calculations to avoid incurring data allocation costs for intermediate values.
     * The pre-allocated data block will be used with the following structure:
     *  - Constraint vector (i.e. C(x)) - numConstraints x 1
     *  - delC matrix (i.e. delC(x)) - numConstraints x numCoordinates
     *  - inverse mass "matrix" (i.e. M^-1) - numPositions x 1 (matrix is diagonal and values repeat, so only a vector is needed)
     *  - alpha tilde "matrix" - numConstraints x 1 (matrix is diagonal, so only a vector is needed)
     *  - LHS matrix (i.e. delC * M^-1 * delC^T + alpha_tilde) - numConstraints x numConstraints
     *  - RHS matrix (i.e. -C - alpha tilde * lambda) - numConstraints x 1
     *  - Delta lambda vector - numConstraints x 1
     *  - Additional memory block - to be used by the Constraint class for intermediate values necessary to evaluate the constraint and its gradient (e.g. calculating the deformation gradient)
     * 
     * Helper functions exist to get the memory address of various quantities within the pre-allocated data block.
     * 
     * @param data_ptr - a pointer to a block of pre-allocated data, assumed to be at least as large as that given by memoryNeeded(). Used to store results from computations, but not necessarily to be used as an output parameter.
     * @param coordinate_updates_ptr (OUTPUT) - a pointer to an array of "coordinate updates" with structure [Delta x1, Delta y1, Delta z1, Delta x2, Delta y2, Delta z2, etc.). Assumed to be at least numCoordintes() x 1. 
     */
    inline virtual void project(double* data_ptr, double* coordinate_updates_ptr)
    {
        // point the data member variable to point to the pre-allocated data block
        _data = data_ptr;

        // extract pointers to the individual vectors/matrices to be stored in the pre-allocated data block
        double* C_ptr = _C_ptr();
        double* delC_ptr = _delC_ptr();
        double* M_inv_ptr = _M_inv_ptr();
        double* alpha_tilde_ptr = _alpha_tilde_ptr();
        double* lhs_ptr = _LHS_ptr();
        double* rhs_ptr = _RHS_ptr();
        double* dlam_ptr = _dlam_ptr();
        double* C_mem_ptr = _C_mem_ptr();

        // evaluate constraints and their gradients
        _evaluateConstraintsAndGradients(C_ptr, delC_ptr, C_mem_ptr);

        // populate M^-1 and alpha_tilde
        MInv(M_inv_ptr);
        alphaTilde(alpha_tilde_ptr);

        // evaluate LHS and RHS
        _LHS(delC_ptr, M_inv_ptr, alpha_tilde_ptr, lhs_ptr);
        _RHS(C_ptr, delC_ptr, alpha_tilde_ptr, rhs_ptr);

        // std::cout << "rhs: " << _RHS_ptr()[0] << "\tlhs: " << _LHS_ptr()[0] << std::endl;
        // std::cout << "delC: " << _delC()[0] << ", " << _delC()[1] << ", " << _delC()[2] << ", " << _delC()[3] << ", " << _delC()[4] << ", " <<
        //  _delC()[5] << ", " << _delC()[6] << ", " << _delC()[7] << ", " << _delC()[8] << ", " << _delC()[9] << ", " << _delC()[10] << ", " << _delC()[11] << std::endl;

        // solve the system for dlam
        if (numConstraints() == 1)
        {
            // if system is 1x1, we can simply divide RHS by LHS to find dlam
            dlam_ptr[0] = rhs_ptr[0] / lhs_ptr[0];
        }
        else if (numConstraints() == 2)
        {
            // use an analytical system solve if it is 2x2 - this should be faster than using Eigen
            const double det = lhs_ptr[0]*lhs_ptr[3] - lhs_ptr[1]*lhs_ptr[2];

            dlam_ptr[0] = (rhs_ptr[0]*lhs_ptr[3] - rhs_ptr[1]*lhs_ptr[2]) / det;
            dlam_ptr[1] = (rhs_ptr[1]*lhs_ptr[0] - rhs_ptr[0]*lhs_ptr[1]) / det;
        }
        else
        {
            // if system is larger than 2x2, use Eigen direct solver - probably slow
            Eigen::Map<Eigen::MatrixXd> lhs_mat(lhs_ptr, numConstraints(), numConstraints());
            Eigen::Map<Eigen::VectorXd> dlam_vec(dlam_ptr, numConstraints());
            Eigen::Map<Eigen::VectorXd> rhs_vec(rhs_ptr, numConstraints());

            dlam_vec = lhs_mat.ldlt().solve(rhs_vec);
        }

        // update the lambdas with the recently calculate dlam vector
        for (unsigned i = 0; i < numConstraints(); i++)
        {
            setLambda(i, _lambda[i]+dlam_ptr[i]);
        }
        

        // calculate position updates based on computed quantities
        for (unsigned i = 0; i < numPositions(); i++)
        {
           _getPositionUpdate(i, delC_ptr, M_inv_ptr[i], dlam_ptr, coordinate_updates_ptr+3*i);
        }
    }

    /** Evaluates the constraint equation (Equation (9) in the XPBD paper).
     * This is the same as "h" and the constraint residual.
     * Uses a pre-allocated data block to perform calculations to avoid incurring data allocation costs for intermediate values.
     * The pre-allocated data block has the same structure as in the project() method.
     * 
     * @param data_ptr - a pointer to a block of pre-allocated data, assumed to be at least as large as the size given by memoryNeeded().
     * @param result (OUTPUT) - a pointer to the first element of the result vector that will store the constraint residual vector. Expects it to be numConstraints x 1.
     */
    inline virtual void constraintEquation(double* data_ptr, double* result)
    {
        // point the data member variable to point to the pre-allocated data block
        _data = data_ptr;

        double* C_ptr = _C_ptr();
        double* delC_ptr = _delC_ptr();
        double* alpha_tilde_ptr = _alpha_tilde_ptr();
        double* C_mem_ptr = _C_mem_ptr();

        // evaluate constraints and their gradients
        _evaluateConstraintsAndGradients(C_ptr, delC_ptr, C_mem_ptr);

        alphaTilde(alpha_tilde_ptr);

        _RHS(C_ptr, delC_ptr, alpha_tilde_ptr, result);
    }

    /** The number of constraints projected simultaneously by this ConstraintProjector. */
    inline unsigned numConstraints() const { return _constraints.size(); }

    /** The number of positions (vertices) affected by the constraints. */
    inline unsigned numPositions() const { return _positions.size(); }

    /** The number of coordinates (i.e. x1,y1,z1, x2,y2,z2, etc.) affected by the constraints = 3 times the number of positions. */
    inline unsigned numCoordinates() const { return numPositions() * 3; }

    /** Amount of space in bytes required to perform the computation. */
    inline unsigned memoryNeeded() const
    {
        unsigned num_bytes = 0;
        num_bytes += numCoordinates() * numConstraints() * sizeof(double); // size of the gradient matrix
        num_bytes += numConstraints() * sizeof(double); // size of constraint vector
        num_bytes += numPositions() * sizeof(double); // size needed for M^-1
        num_bytes += numConstraints() * sizeof(double); // size needed for alpha_tilde
        num_bytes += numConstraints() * numConstraints() * sizeof(double); // size needed for LHS matrix
        num_bytes += numConstraints()* sizeof(double); // size needed for RHS vector
        num_bytes += numConstraints()* sizeof(double); // size needed for dlam vector

        // find max amount of additional memory needed by constraints for doing calculations
        unsigned max_constraint_mem = 0;
        for (const auto& c : _constraints)
        {
            if (c->memoryNeeded() > max_constraint_mem)
            {
                max_constraint_mem = c->memoryNeeded();
            }
        }
        num_bytes += max_constraint_mem;

        return num_bytes;
    }

    /** Returns the vector of constraints that are projected by this ConstraintProjector. */
    inline const std::vector<Constraint*>& constraints() const { return _constraints; }

    /** Returns the vector of positions that are affected by the constraints projected by this ConstraintProjector. */
    inline const std::vector<PositionReference>& positions() const { return _positions; }

    /** Returns the vector of the Lagrange multipliers associated with the constraints projected by this ConstraintProjector. */
    inline const std::vector<double>& lambda() const { return _lambda; }

    /** Computes the alpha tilde matrix.
     * @param alpha_tilde_ptr (OUTPUT) - the pointer to the (currently empty) alpha tilde "matrix". The diagonal matrix is represented as a vector that is numConstraints x 1.
     */
    virtual void alphaTilde(double* alpha_tilde_ptr) const
    {
        for (unsigned i = 0; i < numConstraints(); i++)
        {
            alpha_tilde_ptr[i] = _constraints[i]->alpha() / (_dt * _dt);
        }
    }

    /** Computes the inverse mass matrix (i.e. M^-1).
     * @param inv_M_ptr (OUTPUT) - the pointer to the (currently empty) mass "matrix". The diagonal matrix is represented as a vector that is numPositions x 1. 
     */
    void MInv(double* inv_M_ptr) const
    {
        for (unsigned i = 0; i < numPositions(); i++)
        {
            inv_M_ptr[i] = _positions[i].inv_mass;
        }
    }

    /** Sets the Lagrange multiplier for the constraint at the specified index. This is usually done to initialize the multipliers or to apply the lambda update.
     * @param index - the constraint index
     * @param val - the new Lagrange multiplier
     */
    virtual void setLambda(const unsigned index, const double val) { _lambda[index] = val; }

    /** Whether or not this constraint needs the primary residual to do its constraint projection.
     * By default, this is false, but can be overridden by derived classes to be true if a constraint needs the primary residual.
     * 
     * The Solver class will query each constraint to see if it needs to compute the primary residual is needs to be calculated before each GS iteration.
     */
    virtual bool usesPrimaryResidual() const { return false; }

    /** Whether or not this constraint uses damping in its constraint projection.
     * By default, this is false, but can be overridden by derived classes to be true if the constraint uses damping.
     */
    virtual bool usesDamping() const { return false; }

    protected:

    /** Evaluates the constraints and their gradients. This may be overloaded by derived classes if there is a more efficient way than the below naive approach. 
     * @param C_ptr - the pointer to the constraint vector. Expects it to be numConstraints x 1.
     * @param delC_ptr - the pointer to the delC matrix. Expects it to be row-major and numConstraints x numCoordinates.
     * @param C_mem_ptr - the pointer to additional memory for the constraints to store intermediate calculations 
    */
    inline virtual void _evaluateConstraintsAndGradients(double* C_ptr, double* delC_ptr, double* C_mem_ptr)
    {
        for (unsigned ci = 0; ci < _constraints.size(); ci++)
        {
            // C(x) is a vector: C_ptr+ci points to the index ci in the C(x) vector
            // delC(x) is a matrix: delC_ptr + ci*numCoordinates() points to the row corresponding to the ci'th constraint
            _constraints[ci]->evaluateWithGradient(C_ptr+ci, delC_ptr+ci*numCoordinates(), C_mem_ptr);
        }
    }

    /** Computes the LHS matrix (i.e. delC * M^-1 * delC^T + alpha_tilde). Expects that delC, M^-1, and alpha_tilde have all been computed already.
     * @param delC_ptr - the pointer to the delC matrix. Expects it to be row-major and numConstraints x numCoordinates.
     * @param M_inv_ptr - the pointer to the M^-1 "matrix". Expects it to be a vector that is numPositions x 1.
     * @param alpha_tilde_ptr - the pointer to the alpha_tilde "matrix". Expects it to be a vector and numConstraints x 1.
     * @param lhs_ptr (OUTPUT) - the pointer to the (currently empty) LHS matrix. Expects it to be column-major and numConstraints x numConstraints.
     */
    inline virtual void _LHS(const double* delC_ptr, const double* M_inv_ptr, const double* alpha_tilde_ptr, double* lhs_ptr)
    {
        // set LHS matrix to 0 with alpha_tilde along the diagonal
        for (unsigned ci = 0; ci < numConstraints(); ci++)
        {
            for (unsigned cj = 0; cj < numConstraints(); cj++)
            {
                // if ci == cj, this is the diagonal, so put alpha_tilde there
                if (ci == cj)
                {
                    lhs_ptr[cj*numConstraints() + ci] = alpha_tilde_ptr[ci];
                }
                // otherwise 0
                else
                {
                    lhs_ptr[cj*numConstraints() + ci] = 0;
                }
            }
        }

        // add up contributions from delC*M^-1*delC^T
        for (unsigned ci = 0; ci < numConstraints(); ci++)
        {
            const double* delC_i = delC_ptr + numCoordinates()*ci;        // pointer to the delC vector of the ith constraint (1 x numCoordinates)
            for (unsigned cj = ci; cj < numConstraints(); cj++)     // delC*M^-1*delC^T is symmetric, so we only need to iterate the upper triangle
            {
                const double* delC_j = delC_ptr + numCoordinates()*cj;    // pointer to the delC vector of the jth constraint (1 x numCoordinates)
                for (unsigned pi = 0; pi < numPositions(); pi++)    
                {
                    const double inv_m = M_inv_ptr[pi];
                    // add the contribution for each position (inv_m times the dot product of the constraint gradient vectors)
                    lhs_ptr[cj*numConstraints() + ci] += inv_m * (delC_i[3*pi]*delC_j[3*pi] + delC_i[3*pi+1]*delC_j[3*pi+1] + delC_i[3*pi+2]*delC_j[3*pi+2]);
                }

                // take advantage of symmetry
                lhs_ptr[ci*numConstraints() + cj] = lhs_ptr[cj*numConstraints() + ci];
            }
        }
    }

    /** Computes the RHS vector (i.e. -C - alpha_tilde * lambda). Expects that C and alpha_tilde have all been computed already.
     * @param C_ptr - the pointer to the C vector. Expects it to be numConstraints x 1.
     * @param delC_ptr - the pointer to the delC matrix. Expects it to be row-major and numConstraints x numCoordinates.
     * @param alpha_tilde_ptr - the pointer to the alpha_tilde "matrix". Expects it to be a vector and numConstraints x 1.
     * @param rhs_ptr (OUTPUT) - the pointer to the (currently empty) RHS vector. Expects it to be numConstraints x 1.
     */
    inline virtual void _RHS(const double* C_ptr, const double* delC_ptr, const double* alpha_tilde_ptr, double* rhs_ptr)
    {
        for (unsigned ci = 0; ci < numConstraints(); ci++)
        {
            // RHS vector is simply -C - alpha_tilde * lambda
            rhs_ptr[ci] = -C_ptr[ci] - alpha_tilde_ptr[ci] * _lambda[ci];
        }
    }


    /** Computes the position update (i.e. Delta x = M^-1 * delC^T * Delta lambda) for the given position index. Expects that Delta lambda, delC, and M^-1 have been computed already.
     * @param position_index - the index of the position to calculate the update for in the _positions member vector.
     * @param delC_ptr - the pointer to the delC matrix. Expects it to be row-major and numConstraints x numCoordinates.
     * @param inv_m - the inverse mass associated with this position.
     * @param dlam_ptr - the pointer to the Delta lambda vector. Expects it to be numConstraints x 1.
     * @param pos_update_ptr (OUTPUT) - the pointer to the (currently empty) position update vector. Expects it to be a 3-vector.
     */
    inline virtual void _getPositionUpdate(const unsigned position_index, const double* delC_ptr, const double inv_m, const double* dlam_ptr, double* pos_update_ptr) const
    {
        pos_update_ptr[0] = 0;
        pos_update_ptr[1] = 0;
        pos_update_ptr[2] = 0;
        for (unsigned ci = 0; ci < numConstraints(); ci++)
        {
            pos_update_ptr[0] += inv_m * delC_ptr[ci*numCoordinates() + 3*position_index]   * dlam_ptr[ci];
            pos_update_ptr[1] += inv_m * delC_ptr[ci*numCoordinates() + 3*position_index+1] * dlam_ptr[ci];
            pos_update_ptr[2] += inv_m * delC_ptr[ci*numCoordinates() + 3*position_index+2] * dlam_ptr[ci];
        }
        
    }

    protected:

    /** Helper functions that subdivide the allocated data block into its corresponding parts. */
    /** Returns a pointer to the start of the constraint evaluations (i.e. the C(x) vector).
     * This vector has size numConstraints (i.e. a single Real for each constraint)
     */
    inline double* _C_ptr() const { return _data; }

    /** Returns a pointer to the start of the constraint gradient matrix (i.e. delC(x) matrix).
     * The constraint gradient is represented in memory as a flat vector - the first numCooordinates values are the gradient of the first constraint,
     * the second numCoordinates values are the gradient of the second constraint, and so on.
     * 
     * The gradient matrix has size numConstraints x numCoordinates.
     */
    inline double* _delC_ptr() const { return _C_ptr() + numConstraints(); }

    /** Returns a pointer to the start of the inverse mass matrix (i.e. M^-1 matrix).
     * The inverse mass matrix is diagonal and has duplicate information so it is represented by a vector that is numPositions long
     *  (i.e. each position has a unique mass)
     */
    inline double* _M_inv_ptr() const { return _delC_ptr() + numConstraints()*numCoordinates(); }

    /** Returns a pointer to the start of the alpha_tilde matrix.
     * The alpha_tilde matrix is diagonal so it is represented by a vector that is numConstraints long.
     */
    inline double* _alpha_tilde_ptr() const { return _M_inv_ptr() + numPositions(); }

    /** Returns a pointer to the start of the LHS matrix (i.e. delC * M^-1 * delC + alpha_tilde).
     * The LHS matrix has size numConstraints x numConstraints.
     */
    inline double* _LHS_ptr() const { return _alpha_tilde_ptr() + numConstraints(); }

    /** Returns a pointer to the start of the RHS vector (i.e. -C - alpha_tilde * lambda).
     * The RHS matrix has length of numConstraints.
     */
    inline double* _RHS_ptr() const { return _LHS_ptr() + numConstraints() * numConstraints(); }

    /** Retruns a pointer to the start of the delta_lambda vector.
     * The delta_lambda vector has length numConstraints.
     */
    inline double* _dlam_ptr() const { return _RHS_ptr() + numConstraints(); }

    /** Returns a pointer to the start of the additional scratch memory used by the Constraint class to perform calculations. */
    inline double* _C_mem_ptr() const { return _dlam_ptr() + numConstraints(); }

    protected:
    double _dt;         // the size of the time step used during constraint projection
    std::vector<double> _lambda;            // Lagrange multipliers for this constraint
    std::vector<Constraint*> _constraints;  // constraint(s) to be projected simultaneously
    std::vector<PositionReference> _positions; // position(s) associated with the constraints

    /* a generic data pointer with space to perform computations efficiently
        - trying to avoid dynamically allocating data inside the loop
    */
    double* _data;
};


} // namespace Solver

#endif // __CONSTRAINT_PROJECTOR_HPP