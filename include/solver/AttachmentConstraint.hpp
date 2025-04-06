#ifndef __ATTACHMENT_CONSTRAINT_HPP
#define __ATTACHMENT_CONSTRAINT_HPP

#include "solver/Constraint.hpp"

namespace Solver
{

class AttachmentConstraint : public Constraint
{
    public:
    explicit AttachmentConstraint(const Sim::XPBDMeshObject* obj, int v_ind, const Eigen::Vector3d* attached_pos_ptr, const Eigen::Vector3d& attachment_offset)
        : Constraint(std::vector<PositionReference>({
            PositionReference(obj, v_ind)
        })), _attached_pos_ptr(attached_pos_ptr), _attachment_offset(attachment_offset)
    {

    }

    /** Evaluates the current value of this constraint.
     * i.e. returns C(x)
     */
    inline double evaluate() const override
    {
        assert(0);
        return 0;
    }

    /** Returns the gradient of this constraint in vector form.
     * i.e. returns delC(x)
     */
    inline Eigen::VectorXd gradient() const override
    {
        assert(0);
        return Eigen::VectorXd::Zero(4);
        
    }

    /** Returns the value and gradient of this constraint.
     * i.e. returns C(x) and delC(x) together.
     * 
     * This may be desirable when there would be duplicate work involved to evaluate constraint and its gradient separately.
     */
    inline Constraint::ValueAndGradient evaluateWithGradient() const override
    {
        assert(0);
        return ValueAndGradient();
    }


    /** Evaluates the current value of this constraint with pre-allocated memory.
     * i.e. returns C(x)
     * 
     * @param C (OUTPUT) - the pointer to the (currently empty) value of the constraint
     */
    inline void evaluate(double* C) const override
    {
        *C = ( Eigen::Map<Eigen::Vector3d>(_positions[0].position_ptr) - (*_attached_pos_ptr + _attachment_offset) ).norm();
    }

    /** Computes the gradient of this constraint in vector form with pre-allocated memory.
     * i.e. returns delC(x)
     * 
     * @param grad (OUTPUT) - the pointer to the (currently empty) constraint gradient vector. Expects it to be _gradient_vector_size x 1.
     */
    inline void gradient(double* grad) const override
    {
        const Eigen::Vector3d& attach_pt = (*_attached_pos_ptr + _attachment_offset);
        const double dist = ( Eigen::Map<Eigen::Vector3d>(_positions[0].position_ptr) - attach_pt ).norm();
        grad[0] = (_positions[0].position_ptr[0] - attach_pt[0]) / dist;
        grad[1] = (_positions[0].position_ptr[1] - attach_pt[1]) / dist;
        grad[2] = (_positions[0].position_ptr[2] - attach_pt[2]) / dist;
    }


    /** Computes the value and gradient of this constraint with pre-allocated memory.
     * i.e. returns C(x) and delC(x) together.
     * 
     * This may be desirable when there would be duplicate work involved to evaluate constraint and its gradient separately.
     * 
     * @param C (OUTPUT) - the pointer to the (currently empty) value of the constraint
     * @param grad (OUTPUT) - the pointer to the (currently empty) constraint gradient vector. Expects it to be _gradient_vector_size x 1.
     */
    void evaluateWithGradient(double* C, double* grad) const override
    {
        const Eigen::Vector3d& attach_pt = (*_attached_pos_ptr + _attachment_offset);
        const double dist = ( Eigen::Map<Eigen::Vector3d>(_positions[0].position_ptr) - attach_pt ).norm();
        *C = dist;

        grad[0] = (_positions[0].position_ptr[0] - attach_pt[0]) / dist;
        grad[1] = (_positions[0].position_ptr[1] - attach_pt[1]) / dist;
        grad[2] = (_positions[0].position_ptr[2] - attach_pt[2]) / dist;
    }

    private:
    const Eigen::Vector3d* _attached_pos_ptr;
    const Eigen::Vector3d _attachment_offset;

};

} // namespace Solver

#endif // __ATTACHMENT_CONSTRAINT_HPP