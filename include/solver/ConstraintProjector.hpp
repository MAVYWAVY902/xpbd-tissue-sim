#ifndef __CONSTRAINT_PROJECTOR_HPP
#define __CONSTRAINT_PROJECTOR_HPP

#include "solver/Constraint.hpp"

namespace Solver
{

class ConstraintProjector
{
    friend class ConstraintDecorator;
    public:
    typedef std::pair<PositionReference, Eigen::Vector3d> PositionUpdate;
    typedef std::pair<Eigen::VectorXd, Eigen::MatrixXd> VectorValueAndGradient;

    explicit ConstraintProjector(std::vector<Constraint*> constraints, const double dt)
        : _constraints(std::move(constraints)), _dt(dt)
    {
        // iterate through constraints and assemble vector of position references
        for (const auto& c : _constraints)
        {
            for (const auto& pref : c->positions())
            {
                // check if position is not accounted for yet
                if (std::find(_positions.begin(), _positions.end(), pref) == _positions.end())
                {
                    _positions.push_back(pref);
                }
            }
        }

        // update gradient vector sizes for each constraint
        for (const auto& c : _constraints)
        {
            c->setGradientVectorSize(_positions.size()*3);
            for (unsigned i = 0; i < c->positions().size(); i++)
            {
                const PositionReference& pref_i = c->positions()[i];
                // get index of position in the combined _positions vector
                unsigned p_index = std::distance(_positions.begin(), std::find(_positions.begin(), _positions.end(), pref_i));
                // map the ith position in the constraint to the correct position in the constraint gradient vector
                c->setGradientVectorPosition(i, p_index*3);
            }
        }
    }

    inline virtual void initialize()
    {
        setLambda(Eigen::VectorXd::Zero(numConstraints()));
    }

    inline virtual std::vector<PositionUpdate> project()
    {
        const VectorValueAndGradient val_and_grad = _evaluateWithGradient();
        const Eigen::VectorXd rhs = _RHS(val_and_grad);
        const Eigen::MatrixXd lhs = _LHS(val_and_grad);
        const Eigen::VectorXd dlam = lhs.ldlt().solve(rhs); 

        // std::cout << "rhs: " << rhs << std::endl;
        // std::cout << "lhs: " << lhs << std::endl;
        // std::cout << "dlam: " << dlam << std::endl;
        

        std::vector<PositionUpdate> position_updates(_positions.size());
        for (unsigned i = 0; i < _positions.size(); i++)
        {
            position_updates.at(i) = std::move(_getPositionUpdate(i, dlam, val_and_grad.second));
        }

        setLambda(lambda() + dlam);

        return position_updates;
    }

    inline unsigned numConstraints() const { return _constraints.size(); }

    inline const std::vector<Constraint*>& constraints() const { return _constraints; }

    inline const std::vector<PositionReference>& positions() const { return _positions; }

    /** Returns the value of the Lagrange multiplier associated with this constraint. */
    inline Eigen::VectorXd lambda() const { return _lambda; }

    /** Returns the alpha tilde for this constraint. */
    virtual Eigen::MatrixXd alphaTilde() const
    {
        Eigen::MatrixXd mat(numConstraints(), numConstraints());
        for (unsigned i = 0; i < numConstraints(); i++)
        {
            mat(i,i) = _constraints[i]->alpha() / (_dt * _dt);
        }

        return mat;
    }

    inline Eigen::MatrixXd Minv() const
    {
        Eigen::MatrixXd mat(3*_positions.size(), 3*_positions.size());
        for (unsigned i = 0; i < _positions.size(); i++)
        {
            const double inv_m = positionInvMass(i);
            mat(i,i) = inv_m;
            mat(i+1,i+1) = inv_m;
            mat(i+2,i+2) = inv_m;
        }
        return mat;
    }

    virtual void setLambda(const Eigen::VectorXd& new_lambda) { _lambda = new_lambda; }

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

    /** Returns the inverse mass for the position at the specified index. */
    virtual double positionInvMass(const unsigned position_index) const { return _positions[position_index].invMass(); }

    protected:
    inline virtual Eigen::MatrixXd _LHS(const VectorValueAndGradient& val_and_grad) const
    {
        const Eigen::MatrixXd& alpha_tilde = alphaTilde();
        const Eigen::MatrixXd& m_inv = Minv();
        const Eigen::MatrixXd& delC = val_and_grad.second;

        // std::cout << "delC: " << delC.rows() << "x" << delC.cols() << std::endl;
        // std::cout << "minv: " << m_inv.rows() << "x" << m_inv.cols() << std::endl;
        // std::cout << "alpha_tilde: " << alpha_tilde.rows() << "x" << alpha_tilde.cols() << std::endl;

        return delC.transpose() * m_inv * delC + alpha_tilde;
    }

    inline virtual Eigen::VectorXd _RHS(const VectorValueAndGradient& val_and_grad) const
    {
        const Eigen::MatrixXd alpha_tilde = alphaTilde();
        const Eigen::VectorXd C = val_and_grad.first;

        // std::cout << alpha_tilde << std::endl;
        // std::cout << C << std::endl;
        // std::cout << _lambda << std::endl;
        // const Eigen::VectorXd rhs = -C - alpha_tilde * _lambda;

        return -C - alpha_tilde * _lambda;
    }

    inline virtual PositionUpdate _getPositionUpdate(const unsigned position_index, const Eigen::VectorXd& dlam, const Eigen::MatrixXd& grads) const
    {
        PositionUpdate position_update;
        position_update.first = _positions[position_index];
        position_update.second = positionInvMass(position_index) * (grads(Eigen::seq(3*position_index, 3*position_index+2), Eigen::placeholders::all) * dlam);
        return position_update;
    }

    private:
    inline VectorValueAndGradient _evaluateWithGradient() const
    {
        Eigen::VectorXd C(numConstraints());
        Eigen::MatrixXd delC(3*_positions.size(), numConstraints());
        for (unsigned ci = 0; ci < numConstraints(); ci++)
        {
            const Constraint::ValueAndGradient& val_and_grad = _constraints[ci]->evaluateWithGradient();
            C(ci) = val_and_grad.first;
            delC.col(ci) = val_and_grad.second; // copy happening here
        }

        return VectorValueAndGradient(C, delC);
    }

    protected:
    double _dt;         // the size of the time step used during constraint projection
    Eigen::VectorXd _lambda;     // Lagrange multiplier for this constraint
    std::vector<Constraint*> _constraints;  // constraint(s) to be projected simultaneously
    std::vector<PositionReference> _positions; // position(s) associated with the constraints
    std::map<std::shared_ptr<PositionReference>, unsigned> _pref_to_pindex;  // maps constraint position references to associated indeices in _positions

};


} // namespace Solver

#endif // __CONSTRAINT_PROJECTOR_HPP