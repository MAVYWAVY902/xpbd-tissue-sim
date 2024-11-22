// #ifndef __PRIMARY_RESIDUAL_CONSTRAINT_HPP
// #define __PRIMARY_RESIDUAL_CONSTRAINT_HPP

// #include "solver/Constraint.hpp"

// namespace Solver
// {

// class PrimaryResidualConstraint : public virtual Constraint
// {
//     public:
//     explicit PrimaryResidualConstraint(const double dt)
//         : Constraint(dt)
//     {}

//     void setPrimaryResidual(std::shared_ptr<Eigen::VectorXd> res_ptr) { _res_ptr = res_ptr; }
    
//     bool needsPrimaryResidual() const override { return true; }

//     protected:
//     std::shared_ptr<Eigen::VectorXd> _res_ptr;
    
// };

// } // namespace Solver

// #endif // __PRIMARY_RESIDUAL_CONSTRAINT_HPP