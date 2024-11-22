// #include "solver/PrimaryResidualElementConstraint.hpp"

// namespace Solver
// {

// PrimaryResidualElementConstraint::PrimaryResidualElementConstraint(const double dt, ElasticMeshObject* obj, unsigned v1, unsigned v2, unsigned v3, unsigned v4)
//     : Constraint(dt),
//       PrimaryResidualConstraint(dt),
//       ElementConstraint(dt, obj, v1, v2, v3, v4)
// {

// }

// std::vector<Constraint::PositionUpdate> PrimaryResidualElementConstraint::project()
// {
//     const double alpha_tilde = _alpha / (_dt*_dt);

//     // evaluate the constraint and its gradient
//     const ValueAndGradient val_and_grad = evaluateWithGradient();
//     const double C = val_and_grad.first;
//     const Eigen::VectorXd grads = val_and_grad.second;
    
//     // get masses
//     const double inv_m0 = 1.0/_positions[0].mass();
//     const double inv_m1 = 1.0/_positions[1].mass();
//     const double inv_m2 = 1.0/_positions[2].mass();
//     const double inv_m3 = 1.0/_positions[3].mass();

//     // get the primary residual for each tetrahedral position
//     assert(_res_ptr);
//     const Eigen::Vector3d g0 = (*_res_ptr)(Eigen::seq(3*_positions[0].index,3*_positions[0].index+2));
//     const Eigen::Vector3d g1 = (*_res_ptr)(Eigen::seq(3*_positions[0].index,3*_positions[0].index+2));
//     const Eigen::Vector3d g2 = (*_res_ptr)(Eigen::seq(3*_positions[0].index,3*_positions[0].index+2));
//     const Eigen::Vector3d g3 = (*_res_ptr)(Eigen::seq(3*_positions[0].index,3*_positions[0].index+2));

//     // scale the primary residual
//     const Eigen::Vector3d scaled_g0 = g0 / (2*_positions[0].attachedElements());
//     const Eigen::Vector3d scaled_g1 = g1 / (2*_positions[1].attachedElements());
//     const Eigen::Vector3d scaled_g2 = g2 / (2*_positions[2].attachedElements());
//     const Eigen::Vector3d scaled_g3 = g3 / (2*_positions[3].attachedElements());

//     // compute dlam
//     const double dlam = (-C - alpha_tilde * _lambda + 
//         inv_m0 * grads(Eigen::seq(0,2)).dot(scaled_g0) +
//         inv_m1 * grads(Eigen::seq(3,5)).dot(scaled_g1) +
//         inv_m2 * grads(Eigen::seq(6,8)).dot(scaled_g2) +
//         inv_m3 * grads(Eigen::seq(9,11)).dot(scaled_g3)) / 
//         (inv_m0 * grads(Eigen::seq(0,2)).squaredNorm() +
//          inv_m1 * grads(Eigen::seq(3,5)).squaredNorm() +
//          inv_m2 * grads(Eigen::seq(6,8)).squaredNorm() +
//          inv_m3 * grads(Eigen::seq(9,11)).squaredNorm() + alpha_tilde);
    
//     // update lambda
//     _lambda += dlam;

//     // compute position updates for the computed dlam
//     std::vector<PositionUpdate> position_updates(4);
//     position_updates[0].first = _positions[0];
//     position_updates[1].first = _positions[1];
//     position_updates[2].first = _positions[2];
//     position_updates[3].first = _positions[3];
//     position_updates[0].second = (grads(Eigen::seq(0,2)) * dlam - scaled_g0) * inv_m0;
//     position_updates[1].second = (grads(Eigen::seq(3,5)) * dlam - scaled_g1) * inv_m1;
//     position_updates[2].second = (grads(Eigen::seq(6,8)) * dlam - scaled_g2) * inv_m2;
//     position_updates[3].second = (grads(Eigen::seq(9,11)) * dlam - scaled_g3) * inv_m3;

//     return position_updates;
// }

// } // namespace Solver