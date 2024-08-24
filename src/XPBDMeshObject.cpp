#include "XPBDMeshObject.hpp"

#include <easy3d/renderer/renderer.h>
#include <easy3d/core/types.h>

#include <chrono>
#include <random>
#include <algorithm>

// XPBDMeshObject::XPBDMeshObject(const std::string& name, const YAML::Node& config)
//     : ElasticMeshObject(name, config)
// {
//     _init();
//     _precomputeQuantities();
// }
XPBDMeshObject::XPBDMeshObject(const XPBDMeshObjectConfig* config)
    : ElasticMeshObject(config)
{
    _num_iters = config->numSolverIters().value();
    
    _solve_mode = config->solveMode().value();

    _damping_stiffness = config->dampingStiffness().value();

    _init();
    _precomputeQuantities();
}

XPBDMeshObject::XPBDMeshObject(const std::string& name, const std::string& filename, const ElasticMaterial& material)
    : ElasticMeshObject(name, filename, material), _num_iters(1)
{
    _init();
    _precomputeQuantities();
}

XPBDMeshObject::XPBDMeshObject(const std::string& name, const VerticesMat& verts, const ElementsMat& elems, const ElasticMaterial& material)
    : ElasticMeshObject(name, verts, elems, material), _num_iters(1)
{
    _init();
    _precomputeQuantities();
}

void XPBDMeshObject::_init()
{
    _x_prev = _vertices;
}

void XPBDMeshObject::_precomputeQuantities()
{
    // reserve space for vectors
    // Q and volumes are per-element
    _Q.resize(_elements.rows());
    _vols.conservativeResize(_elements.rows());
    // masses are per-vertex
    _m = Eigen::VectorXd::Zero(_vertices.rows());

    for (unsigned i = 0; i < _elements.rows(); i++)
    {
        // compute Q for each element
        const Eigen::Vector3d& X1 = _vertices.row(_elements(i,0));
        const Eigen::Vector3d& X2 = _vertices.row(_elements(i,1));
        const Eigen::Vector3d& X3 = _vertices.row(_elements(i,2));
        const Eigen::Vector3d& X4 = _vertices.row(_elements(i,3));

        Eigen::Matrix3d X;
        X.col(0) = (X1 - X4);
        X.col(1) = (X2 - X4);
        X.col(2) = (X3 - X4);

        // add it to overall vector of Q's
        _Q.at(i) = X.inverse();

        // compute volume from X
        double vol = std::abs(X.determinant()/6);
        _vols(i) = vol;

        // compute mass of element
        double m_element = vol * _material.density();
        // add mass contribution of element to each of its vertices
        _m(_elements(i,0)) += m_element/4;
        _m(_elements(i,1)) += m_element/4;
        _m(_elements(i,2)) += m_element/4;
        _m(_elements(i,3)) += m_element/4;
    }

    if (_solve_mode == XPBDSolveMode::SEQUENTIAL_INIT_LAMBDA)
    {
        _initial_lambda_ds = Eigen::VectorXd::Zero(_elements.rows());
        _initial_lambda_hs = Eigen::VectorXd::Zero(_elements.rows());
        // calculate initial guesses for lambdas based on last constraint
        for (int i = 0; i < _elements.rows(); i++)
        {
            const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);
            Eigen::Matrix3d X;
            X.col(0) = _vertices.row(elem(0)) - _vertices.row(elem(3));
            X.col(1) = _vertices.row(elem(1)) - _vertices.row(elem(3));
            X.col(2) = _vertices.row(elem(2)) - _vertices.row(elem(3));

            // compute F
            Eigen::Matrix3d F = X * _Q[i];

            // compute constraint itself
            const double C_d = std::sqrt(F.col(0).squaredNorm() + F.col(1).squaredNorm() + F.col(2).squaredNorm());
            // compute the deviatoric alpha
            const double alpha_d = 1/(_material.mu() * _vols[i]);

            // compute constraint itself
            const double C_h = F.determinant() - (1 + _material.mu()/_material.lambda());
            // compute the hydrostatic alpha
            const double alpha_h = 1/(_material.lambda() * _vols[i]);

            _initial_lambda_ds(i) = -1.0 / alpha_d * C_d;
            _initial_lambda_hs(i) = -1.0 / alpha_h * C_h;
        }
    }
    if (_solve_mode == XPBDSolveMode::RUCKER_FULL)
    {
        _constraints_per_position.resize(_vertices.rows());
        for (int i = 0; i < _elements.rows(); i++)
        {
            const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);
            _constraints_per_position[elem(0)].push_back(std::make_pair(2*i, 0));
            _constraints_per_position[elem(0)].push_back(std::make_pair(2*i+1, 0));

            _constraints_per_position[elem(1)].push_back(std::make_pair(2*i, 1));
            _constraints_per_position[elem(1)].push_back(std::make_pair(2*i+1, 1));

            _constraints_per_position[elem(2)].push_back(std::make_pair(2*i, 2));
            _constraints_per_position[elem(2)].push_back(std::make_pair(2*i+1, 2));

            _constraints_per_position[elem(3)].push_back(std::make_pair(2*i, 3));
            _constraints_per_position[elem(3)].push_back(std::make_pair(2*i+1, 3));
        }
    }

    std::cout << "Precomputed quantities" << std::endl;

    // _calculateForces();
}

void XPBDMeshObject::update(const double dt, const double g_accel)
{
    auto t1 = std::chrono::steady_clock::now();
    _movePositionsIntertially(dt, g_accel);
    auto t2 = std::chrono::steady_clock::now();
    if (_solve_mode == XPBDSolveMode::SEQUENTIAL)
        _projectConstraintsSequential(dt);
    else if (_solve_mode == XPBDSolveMode::SIMULTANEOUS)
        _projectConstraintsSimultaneous(dt);
    else if (_solve_mode == XPBDSolveMode::CONSTANTX)
        _projectConstraintsConstantX(dt);
    else if (_solve_mode == XPBDSolveMode::SEQUENTIAL_RANDOMIZED)
        _projectConstraintsSequentialRandomized(dt);
    else if (_solve_mode == XPBDSolveMode::SEQUENTIAL_INIT_LAMBDA)
        _projectConstraintsSequentialInitLambda(dt);
    else if (_solve_mode == XPBDSolveMode::RUCKER_FULL)
        _projectConstraintsRuckerFull(dt);

    auto t3 = std::chrono::steady_clock::now();
    _projectCollisionConstraints();
    auto t4 = std::chrono::steady_clock::now();
    _updateVelocities(dt);
    auto t5 = std::chrono::steady_clock::now();

    // std::cout << "XPBDMeshObject::update =========" << std::endl;
    // std::cout << "\tmovePositionsInertially took " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << " us" << std::endl;
    // std::cout << "\tprojectConstraints took " << std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count() << " us" << std::endl;
    // std::cout << "\tprojectCollisionConstraints took " << std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count() << " us" << std::endl;
    // std::cout << "\tupdateVelocities took " << std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count() << " us" << std::endl;
}

double XPBDMeshObject::primaryResidual()
{
    return _primary_residual;
}

double XPBDMeshObject::constraintResidual()
{
    return _constraint_residual;
}

std::string XPBDMeshObject::solveMode()
{
    if (_solve_mode == XPBDSolveMode::SEQUENTIAL)
        return "Sequential";
    if (_solve_mode == XPBDSolveMode::SEQUENTIAL_RANDOMIZED)
        return "Sequential-Randomized";
    if (_solve_mode == XPBDSolveMode::SIMULTANEOUS)
        return "Simultaneous";
    if (_solve_mode == XPBDSolveMode::CONSTANTX)
        return "ConstantX";
    if (_solve_mode == XPBDSolveMode::SEQUENTIAL_INIT_LAMBDA)
        return "Sequential-Init-Lambda";
    
    return "";
}

void XPBDMeshObject::_movePositionsIntertially(const double dt, const double g_accel)
{
    // move vertices according to their velocity
    _vertices += dt*_v;
    // external forces (right now just gravity, which acts in -z direction)
    for (int i = 0; i < _vertices.rows(); i++)
    {
        _vertices(i,2) += -g_accel * dt * dt;
    }
    

}

void XPBDMeshObject::_projectConstraintsSequential(const double dt)
{
    Eigen::VectorXd lambda_hs = Eigen::VectorXd::Zero(_elements.rows());
    // accumulated deviatoric Lagrange multipliers
    Eigen::VectorXd lambda_ds = Eigen::VectorXd::Zero(_elements.rows());

    // dlams
    Eigen::VectorXd dlam_hs = Eigen::VectorXd::Zero(_elements.rows());
    Eigen::VectorXd dlam_ds = Eigen::VectorXd::Zero(_elements.rows());

    // store positions before constraints are projected
    // this is x-tilde, seen in XPBD eqn 8 - used for computing primary residual
    VerticesMat inertial_positions = _vertices;

    // define Eigen loop variables
    Eigen::Matrix3d X, F, F_cross, C_h_grads, C_d_grads;
    Eigen::Vector3d C_h_grad_4, C_d_grad_4;
    for (unsigned gi = 0; gi < _num_iters; gi++)
    {
        for (int i = 0; i < _elements.rows(); i++)
        {
            const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);
            // extract masses of each vertex in the current element
            const double m1 = _m[elem(0)];
            const double m2 = _m[elem(1)];
            const double m3 = _m[elem(2)];
            const double m4 = _m[elem(3)];

            /** DEVIATORIC CONSTRAINT */
            // create the deformed shape matrix from current deformed vertex positions
            X.col(0) = _vertices.row(elem(0)) - _vertices.row(elem(3));
            X.col(1) = _vertices.row(elem(1)) - _vertices.row(elem(3));
            X.col(2) = _vertices.row(elem(2)) - _vertices.row(elem(3));

            // compute F
            F = X * _Q[i];
            
            // compute constraint itself
            const double C_d = std::sqrt(F.col(0).squaredNorm() + F.col(1).squaredNorm() + F.col(2).squaredNorm());

            // compute constraint gradient
            C_d_grads = 1/C_d * (F * _Q[i].transpose());
            // compute the deviatoric alpha
            const double alpha_d = 1/(_material.mu() * _vols[i]);

            const double beta_tilde = _damping_stiffness * (dt*dt);
            const double gamma_d = alpha_d / (dt*dt) * beta_tilde / dt;

            // by definition, the gradient for vertex 4 in the element is the negative sum of other gradients
            C_d_grad_4 = -C_d_grads.col(0) - C_d_grads.col(1) - C_d_grads.col(2);
            
            // const double denom_d = ((1/m1)*C_d_grads.col(0).squaredNorm() + (1/m2)*C_d_grads.col(1).squaredNorm() + (1/m3)*C_d_grads.col(2).squaredNorm() + (1/m4)*C_d_grad_4.squaredNorm() + alpha_d/(dt*dt));
            const double delC_x_prev_d =  C_d_grads.col(0).dot(_vertices.row(elem(0)) - _x_prev.row(elem(0))) +
                                        C_d_grads.col(1).dot(_vertices.row(elem(1)) - _x_prev.row(elem(1))) +
                                        C_d_grads.col(2).dot(_vertices.row(elem(2)) - _x_prev.row(elem(2))) +
                                        C_d_grad_4.dot(_vertices.row(elem(3)) - _x_prev.row(elem(3)));

            const double dlam_d = (-C_d - alpha_d * lambda_ds(i) / (dt*dt) - gamma_d * delC_x_prev_d) /
                ((1 + gamma_d) * (
                    (1/m1)*C_d_grads.col(0).squaredNorm() + 
                    (1/m2)*C_d_grads.col(1).squaredNorm() + 
                    (1/m3)*C_d_grads.col(2).squaredNorm() + 
                    (1/m4)*C_d_grad_4.squaredNorm()
                 ) + alpha_d/(dt*dt)); 
                // std::cout << "C_h: " << C_h << "\talpha_h: " << alpha_h <<"\tdlam_h: " << dlam_h << std::endl;
                // std::cout << "C_d: " << C_d << "\talpha_d: " << alpha_d << "\tdenominator: " <<  denom_d << "\tdlam_d: " << dlam_d << std::endl;
            
            _vertices.row(elem(0)) += C_d_grads.col(0) * dlam_d/m1;
            _vertices.row(elem(1)) += C_d_grads.col(1) * dlam_d/m2;
            _vertices.row(elem(2)) += C_d_grads.col(2) * dlam_d/m3;
            _vertices.row(elem(3)) += C_d_grad_4 * dlam_d/m4;


            /** HYDROSTATIC CONSTRAINT */

            // create the deformed shape matrix from current deformed vertex positions
            X.col(0) = _vertices.row(elem(0)) - _vertices.row(elem(3));
            X.col(1) = _vertices.row(elem(1)) - _vertices.row(elem(3));
            X.col(2) = _vertices.row(elem(2)) - _vertices.row(elem(3));

            // compute F
            F = X * _Q[i];

            // compute constraint itself
            const double C_h = F.determinant() - (1 + _material.mu()/_material.lambda());

            // compute constraint gradient
            F_cross.col(0) = F.col(1).cross(F.col(2));
            F_cross.col(1) = F.col(2).cross(F.col(0));
            F_cross.col(2) = F.col(0).cross(F.col(1));
            C_h_grads = F_cross * _Q[i].transpose();
            // compute the hydrostatic alpha
            const double alpha_h = 1/(_material.lambda() * _vols[i]);
            
            const double gamma_h = alpha_h / (dt*dt) * beta_tilde / dt;


            C_h_grad_4 = -C_h_grads.col(0) - C_h_grads.col(1) - C_h_grads.col(2);

            const double delC_x_prev_h =  C_h_grads.col(0).dot(_vertices.row(elem(0)) - _x_prev.row(elem(0))) +
                                        C_h_grads.col(1).dot(_vertices.row(elem(1)) - _x_prev.row(elem(1))) +
                                        C_h_grads.col(2).dot(_vertices.row(elem(2)) - _x_prev.row(elem(2))) +
                                        C_h_grad_4.dot(_vertices.row(elem(3)) - _x_prev.row(elem(3)));

            const double dlam_h = (-C_h - alpha_h * lambda_hs(i) / (dt*dt) - gamma_h * delC_x_prev_h) / 
                ((1 + gamma_h) * (
                    (1/m1)*C_h_grads.col(0).squaredNorm() + 
                    (1/m2)*C_h_grads.col(1).squaredNorm() + 
                    (1/m3)*C_h_grads.col(2).squaredNorm() + 
                    (1/m4)*C_h_grad_4.squaredNorm()
                 ) + alpha_h/(dt*dt));

            _vertices.row(elem(0)) += C_h_grads.col(0) * dlam_h/m1;
            _vertices.row(elem(1)) += C_h_grads.col(1) * dlam_h/m2;
            _vertices.row(elem(2)) += C_h_grads.col(2) * dlam_h/m3;
            _vertices.row(elem(3)) += C_h_grad_4 * dlam_h/m4;

            // update Lagrange multipliers
            lambda_hs(i) += dlam_h;
            lambda_ds(i) += dlam_d;
        }
    }

    _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
}

void XPBDMeshObject::_projectConstraintsSequentialRandomized(const double dt)
{
    Eigen::VectorXd lambda_hs = Eigen::VectorXd::Zero(_elements.rows());
    // accumulated deviatoric Lagrange multipliers
    Eigen::VectorXd lambda_ds = Eigen::VectorXd::Zero(_elements.rows());

    // dlams
    Eigen::VectorXd dlam_hs = Eigen::VectorXd::Zero(_elements.rows());
    Eigen::VectorXd dlam_ds = Eigen::VectorXd::Zero(_elements.rows());

    // store positions before constraints are projected
    // this is x-tilde, seen in XPBD eqn 8 - used for computing primary residual
    VerticesMat inertial_positions = _vertices;

    // define Eigen loop variables
    Eigen::Matrix3d X, F, F_cross, C_h_grads, C_d_grads;
    Eigen::Vector3d C_h_grad_4, C_d_grad_4;

    // create vector of element indices to shuffle
    std::vector<int> element_indices(_elements.rows());
    std::iota(std::begin(element_indices), std::end(element_indices), 0);
    auto rd = std::random_device {};
    auto rng = std::default_random_engine {rd()};

    for (unsigned gi = 0; gi < _num_iters; gi++)
    {
        // shuffle the element indices
        std::shuffle(std::begin(element_indices), std::end(element_indices), rng);
        for (const auto& i : element_indices)
        {
            const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);
            // extract masses of each vertex in the current element
            const double m1 = _m[elem(0)];
            const double m2 = _m[elem(1)];
            const double m3 = _m[elem(2)];
            const double m4 = _m[elem(3)];

            /** DEVIATORIC CONSTRAINT */
            // create the deformed shape matrix from current deformed vertex positions
            X.col(0) = _vertices.row(elem(0)) - _vertices.row(elem(3));
            X.col(1) = _vertices.row(elem(1)) - _vertices.row(elem(3));
            X.col(2) = _vertices.row(elem(2)) - _vertices.row(elem(3));

            // compute F
            F = X * _Q[i];
            
            // compute constraint itself
            const double C_d = std::sqrt(F.col(0).squaredNorm() + F.col(1).squaredNorm() + F.col(2).squaredNorm());

            // compute constraint gradient
            C_d_grads = 1/C_d * (F * _Q[i].transpose());
            // compute the deviatoric alpha
            const double alpha_d = 1/(_material.mu() * _vols[i]);

            const double beta_tilde = _damping_stiffness * (dt*dt);
            const double gamma_d = alpha_d / (dt*dt) * beta_tilde / dt;

            // by definition, the gradient for vertex 4 in the element is the negative sum of other gradients
            C_d_grad_4 = -C_d_grads.col(0) - C_d_grads.col(1) - C_d_grads.col(2);
            
            // const double denom_d = ((1/m1)*C_d_grads.col(0).squaredNorm() + (1/m2)*C_d_grads.col(1).squaredNorm() + (1/m3)*C_d_grads.col(2).squaredNorm() + (1/m4)*C_d_grad_4.squaredNorm() + alpha_d/(dt*dt));
            const double delC_x_prev_d =  C_d_grads.col(0).dot(_vertices.row(elem(0)) - _x_prev.row(elem(0))) +
                                        C_d_grads.col(1).dot(_vertices.row(elem(1)) - _x_prev.row(elem(1))) +
                                        C_d_grads.col(2).dot(_vertices.row(elem(2)) - _x_prev.row(elem(2))) +
                                        C_d_grad_4.dot(_vertices.row(elem(3)) - _x_prev.row(elem(3)));

            const double dlam_d = (-C_d - alpha_d * lambda_ds(i) / (dt*dt) - gamma_d * delC_x_prev_d) /
                ((1 + gamma_d) * (
                    (1/m1)*C_d_grads.col(0).squaredNorm() + 
                    (1/m2)*C_d_grads.col(1).squaredNorm() + 
                    (1/m3)*C_d_grads.col(2).squaredNorm() + 
                    (1/m4)*C_d_grad_4.squaredNorm()
                 ) + alpha_d/(dt*dt)); 
                // std::cout << "C_h: " << C_h << "\talpha_h: " << alpha_h <<"\tdlam_h: " << dlam_h << std::endl;
                // std::cout << "C_d: " << C_d << "\talpha_d: " << alpha_d << "\tdenominator: " <<  denom_d << "\tdlam_d: " << dlam_d << std::endl;
            
            _vertices.row(elem(0)) += C_d_grads.col(0) * dlam_d/m1;
            _vertices.row(elem(1)) += C_d_grads.col(1) * dlam_d/m2;
            _vertices.row(elem(2)) += C_d_grads.col(2) * dlam_d/m3;
            _vertices.row(elem(3)) += C_d_grad_4 * dlam_d/m4;


            /** HYDROSTATIC CONSTRAINT */

            // create the deformed shape matrix from current deformed vertex positions
            X.col(0) = _vertices.row(elem(0)) - _vertices.row(elem(3));
            X.col(1) = _vertices.row(elem(1)) - _vertices.row(elem(3));
            X.col(2) = _vertices.row(elem(2)) - _vertices.row(elem(3));

            // compute F
            F = X * _Q[i];

            // compute constraint itself
            const double C_h = F.determinant() - (1 + _material.mu()/_material.lambda());

            // compute constraint gradient
            F_cross.col(0) = F.col(1).cross(F.col(2));
            F_cross.col(1) = F.col(2).cross(F.col(0));
            F_cross.col(2) = F.col(0).cross(F.col(1));
            C_h_grads = F_cross * _Q[i].transpose();
            // compute the hydrostatic alpha
            const double alpha_h = 1/(_material.lambda() * _vols[i]);
            
            const double gamma_h = alpha_h / (dt*dt) * beta_tilde / dt;


            C_h_grad_4 = -C_h_grads.col(0) - C_h_grads.col(1) - C_h_grads.col(2);

            const double delC_x_prev_h =  C_h_grads.col(0).dot(_vertices.row(elem(0)) - _x_prev.row(elem(0))) +
                                        C_h_grads.col(1).dot(_vertices.row(elem(1)) - _x_prev.row(elem(1))) +
                                        C_h_grads.col(2).dot(_vertices.row(elem(2)) - _x_prev.row(elem(2))) +
                                        C_h_grad_4.dot(_vertices.row(elem(3)) - _x_prev.row(elem(3)));

            const double dlam_h = (-C_h - alpha_h * lambda_hs(i) / (dt*dt) - gamma_h * delC_x_prev_h) / 
                ((1 + gamma_h) * (
                    (1/m1)*C_h_grads.col(0).squaredNorm() + 
                    (1/m2)*C_h_grads.col(1).squaredNorm() + 
                    (1/m3)*C_h_grads.col(2).squaredNorm() + 
                    (1/m4)*C_h_grad_4.squaredNorm()
                 ) + alpha_h/(dt*dt));

            _vertices.row(elem(0)) += C_h_grads.col(0) * dlam_h/m1;
            _vertices.row(elem(1)) += C_h_grads.col(1) * dlam_h/m2;
            _vertices.row(elem(2)) += C_h_grads.col(2) * dlam_h/m3;
            _vertices.row(elem(3)) += C_h_grad_4 * dlam_h/m4;

            // update Lagrange multipliers
            lambda_hs(i) += dlam_h;
            lambda_ds(i) += dlam_d;
        }
    }

    _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
}

void XPBDMeshObject::_projectConstraintsSimultaneous(const double dt)
{
    // accumulated hydrostatic Lagrange multipliers
    Eigen::VectorXd lambda_hs = Eigen::VectorXd::Zero(_elements.rows());
    // accumulated deviatoric Lagrange multipliers
    Eigen::VectorXd lambda_ds = Eigen::VectorXd::Zero(_elements.rows());

    // store positions before constraints are projected
    // this is x-tilde, seen in XPBD eqn 8 - used for computing primary residual
    VerticesMat inertial_positions = _vertices;

    // define Eigen loop variables
    Eigen::Matrix3d X, F, F_cross, C_h_grads, C_d_grads;
    Eigen::Vector3d C_h_grad_4, C_d_grad_4;
    for (unsigned gi = 0; gi < _num_iters; gi++)
    {
        for (int i = 0; i < _elements.rows(); i++)
        {
            const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);

            // create the deformed shape matrix from current deformed vertex positions
            X.col(0) = _vertices.row(elem(0)) - _vertices.row(elem(3));
            X.col(1) = _vertices.row(elem(1)) - _vertices.row(elem(3));
            X.col(2) = _vertices.row(elem(2)) - _vertices.row(elem(3));

            // extract masses of each vertex in the current element
            const double m1 = _m[elem(0)];
            const double m2 = _m[elem(1)];
            const double m3 = _m[elem(2)];
            const double m4 = _m[elem(3)];

            // compute F
            F = X * _Q[i];

            /** HYDROSTATIC CONSTRAINT */
            // compute constraint itself
            const double C_h = F.determinant() - (1 + _material.mu()/_material.lambda());

            // compute constraint gradient
            F_cross.col(0) = F.col(1).cross(F.col(2));
            F_cross.col(1) = F.col(2).cross(F.col(0));
            F_cross.col(2) = F.col(0).cross(F.col(1));
            C_h_grads = F_cross * _Q[i].transpose();
            // compute the hydrostatic alpha
            const double alpha_h = 1/(_material.lambda() * _vols[i]);

            /** DEVIATORIC CONSTRAINT */
            // compute constraint itself
            const double C_d = std::sqrt(F.col(0).squaredNorm() + F.col(1).squaredNorm() + F.col(2).squaredNorm());

            // compute constraint gradient
            C_d_grads = 1/C_d * (F * _Q[i].transpose());
            // compute the deviatoric alpha
            const double alpha_d = 1/(_material.mu() * _vols[i]);

            // by definition, the gradient for vertex 4 in the element is the negative sum of other gradients
            C_h_grad_4 = -C_h_grads.col(0) - C_h_grads.col(1) - C_h_grads.col(2);
            C_d_grad_4 = -C_d_grads.col(0) - C_d_grads.col(1) - C_d_grads.col(2);


            double dlam_d = 0;
            double dlam_h = 0; 

            /** SOLVE FOR DLAM SIMULTANEOUSLY */
            // set up 2x2 system with both constraints
            const double off_diagonal = 1/m1*C_h_grads.col(0).dot(C_d_grads.col(0)) +
                                        1/m2*C_h_grads.col(1).dot(C_d_grads.col(1)) +
                                        1/m3*C_h_grads.col(2).dot(C_d_grads.col(2)) +
                                        1/m4*C_h_grad_4.dot(C_d_grad_4);
            Eigen::Matrix2d A {  {1/m1*C_h_grads.col(0).squaredNorm() + 1/m2*C_h_grads.col(1).squaredNorm() + 1/m3*C_h_grads.col(2).squaredNorm() + 1/m4*C_h_grad_4.squaredNorm() + alpha_h/(dt*dt),
                                        off_diagonal},
                                        {off_diagonal,
                                        1/m1*C_d_grads.col(0).squaredNorm() + 1/m2*C_d_grads.col(1).squaredNorm() + 1/m3*C_d_grads.col(2).squaredNorm() + 1/m4*C_d_grad_4.squaredNorm() + alpha_d/(dt*dt)
            } };

            Eigen::Vector2d b {  -C_h - alpha_h * lambda_hs(i) / (dt*dt),
                                        -C_d - alpha_d * lambda_ds(i) / (dt*dt)    
                                    };

            // solve the system
            // TODO: can we use a faster system solve?
            Eigen::ColPivHouseholderQR<Eigen::Matrix2d> dec(A);
            const Eigen::Vector2d& dlambda = dec.solve(b);

            dlam_h = dlambda(0);
            dlam_d = dlambda(1);

            // update vertex positions
            _vertices.row(elem(0)) += C_h_grads.col(0) * dlam_h/m1;
            _vertices.row(elem(1)) += C_h_grads.col(1) * dlam_h/m2;
            _vertices.row(elem(2)) += C_h_grads.col(2) * dlam_h/m3;
            _vertices.row(elem(3)) += C_h_grad_4 * dlam_h/m4;

            _vertices.row(elem(0)) += C_d_grads.col(0) * dlam_d/m1;
            _vertices.row(elem(1)) += C_d_grads.col(1) * dlam_d/m2;
            _vertices.row(elem(2)) += C_d_grads.col(2) * dlam_d/m3;
            _vertices.row(elem(3)) += C_d_grad_4 * dlam_d/m4;
            

            // update Lagrange multipliers
            lambda_hs(i) += dlam_h;
            lambda_ds(i) += dlam_d;
        }
    }

    _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
    // _calculateForces();

}

void XPBDMeshObject::_projectConstraintsConstantX(const double dt)
{
    // accumulated hydrostatic Lagrange multipliers
    Eigen::VectorXd lambda_hs = Eigen::VectorXd::Zero(_elements.rows());
    // accumulated deviatoric Lagrange multipliers
    Eigen::VectorXd lambda_ds = Eigen::VectorXd::Zero(_elements.rows());

    Eigen::VectorXd alpha_hs(_elements.rows());
    Eigen::VectorXd alpha_ds(_elements.rows());

    Eigen::VectorXd dlam_hs(_elements.rows());
    Eigen::VectorXd dlam_ds(_elements.rows());

    // store positions before constraints are projected
    // this is x-tilde, seen in XPBD eqn 8 - used for computing primary residual
    VerticesMat inertial_positions = _vertices;

    VerticesMat position_updates = VerticesMat::Zero(_vertices.rows(), 3);

    Eigen::MatrixXd delC(_elements.rows()*2, 12);
    Eigen::VectorXd C(_elements.rows()*2);

    // define Eigen loop variables
    Eigen::Matrix3d X, F, F_cross, C_h_grads, C_d_grads;
    Eigen::Vector3d C_h_grad_4, C_d_grad_4;

    for (int i = 0; i < _elements.rows(); i++)
    {
        const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);

        // create the deformed shape matrix from current deformed vertex positions
        X.col(0) = inertial_positions.row(elem(0)) - inertial_positions.row(elem(3));
        X.col(1) = inertial_positions.row(elem(1)) - inertial_positions.row(elem(3));
        X.col(2) = inertial_positions.row(elem(2)) - inertial_positions.row(elem(3));

        // compute F
        F = X * _Q[i];

        /** HYDROSTATIC CONSTRAINT */
        // compute constraint itself
        const double C_h = F.determinant() - (1 + _material.mu()/_material.lambda());

        // compute constraint gradient
        F_cross.col(0) = F.col(1).cross(F.col(2));
        F_cross.col(1) = F.col(2).cross(F.col(0));
        F_cross.col(2) = F.col(0).cross(F.col(1));
        C_h_grads = F_cross * _Q[i].transpose();
        // compute the hydrostatic alpha
        const double alpha_h = 1/(_material.lambda() * _vols[i]);

        /** DEVIATORIC CONSTRAINT */
        // compute constraint itself
        const double C_d = std::sqrt(F.col(0).squaredNorm() + F.col(1).squaredNorm() + F.col(2).squaredNorm());

        // compute constraint gradient
        C_d_grads = 1/C_d * (F * _Q[i].transpose());
        // compute the deviatoric alpha
        const double alpha_d = 1/(_material.mu() * _vols[i]);

        // by definition, the gradient for vertex 4 in the element is the negative sum of other gradients
        C_h_grad_4 = -C_h_grads.col(0) - C_h_grads.col(1) - C_h_grads.col(2);
        C_d_grad_4 = -C_d_grads.col(0) - C_d_grads.col(1) - C_d_grads.col(2);

        // populate C
        C(2*i) = C_h;
        C(2*i+1) = C_d;


        // populate delC
        delC(2*i, Eigen::seq(0,2)) = C_h_grads.col(0);
        delC(2*i, Eigen::seq(3,5)) = C_h_grads.col(1);
        delC(2*i, Eigen::seq(6,8)) = C_h_grads.col(2);
        delC(2*i, Eigen::seq(9,11)) = C_h_grad_4;
        delC(2*i+1, Eigen::seq(0,2)) = C_d_grads.col(0);
        delC(2*i+1, Eigen::seq(3,5)) = C_d_grads.col(1);
        delC(2*i+1, Eigen::seq(6,8)) = C_d_grads.col(2);
        delC(2*i+1, Eigen::seq(9,11)) = C_d_grad_4;


        // populate alpha vectors
        alpha_hs(i) = alpha_h;
        alpha_ds(i) = alpha_d;
    }

    // iteratively solve dlams
    for (unsigned gi = 0; gi < _num_iters; gi++)
    {
        for (int i = 0; i < _elements.rows(); i++)
        {
            const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);
            
            // extract masses of each vertex in the current element
            const double m1 = _m[elem(0)];
            const double m2 = _m[elem(1)];
            const double m3 = _m[elem(2)];
            const double m4 = _m[elem(3)];

            const double C_h = C(2*i);
            const double C_d = C(2*i+1);

            const Eigen::VectorXd& C_h_grads = delC.row(2*i);
            const Eigen::VectorXd& C_d_grads = delC.row(2*i+1); 

            const double dlam_h = (-C_h - alpha_hs(i) * lambda_hs(i) / (dt*dt)) / 
                ((1/m1)*delC(2*i, Eigen::seq(0,2)).squaredNorm() + 
                (1/m2)*delC(2*i, Eigen::seq(3,5)).squaredNorm() + 
                (1/m3)*delC(2*i, Eigen::seq(6,8)).squaredNorm() + 
                (1/m4)*delC(2*i, Eigen::seq(9,11)).squaredNorm() + 
                alpha_hs(i)/(dt*dt));


            const double dlam_d = (-C_d - alpha_ds(i) * lambda_ds(i) / (dt*dt)) /
                ((1/m1)*delC(2*i+1, Eigen::seq(0,2)).squaredNorm() + 
                (1/m2)*delC(2*i+1, Eigen::seq(3,5)).squaredNorm() + 
                (1/m3)*delC(2*i+1, Eigen::seq(6,8)).squaredNorm() + 
                (1/m4)*delC(2*i+1, Eigen::seq(9,11)).squaredNorm() + 
                alpha_ds(i)/(dt*dt)); 

            // update Lagrange multipliers
            lambda_hs(i) += dlam_h;
            lambda_ds(i) += dlam_d;

            dlam_hs(i) = dlam_h;
            dlam_ds(i) = dlam_d;
        }
    }

    // calculate position updates
    for (int i = 0; i < _elements.rows(); i++)
    {
        const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);
            
        // extract masses of each vertex in the current element
        const double m1 = _m[elem(0)];
        const double m2 = _m[elem(1)];
        const double m3 = _m[elem(2)];
        const double m4 = _m[elem(3)];

        // update vertex positions
        position_updates.row(elem(0)) += delC(2*i, Eigen::seq(0,2)) * dlam_hs(i)/m1;
        position_updates.row(elem(1)) += delC(2*i, Eigen::seq(3,5)) * dlam_hs(i)/m2;
        position_updates.row(elem(2)) += delC(2*i, Eigen::seq(6,8)) * dlam_hs(i)/m3;
        position_updates.row(elem(3)) += delC(2*i, Eigen::seq(9,11))* dlam_hs(i)/m4;

        position_updates.row(elem(0)) += delC(2*i+1, Eigen::seq(0,2)) * dlam_ds(i)/m1;
        position_updates.row(elem(1)) += delC(2*i+1, Eigen::seq(3,5)) * dlam_ds(i)/m2;
        position_updates.row(elem(2)) += delC(2*i+1, Eigen::seq(6,8)) * dlam_ds(i)/m3;
        position_updates.row(elem(3)) += delC(2*i+1, Eigen::seq(9,11)) * dlam_ds(i)/m4;
    }

    _vertices += position_updates;

    _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
}

void XPBDMeshObject::_projectConstraintsSequentialInitLambda(const double dt)
{
    // accumulated hydrostatic Lagrange multipliers
    Eigen::VectorXd lambda_hs = _initial_lambda_hs/(dt*dt);//Eigen::VectorXd::Zero(_elements.rows());
    // accumulated deviatoric Lagrange multipliers
    Eigen::VectorXd lambda_ds = _initial_lambda_ds/(dt*dt);//Eigen::VectorXd::Zero(_elements.rows());

    // calculate initial guesses for lambdas based on last constraint
    for (int i = 0; i < _elements.rows(); i++)
    {
        const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);
        Eigen::Matrix3d X;
        X.col(0) = _x_prev.row(elem(0)) - _x_prev.row(elem(3));
        X.col(1) = _x_prev.row(elem(1)) - _x_prev.row(elem(3));
        X.col(2) = _x_prev.row(elem(2)) - _x_prev.row(elem(3));

        // compute F
        Eigen::Matrix3d F = X * _Q[i];

        // compute constraint itself
        const double C_d = std::sqrt(F.col(0).squaredNorm() + F.col(1).squaredNorm() + F.col(2).squaredNorm());
        // compute the deviatoric alpha
        const double alpha_d_tilde = 1/(_material.mu() * _vols[i]) / (dt*dt);

        // compute constraint itself
        const double C_h = F.determinant() - (1 + _material.mu()/_material.lambda());
        // compute the hydrostatic alpha
        const double alpha_h_tilde = 1/(_material.lambda() * _vols[i]) / (dt*dt);

        lambda_ds(i) = -1 / alpha_d_tilde * C_d;
        lambda_hs(i) = -1 / alpha_h_tilde * C_h;
    }

    std::cout << "lambdad(0): " << lambda_ds(0) << std::endl;
    std::cout << "lambdah(0): " << lambda_hs(0) << std::endl;
    

    // dlams
    Eigen::VectorXd dlam_hs = Eigen::VectorXd::Zero(_elements.rows());
    Eigen::VectorXd dlam_ds = Eigen::VectorXd::Zero(_elements.rows());

    // store positions before constraints are projected
    // this is x-tilde, seen in XPBD eqn 8 - used for computing primary residual
    VerticesMat inertial_positions = _vertices;

    // define Eigen loop variables
    Eigen::Matrix3d X, F, F_cross, C_h_grads, C_d_grads;
    Eigen::Vector3d C_h_grad_4, C_d_grad_4;
    for (unsigned gi = 0; gi < _num_iters; gi++)
    {
        for (int i = 0; i < _elements.rows(); i++)
        {
            const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);
            // extract masses of each vertex in the current element
            const double m1 = _m[elem(0)];
            const double m2 = _m[elem(1)];
            const double m3 = _m[elem(2)];
            const double m4 = _m[elem(3)];

            /** DEVIATORIC CONSTRAINT */
            // create the deformed shape matrix from current deformed vertex positions
            X.col(0) = _vertices.row(elem(0)) - _vertices.row(elem(3));
            X.col(1) = _vertices.row(elem(1)) - _vertices.row(elem(3));
            X.col(2) = _vertices.row(elem(2)) - _vertices.row(elem(3));

            // compute F
            F = X * _Q[i];
            
            // compute constraint itself
            const double C_d = std::sqrt(F.col(0).squaredNorm() + F.col(1).squaredNorm() + F.col(2).squaredNorm());

            // compute constraint gradient
            C_d_grads = 1/C_d * (F * _Q[i].transpose());
            // compute the deviatoric alpha
            const double alpha_d = 1/(_material.mu() * _vols[i]);

            const double beta_tilde = _damping_stiffness * (dt*dt);
            const double gamma_d = alpha_d / (dt*dt) * beta_tilde / dt;

            // by definition, the gradient for vertex 4 in the element is the negative sum of other gradients
            C_d_grad_4 = -C_d_grads.col(0) - C_d_grads.col(1) - C_d_grads.col(2);
            
            // const double denom_d = ((1/m1)*C_d_grads.col(0).squaredNorm() + (1/m2)*C_d_grads.col(1).squaredNorm() + (1/m3)*C_d_grads.col(2).squaredNorm() + (1/m4)*C_d_grad_4.squaredNorm() + alpha_d/(dt*dt));
            const double delC_x_prev_d =  C_d_grads.col(0).dot(_vertices.row(elem(0)) - _x_prev.row(elem(0))) +
                                        C_d_grads.col(1).dot(_vertices.row(elem(1)) - _x_prev.row(elem(1))) +
                                        C_d_grads.col(2).dot(_vertices.row(elem(2)) - _x_prev.row(elem(2))) +
                                        C_d_grad_4.dot(_vertices.row(elem(3)) - _x_prev.row(elem(3)));

            const double dlam_d = (-C_d - alpha_d * lambda_ds(i) / (dt*dt) - gamma_d * delC_x_prev_d) /
                ((1 + gamma_d) * (
                    (1/m1)*C_d_grads.col(0).squaredNorm() + 
                    (1/m2)*C_d_grads.col(1).squaredNorm() + 
                    (1/m3)*C_d_grads.col(2).squaredNorm() + 
                    (1/m4)*C_d_grad_4.squaredNorm()
                 ) + alpha_d/(dt*dt)); 
                // std::cout << "C_h: " << C_h << "\talpha_h: " << alpha_h <<"\tdlam_h: " << dlam_h << std::endl;
                // std::cout << "C_d: " << C_d << "\talpha_d: " << alpha_d << "\tdenominator: " <<  denom_d << "\tdlam_d: " << dlam_d << std::endl;
            
            _vertices.row(elem(0)) += C_d_grads.col(0) * dlam_d/m1;
            _vertices.row(elem(1)) += C_d_grads.col(1) * dlam_d/m2;
            _vertices.row(elem(2)) += C_d_grads.col(2) * dlam_d/m3;
            _vertices.row(elem(3)) += C_d_grad_4 * dlam_d/m4;


            /** HYDROSTATIC CONSTRAINT */

            // create the deformed shape matrix from current deformed vertex positions
            X.col(0) = _vertices.row(elem(0)) - _vertices.row(elem(3));
            X.col(1) = _vertices.row(elem(1)) - _vertices.row(elem(3));
            X.col(2) = _vertices.row(elem(2)) - _vertices.row(elem(3));

            // compute F
            F = X * _Q[i];

            // compute constraint itself
            const double C_h = F.determinant() - (1 + _material.mu()/_material.lambda());

            // compute constraint gradient
            F_cross.col(0) = F.col(1).cross(F.col(2));
            F_cross.col(1) = F.col(2).cross(F.col(0));
            F_cross.col(2) = F.col(0).cross(F.col(1));
            C_h_grads = F_cross * _Q[i].transpose();
            // compute the hydrostatic alpha
            const double alpha_h = 1/(_material.lambda() * _vols[i]);
            
            const double gamma_h = alpha_h / (dt*dt) * beta_tilde / dt;


            C_h_grad_4 = -C_h_grads.col(0) - C_h_grads.col(1) - C_h_grads.col(2);

            const double delC_x_prev_h =  C_h_grads.col(0).dot(_vertices.row(elem(0)) - _x_prev.row(elem(0))) +
                                        C_h_grads.col(1).dot(_vertices.row(elem(1)) - _x_prev.row(elem(1))) +
                                        C_h_grads.col(2).dot(_vertices.row(elem(2)) - _x_prev.row(elem(2))) +
                                        C_h_grad_4.dot(_vertices.row(elem(3)) - _x_prev.row(elem(3)));

            const double dlam_h = (-C_h - alpha_h * lambda_hs(i) / (dt*dt) - gamma_h * delC_x_prev_h) / 
                ((1 + gamma_h) * (
                    (1/m1)*C_h_grads.col(0).squaredNorm() + 
                    (1/m2)*C_h_grads.col(1).squaredNorm() + 
                    (1/m3)*C_h_grads.col(2).squaredNorm() + 
                    (1/m4)*C_h_grad_4.squaredNorm()
                 ) + alpha_h/(dt*dt));

            _vertices.row(elem(0)) += C_h_grads.col(0) * dlam_h/m1;
            _vertices.row(elem(1)) += C_h_grads.col(1) * dlam_h/m2;
            _vertices.row(elem(2)) += C_h_grads.col(2) * dlam_h/m3;
            _vertices.row(elem(3)) += C_h_grad_4 * dlam_h/m4;

            // update Lagrange multipliers
            lambda_hs(i) += dlam_h;
            lambda_ds(i) += dlam_d;
        }
    }

    _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
}

void XPBDMeshObject::_projectConstraintsRuckerFull(const double dt)
{
    VerticesMat inertial_positions = _vertices;
    
    std::vector<Eigen::Matrix<double, 3, 4>> C_grads(_elements.rows()*2);
    Eigen::VectorXd alpha_tildes(_elements.rows()*2);

    // compute RHS, which is just -C(x_tilde)
    Eigen::VectorXd neg_C(_elements.rows()*2);

    // define Eigen loop variables
    Eigen::Matrix3d X, F, F_cross, C_h_grads, C_d_grads;
    Eigen::Vector3d C_h_grad_4, C_d_grad_4;
    for (int i = 0; i < _elements.rows(); i++)
    {
        const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);

        // create the deformed shape matrix from current deformed vertex positions
        X.col(0) = _vertices.row(elem(0)) - _vertices.row(elem(3));
        X.col(1) = _vertices.row(elem(1)) - _vertices.row(elem(3));
        X.col(2) = _vertices.row(elem(2)) - _vertices.row(elem(3));

        // compute F
        F = X * _Q[i];

        /** HYDROSTATIC CONSTRAINT */
        // compute constraint itself
        const double C_h = F.determinant() - (1 + _material.mu()/_material.lambda());

        // compute constraint gradient
        F_cross.col(0) = F.col(1).cross(F.col(2));
        F_cross.col(1) = F.col(2).cross(F.col(0));
        F_cross.col(2) = F.col(0).cross(F.col(1));
        C_h_grads = F_cross * _Q[i].transpose();
        // compute the hydrostatic alpha
        const double alpha_h = 1/(_material.lambda() * _vols[i]);

        /** DEVIATORIC CONSTRAINT */
        // compute constraint itself
        const double C_d = std::sqrt(F.col(0).squaredNorm() + F.col(1).squaredNorm() + F.col(2).squaredNorm());

        // compute constraint gradient
        C_d_grads = 1/C_d * (F * _Q[i].transpose());
        // compute the deviatoric alpha
        const double alpha_d = 1/(_material.mu() * _vols[i]);

        // by definition, the gradient for vertex 4 in the element is the negative sum of other gradients
        C_h_grad_4 = -C_h_grads.col(0) - C_h_grads.col(1) - C_h_grads.col(2);
        C_d_grad_4 = -C_d_grads.col(0) - C_d_grads.col(1) - C_d_grads.col(2);

        alpha_tildes(2*i) = alpha_h / (dt*dt);
        alpha_tildes(2*i+1) = alpha_d / (dt*dt);

        C_grads[2*i].col(0) = C_h_grads.col(0);
        C_grads[2*i].col(1) = C_h_grads.col(1);
        C_grads[2*i].col(2) = C_h_grads.col(2);
        C_grads[2*i].col(3) = C_h_grad_4;

        C_grads[2*i+1].col(0) = C_d_grads.col(0);
        C_grads[2*i+1].col(1) = C_d_grads.col(1);
        C_grads[2*i+1].col(2) = C_d_grads.col(2);
        C_grads[2*i+1].col(3) = C_d_grad_4;

        neg_C(2*i) = -C_h;
        neg_C(2*i+1) = -C_d;
    }

    Eigen::MatrixXd delC_Minv_delCT = Eigen::MatrixXd::Zero(2*_elements.rows(), 2*_elements.rows());
    // iterate through vertices and add contributions to delC_Minv_delCT
    for (int vi = 0; vi < _vertices.rows(); vi++)
    {
        double inv_mass = 1/_m[vi];

        for (unsigned ci = 0; ci < _constraints_per_position[vi].size(); ci++)
        {
            unsigned C_index_i = _constraints_per_position[vi][ci].first;
            unsigned C_grad_ind_i = _constraints_per_position[vi][ci].second;
            const Eigen::Vector3d C_grad_i = C_grads[C_index_i].col(C_grad_ind_i);

            for (unsigned cj = ci; cj < _constraints_per_position[vi].size(); cj++)
            {
                unsigned C_index_j = _constraints_per_position[vi][cj].first;
                unsigned C_grad_ind_j = _constraints_per_position[vi][cj].second;
                const Eigen::Vector3d C_grad_j = C_grads[C_index_j].col(C_grad_ind_j);

                const double inv_mass_grad_dot = inv_mass * C_grad_i.dot(C_grad_j);
                delC_Minv_delCT(C_index_i, C_index_j) += inv_mass_grad_dot;
                delC_Minv_delCT(C_index_j, C_index_i) += inv_mass_grad_dot;
            }
        }
    }

    // add alpha along diagonals to complete LHS matrix
    for (int i = 0; i < _elements.rows()*2; i++)
    {
        delC_Minv_delCT(i,i) += alpha_tildes(i);
    }

    // neg_C is RHS and delC_Minv_delCT is LHS
    std::cout << "Solving matrix system..." << std::endl;
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> dec(delC_Minv_delCT);
    const Eigen::VectorXd& lambda = dec.solve(neg_C);
    std::cout << "Done solving matrix system!" << std::endl;

    // use lambda to update X
    for (int vi = 0; vi < _vertices.rows(); vi++)
    {
        double inv_mass = 1/_m[vi];

        for (unsigned ci = 0; ci < _constraints_per_position[vi].size(); ci++)
        {
            unsigned C_index_i = _constraints_per_position[vi][ci].first;
            unsigned C_grad_ind_i = _constraints_per_position[vi][ci].second;
            const Eigen::Vector3d C_grad_i = C_grads[C_index_i].col(C_grad_ind_i);
            _vertices.row(vi) += inv_mass * C_grad_i * lambda(C_index_i);
        }
    }

    // split up lambda for calculation of residual
    Eigen::VectorXd lambda_hs(_elements.rows());
    Eigen::VectorXd lambda_ds(_elements.rows());
    for (int i = 0; i < _elements.rows(); i++)
    {
        lambda_hs(i) = lambda(2*i);
        lambda_ds(i) = lambda(2*i+1);
    }
    _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
    
}

void XPBDMeshObject::_calculateForces()
{
    Eigen::Matrix<double, -1, 3> forces = Eigen::MatrixXd::Zero(_vertices.rows(), 3);

    // define Eigen loop variables
    Eigen::Matrix3d X, F, F_cross, C_h_grads, C_d_grads;
    Eigen::Vector3d C_h_grad_4, C_d_grad_4;
    Eigen::Vector3d f_x1, f_x2, f_x3, f_x4;
    for (int i = 0; i < _elements.rows(); i++)
    {
        const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);

        // create the deformed shape matrix from current deformed vertex positions
        X.col(0) = _vertices.row(elem(0)) - _vertices.row(elem(3));
        X.col(1) = _vertices.row(elem(1)) - _vertices.row(elem(3));
        X.col(2) = _vertices.row(elem(2)) - _vertices.row(elem(3));

        // compute F
        F = X * _Q[i];

        /** HYDROSTATIC CONSTRAINT */
        // compute constraint itself
        const double C_h = F.determinant() - (1 + _material.mu()/_material.lambda());

        // compute constraint gradient
        F_cross.col(0) = F.col(1).cross(F.col(2));
        F_cross.col(1) = F.col(2).cross(F.col(0));
        F_cross.col(2) = F.col(0).cross(F.col(1));
        C_h_grads = F_cross * _Q[i].transpose();
        // compute the hydrostatic alpha
        const double alpha_h = 1/(_material.lambda() * _vols[i]);

        /** DEVIATORIC CONSTRAINT */
        // compute constraint itself
        const double C_d = std::sqrt(F.col(0).squaredNorm() + F.col(1).squaredNorm() + F.col(2).squaredNorm());

        // compute constraint gradient
        C_d_grads = 1/C_d * (F * _Q[i].transpose());
        // compute the deviatoric alpha
        const double alpha_d = 1/(_material.mu() * _vols[i]);

        // by definition, the gradient for vertex 4 in the element is the negative sum of other gradients
        C_h_grad_4 = -C_h_grads.col(0) - C_h_grads.col(1) - C_h_grads.col(2);
        C_d_grad_4 = -C_d_grads.col(0) - C_d_grads.col(1) - C_d_grads.col(2);

        // compute forces per vertex
        f_x1 = -C_h_grads.col(0) / alpha_h * C_h + C_d_grads.col(0) / alpha_d * C_d;
        f_x2 = -C_h_grads.col(1) / alpha_h * C_h + C_d_grads.col(1) / alpha_d * C_d;
        f_x3 = -C_h_grads.col(2) / alpha_h * C_h + C_d_grads.col(2) / alpha_d * C_d;
        f_x4 = -C_h_grad_4 / alpha_h * C_h + C_d_grad_4 / alpha_d * C_d;

        // compute forces
        forces.row(elem(0)) += f_x1;
        forces.row(elem(1)) += f_x2;
        forces.row(elem(2)) += f_x3;
        forces.row(elem(3)) += f_x4;
    }

    std::cout << "Forces:\n" << forces << std::endl;

}

void XPBDMeshObject::_calculateResiduals(const double dt, const VerticesMat& inertial_positions, const Eigen::VectorXd& lambda_hs, const Eigen::VectorXd& lambda_ds)
{
    // calculate constraint violation
    Eigen::Matrix<double, -1, 3> M(_vertices.rows(), 3);
    M.col(0) = _m;
    M.col(1) = _m;
    M.col(2) = _m;
    Eigen::Matrix<double, -1, 3> Mx(_vertices.rows(), 3);
    Mx = M.array() * (_vertices - inertial_positions).array();
    
    Eigen::Matrix<double, -1, 3> del_C_lam = Eigen::MatrixXd::Zero(_vertices.rows(), 3);
    Eigen::Matrix<double, -1, 3> del_C_alpha_C = Eigen::MatrixXd::Zero(_vertices.rows(), 3);

    Eigen::VectorXd lambdas(_elements.rows() * 2);
    Eigen::VectorXd constraint_residual(_elements.rows() * 2);

    Eigen::VectorXd cur_vols(_elements.rows());

    // define Eigen loop variables
    Eigen::Matrix3d X, F, F_cross, C_h_grads, C_d_grads;
    Eigen::Vector3d C_h_grad_4, C_d_grad_4;
    for (int i = 0; i < _elements.rows(); i++)
    {
        const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);

        // create the deformed shape matrix from current deformed vertex positions
        X.col(0) = _vertices.row(elem(0)) - _vertices.row(elem(3));
        X.col(1) = _vertices.row(elem(1)) - _vertices.row(elem(3));
        X.col(2) = _vertices.row(elem(2)) - _vertices.row(elem(3));

        cur_vols(i) = std::abs(X.determinant()/6);

        // compute F
        F = X * _Q[i];

        /** HYDROSTATIC CONSTRAINT */
        // compute constraint itself
        const double C_h = F.determinant() - (1 + _material.mu()/_material.lambda());

        // compute constraint gradient
        F_cross.col(0) = F.col(1).cross(F.col(2));
        F_cross.col(1) = F.col(2).cross(F.col(0));
        F_cross.col(2) = F.col(0).cross(F.col(1));
        C_h_grads = F_cross * _Q[i].transpose();
        // compute the hydrostatic alpha
        const double alpha_h = 1/(_material.lambda() * _vols[i]);

        /** DEVIATORIC CONSTRAINT */
        // compute constraint itself
        const double C_d = std::sqrt(F.col(0).squaredNorm() + F.col(1).squaredNorm() + F.col(2).squaredNorm());

        // compute constraint gradient
        C_d_grads = 1/C_d * (F * _Q[i].transpose());
        // compute the deviatoric alpha
        const double alpha_d = 1/(_material.mu() * _vols[i]);

        // by definition, the gradient for vertex 4 in the element is the negative sum of other gradients
        C_h_grad_4 = -C_h_grads.col(0) - C_h_grads.col(1) - C_h_grads.col(2);
        C_d_grad_4 = -C_d_grads.col(0) - C_d_grads.col(1) - C_d_grads.col(2);

        // primary residual
        del_C_lam.row(elem(0)) += C_h_grads.col(0) * lambda_hs(i) + C_d_grads.col(0) * lambda_ds(i);
        del_C_lam.row(elem(1)) += C_h_grads.col(1) * lambda_hs(i) + C_d_grads.col(1) * lambda_ds(i);
        del_C_lam.row(elem(2)) += C_h_grads.col(2) * lambda_hs(i) + C_d_grads.col(2) * lambda_ds(i);
        del_C_lam.row(elem(3)) += C_h_grad_4 * lambda_hs(i) + C_d_grad_4 * lambda_ds(i); 

        // constraint residual
        const double c_res_h = C_h + alpha_h / (dt*dt) * lambda_hs(i);
        const double c_res_d = C_d + alpha_d / (dt*dt) * lambda_ds(i);
        // std::cout << "C_h: " << C_h << "\talpha_h: " << alpha_h << "\tlambda_h: " << lambda_hs(i) << std::endl;
        // std::cout << "C_d: " << C_d << "\talpha_d: " << alpha_d << "\tlambda_d: " << lambda_ds(i) << std::endl;
        constraint_residual(2*i) = c_res_h;
        constraint_residual(2*i+1) = c_res_d;

        // compute forces
        del_C_alpha_C.row(elem(0)) += C_h_grads.col(0) / alpha_h * C_h + C_d_grads.col(0) / alpha_d * C_d;
        del_C_alpha_C.row(elem(1)) += C_h_grads.col(1) / alpha_h * C_h + C_d_grads.col(1) / alpha_d * C_d;
        del_C_alpha_C.row(elem(2)) += C_h_grads.col(2) / alpha_h * C_h + C_d_grads.col(2) / alpha_d * C_d;
        del_C_alpha_C.row(elem(3)) += C_h_grad_4 / alpha_h * C_h + C_d_grad_4 / alpha_d * C_d;
    }

     

   
    VerticesMat primary_residual = Mx - del_C_lam;
    const double primary_residual_abs_mean = primary_residual.cwiseAbs().mean();
    const double primary_residual_rms = std::sqrt(primary_residual.squaredNorm()/primary_residual.size());
    
    VerticesMat Mxdt2 = Mx.array() / (dt*dt);
    // std::cout << "mxdt2: " << Mxdt2(0,2) << "\tforce: " << del_C_alpha_C(0,2) << std::endl;
    VerticesMat dynamics_residual = Mx - del_C_alpha_C*(dt*dt);
    // std::cout << dynamics_residual << std::endl;
    const double dynamics_residual_abs_mean = dynamics_residual.cwiseAbs().mean();
    const double dynamics_residual_rms = std::sqrt(dynamics_residual.squaredNorm()/dynamics_residual.size());

    const double constraint_residual_rms = std::sqrt(constraint_residual.squaredNorm()/constraint_residual.size());
    const double constraint_residual_abs_mean = constraint_residual.cwiseAbs().mean();

    _primary_residual = primary_residual_rms;
    _constraint_residual = constraint_residual_rms;

    // std::cout << primary_residual << std::endl;

    std::cout << name() << " Residuals:" << std::endl;
    std::cout << "\tDynamics residual\t--\tRMS:\t" << dynamics_residual_rms << "\t\tAverage:\t" << dynamics_residual_abs_mean << std::endl;
    std::cout << "\tPrimary residual\t--\tRMS:\t" << primary_residual_rms << "\t\tAverage:\t" << primary_residual_abs_mean << std::endl;
    std::cout << "\tConstraint residual\t--\tRMS:\t" << constraint_residual_rms << "\t\tAverage:\t" << constraint_residual_abs_mean << std::endl;
    std::cout << "\tLambda_h(0):\t" << lambda_hs(0) << "\tLambda_d(0):\t" << lambda_ds(0) << std::endl;
    std::cout << "\tCurrent volume / initial volume:\t" << cur_vols.sum() / _vols.sum() << std::endl;
    std::cout << "\tConstraint residuals for first element: " << constraint_residual(0) << "\t, " << constraint_residual(1) << std::endl;
    // std::cout << "\tForces:\n" << forces << std::endl;
}

void XPBDMeshObject::_projectCollisionConstraints()
{
    // for now, forbid vertex z-positions going below 0
    for (unsigned i = 0; i < _vertices.rows(); i++)
    {
        if (_vertices(i,2) < 0)
        {
            _vertices(i,2) = 0;
        }
    }

    // snap fixed vertices back to previous position (this should be their initial position)
    for (int i = 0; i < _vertices.rows(); i++)
    {
        if (_fixed_vertices(i) == true)
        {
            _vertices.row(i) = _x_prev.row(i);
        }
    }
}

void XPBDMeshObject::_updateVelocities(const double dt)
{
    // velocities are simply (cur_pos - last_pos) / deltaT
    _v = (_vertices - _x_prev) / dt;
    // set _x_prev to be ready for the next substep
    _x_prev = _vertices;
}
