#include "FirstOrderXPBDMeshObject.hpp"
#include "OutputSimulation.hpp"

FirstOrderXPBDMeshObject::FirstOrderXPBDMeshObject(const FirstOrderXPBDMeshObjectConfig* config)
    : ElasticMeshObject(config)
{
    _num_iters = config->numSolverIters().value();
    
    _solve_mode = config->solveMode().value();

    _residual_policy = config->residualPolicy().value();

    _damping_multiplier = config->dampingMultiplier().value();

    _init();
    _precomputeQuantities();
}

void FirstOrderXPBDMeshObject::_init()
{
    // reserve space for vectors
    // Q and volumes are per-element
    _Q.resize(_elements.rows());
    _vols.conservativeResize(_elements.rows());
    // masses are per-vertex
    _m = Eigen::VectorXd::Zero(_vertices.rows());
    _v_volume = Eigen::VectorXd::Zero(_vertices.rows());

    // initialize loop variables to zeroes
    _lX = Eigen::Matrix3d::Zero();
    _lF = Eigen::Matrix3d::Zero();
    _lF_cross = Eigen::Matrix3d::Zero();
    _lC_h_grads = Eigen::Matrix<double, 3, 4>::Zero();
    _lC_d_grads = Eigen::Matrix<double, 3, 4>::Zero();

    _primary_residual = VerticesMat::Zero(_vertices.rows(), 3);
    _dynamics_residual_rms = 0;
    _constraint_residual_rms = 0;
    _primary_residual_rms = 0;
    _vol_ratio = 1;
    _x_prev = _vertices;

    _jacobi_scaling = 1;
}

void FirstOrderXPBDMeshObject::_precomputeQuantities()
{
    
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

        // add up total volume attached to each vertex
        _v_volume(_elements(i,0)) += vol;
        _v_volume(_elements(i,1)) += vol;
        _v_volume(_elements(i,2)) += vol;
        _v_volume(_elements(i,3)) += vol;
    }

    // compute the number of elements that share a given node
    _num_elements_with_position = Eigen::VectorXi::Zero(_vertices.rows());
    for (int i = 0; i < _elements.rows(); i++)
    {
        _num_elements_with_position(_elements(i,0))++;
        _num_elements_with_position(_elements(i,1))++;
        _num_elements_with_position(_elements(i,2))++;
        _num_elements_with_position(_elements(i,3))++;
    }
    _jacobi_scaling = 1.0/_num_elements_with_position.maxCoeff();
    std::cout << "\nJacobi scaling: " << _jacobi_scaling << std::endl;

    std::cout << "\nSmallest Edge Length: " << smallestEdgeLength() << std::endl;
}

std::string FirstOrderXPBDMeshObject::toString() const
{
    return ElasticMeshObject::toString() + "\n\tSolve mode: " + solveMode() +
        "\n\tNum solver iterations: " + std::to_string(_num_iters) +
        "\n\tMass-to-damping multiplier: " + std::to_string(_damping_multiplier);
}

void FirstOrderXPBDMeshObject::update(const double dt, const double g_accel)
{
    // auto t1 = std::chrono::steady_clock::now();
    _movePositionsIntertially(dt, g_accel);
    // auto t2 = std::chrono::steady_clock::now();
    if (_solve_mode == FirstOrderXPBDSolveMode::SEQUENTIAL)
        _projectConstraintsSequential(dt);
    else if (_solve_mode == FirstOrderXPBDSolveMode::SIMULTANEOUS)
        _projectConstraintsSimultaneous(dt);
    else if (_solve_mode == FirstOrderXPBDSolveMode::SIMULTANEOUS_JACOBI)
        _projectConstraintsSimultaneousJacobi(dt);
    else if (_solve_mode == FirstOrderXPBDSolveMode::SIMULTANEOUS_CONVERGENT_JACOBI)
        _projectConstraintsSimultaneousConvergentJacobi(dt);

    // auto t3 = std::chrono::steady_clock::now();
    _projectCollisionConstraints(dt);
    // auto t4 = std::chrono::steady_clock::now();
    _updateVelocities(dt);
    // auto t5 = std::chrono::steady_clock::now();

    // std::cout << "XPBDMeshObject::update =========" << std::endl;
    // std::cout << "\tmovePositionsInertially took " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << " us" << std::endl;
    // std::cout << "\tprojectConstraints took " << std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count() << " us" << std::endl;
    // std::cout << "\tprojectCollisionConstraints took " << std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count() << " us" << std::endl;
    // std::cout << "\tupdateVelocities took " << std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count() << " us" << std::endl;
}

std::string FirstOrderXPBDMeshObject::solveMode() const
{
    if (_solve_mode == FirstOrderXPBDSolveMode::SEQUENTIAL)
        return "Sequential";
    if (_solve_mode == FirstOrderXPBDSolveMode::SIMULTANEOUS)
        return "Simultaneous";
    if (_solve_mode == FirstOrderXPBDSolveMode::SIMULTANEOUS_JACOBI)
        return "Simultaneous-Jacobi";
    if (_solve_mode == FirstOrderXPBDSolveMode::SIMULTANEOUS_CONVERGENT_JACOBI)
        return "Simultaneous-Convergent-Jacobi";
    
    return "";
}

void FirstOrderXPBDMeshObject::_movePositionsIntertially(const double dt, const double g_accel)
{
    for (int i = 0; i < _vertices.rows(); i++)
    {
        _vertices(i,2) += -g_accel*_m[i] * dt / _damping_multiplier;
    }   
}

void FirstOrderXPBDMeshObject::_projectConstraintsSequential(const double dt)
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


    for (unsigned gi = 0; gi < _num_iters; gi++)
    {
        for (int i = 0; i < _elements.rows(); i++)
        {
            const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);
            // extract masses of each vertex in the current element
            const double b1 = _v_volume[elem(0)] * _damping_multiplier;
            const double b2 = _v_volume[elem(1)] * _damping_multiplier;
            const double b3 = _v_volume[elem(2)] * _damping_multiplier;
            const double b4 = _v_volume[elem(3)] * _damping_multiplier;

            // extract masses of each vertex in the current element
            const double inv_b1 = 1.0/b1;
            const double inv_b2 = 1.0/b2;
            const double inv_b3 = 1.0/b3;
            const double inv_b4 = 1.0/b4;

            /** DEVIATORIC CONSTRAINT */

            _computeF(i, _lX, _lF);
            const double C_d = _computeDeviatoricConstraint(_lF, _Q[i], _lC_d_grads);

            // compute the deviatoric alpha
            const double alpha_d = 1/(_material.mu() * _vols[i]);
            const double alpha_d_tilde = alpha_d / dt;

            const double dlam_d = (-C_d - alpha_d_tilde * lambda_ds(i)) / ((
                    (inv_b1)*_lC_d_grads.col(0).squaredNorm() + 
                    (inv_b2)*_lC_d_grads.col(1).squaredNorm() + 
                    (inv_b3)*_lC_d_grads.col(2).squaredNorm() + 
                    (inv_b4)*_lC_d_grads.col(3).squaredNorm()
                 ) + alpha_d_tilde);

            _vertices.row(elem(0)) += _lC_d_grads.col(0) * dlam_d * inv_b1;
            _vertices.row(elem(1)) += _lC_d_grads.col(1) * dlam_d * inv_b2;
            _vertices.row(elem(2)) += _lC_d_grads.col(2) * dlam_d * inv_b3;
            _vertices.row(elem(3)) += _lC_d_grads.col(3) * dlam_d * inv_b4;


            /** HYDROSTATIC CONSTRAINT */

            _computeF(i, _lX, _lF);
            const double C_h = _computeHydrostaticConstraint(_lF, _Q[i], _lF_cross, _lC_h_grads);
            // compute the hydrostatic alpha
            const double alpha_h = 1/(_material.lambda() * _vols[i]);
            const double alpha_h_tilde = alpha_h / dt;
            
            const double dlam_h = (-C_h - alpha_h_tilde * lambda_hs(i)) / ((
                    (inv_b1)*_lC_h_grads.col(0).squaredNorm() + 
                    (inv_b2)*_lC_h_grads.col(1).squaredNorm() + 
                    (inv_b3)*_lC_h_grads.col(2).squaredNorm() + 
                    (inv_b4)*_lC_h_grads.col(3).squaredNorm()
                 ) + alpha_h_tilde);

            _vertices.row(elem(0)) += _lC_h_grads.col(0) * dlam_h * inv_b1;
            _vertices.row(elem(1)) += _lC_h_grads.col(1) * dlam_h * inv_b2;
            _vertices.row(elem(2)) += _lC_h_grads.col(2) * dlam_h * inv_b3;
            _vertices.row(elem(3)) += _lC_h_grads.col(3) * dlam_h * inv_b4;

            // update Lagrange multipliers
            lambda_hs(i) += dlam_h;
            lambda_ds(i) += dlam_d;
        }

        // if the residual policy is to calculate every iteration, do that
        if (_residual_policy == FirstOrderXPBDResidualPolicy::EVERY_ITERATION)
        {
            if (const OutputSimulation* output_sim = dynamic_cast<const OutputSimulation*>(_sim))
            {
                // _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
                // if we are calculating the residuals every Gauss Seidel iteration, it's likely we want to create
                // a residual vs. iteration number plot, so write info to file
                // output_sim->printInfo();
            }
        }
    }

    // if the residual policy is to calculate every substep, do that
    if (_residual_policy == FirstOrderXPBDResidualPolicy::EVERY_SUBSTEP)
    {
        // _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
    }
}

void FirstOrderXPBDMeshObject::_projectConstraintsSimultaneous(const double dt)
{
    // accumulated hydrostatic Lagrange multipliers
    Eigen::VectorXd lambda_hs = Eigen::VectorXd::Zero(_elements.rows());
    // accumulated deviatoric Lagrange multipliers
    Eigen::VectorXd lambda_ds = Eigen::VectorXd::Zero(_elements.rows());

    // store positions before constraints are projected
    // this is x-tilde, seen in XPBD eqn 8 - used for computing primary residual
    VerticesMat inertial_positions = _vertices;

    for (unsigned gi = 0; gi < _num_iters; gi++)
    {
        for (int i = 0; i < _elements.rows(); i++)
        {
            const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);

            // extract masses of each vertex in the current element
            const double b1 = _v_volume[elem(0)] * _damping_multiplier;
            const double b2 = _v_volume[elem(1)] * _damping_multiplier;
            const double b3 = _v_volume[elem(2)] * _damping_multiplier;
            const double b4 = _v_volume[elem(3)] * _damping_multiplier;

            _computeF(i, _lX, _lF);

            /** DEVIATORIC CONSTRAINT */
            const double C_d = _computeDeviatoricConstraint(_lF, _Q[i], _lC_d_grads);

            // compute the deviatoric alpha
            const double alpha_d = 1/(_material.mu() * _vols[i]);
            const double alpha_d_tilde = alpha_d / dt;

            /** HYDROSTATIC CONSTRAINT */

            const double C_h = _computeHydrostaticConstraint(_lF, _Q[i], _lF_cross, _lC_h_grads);

            // compute the hydrostatic alpha
            const double alpha_h = 1/(_material.lambda() * _vols[i]);
            const double alpha_h_tilde = alpha_h / dt;


            const double inv_b1 = 1/b1;
            const double inv_b2 = 1/b2;
            const double inv_b3 = 1/b3;
            const double inv_b4 = 1/b4;

            // solve the 2x2 system
            const double a11 = inv_b1*_lC_h_grads.col(0).squaredNorm() + 
                               inv_b2*_lC_h_grads.col(1).squaredNorm() + 
                               inv_b3*_lC_h_grads.col(2).squaredNorm() + 
                               inv_b4*_lC_h_grads.col(3).squaredNorm() + 
                               alpha_h_tilde;
            const double a12 = inv_b1*_lC_h_grads.col(0).dot(_lC_d_grads.col(0)) +
                               inv_b2*_lC_h_grads.col(1).dot(_lC_d_grads.col(1)) +
                               inv_b3*_lC_h_grads.col(2).dot(_lC_d_grads.col(2)) +
                               inv_b4*_lC_h_grads.col(3).dot(_lC_d_grads.col(3));
            const double a21 = a12;
            const double a22 = inv_b1*_lC_d_grads.col(0).squaredNorm() + 
                               inv_b2*_lC_d_grads.col(1).squaredNorm() + 
                               inv_b3*_lC_d_grads.col(2).squaredNorm() + 
                               inv_b4*_lC_d_grads.col(3).squaredNorm() + 
                               alpha_d_tilde;
            const double k1 = -C_h - alpha_h_tilde * lambda_hs(i);
            const double k2 = -C_d - alpha_d_tilde * lambda_ds(i);

            const double detA = a11*a22 - a21*a12;

            const double dlam_h = (k1*a22 - k2*a12) / detA;
            const double dlam_d = (a11*k2 - a21*k1) / detA;


            // update vertex positions
            _vertices.row(elem(0)) += _lC_h_grads.col(0) * dlam_h * inv_b1 + _lC_d_grads.col(0) * dlam_d * inv_b1;
            _vertices.row(elem(1)) += _lC_h_grads.col(1) * dlam_h * inv_b2 + _lC_d_grads.col(1) * dlam_d * inv_b2;
            _vertices.row(elem(2)) += _lC_h_grads.col(2) * dlam_h * inv_b3 + _lC_d_grads.col(2) * dlam_d * inv_b3;
            _vertices.row(elem(3)) += _lC_h_grads.col(3) * dlam_h * inv_b4 + _lC_d_grads.col(3) * dlam_d * inv_b4;
            

            // update Lagrange multipliers
            lambda_hs(i) += dlam_h;
            lambda_ds(i) += dlam_d;
        }
        
        // if the residual policy is to calculate every iteration, do that
        if (_residual_policy == FirstOrderXPBDResidualPolicy::EVERY_ITERATION)
        {
            if (const OutputSimulation* output_sim = dynamic_cast<const OutputSimulation*>(_sim))
            {
                // _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
                // if we are calculating the residuals every Gauss Seidel iteration, it's likely we want to create
                // a residual vs. iteration number plot, so write info to file
                // output_sim->printInfo();
            }
        }
    }

    // if the residual policy is to calculate every substep, do that
    if (_residual_policy == FirstOrderXPBDResidualPolicy::EVERY_SUBSTEP)
    {
        // _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
    }
}

void FirstOrderXPBDMeshObject::_projectConstraintsSimultaneousJacobi(const double dt)
{
    // accumulated hydrostatic Lagrange multipliers
    Eigen::VectorXd lambda_hs = Eigen::VectorXd::Zero(_elements.rows());
    // accumulated deviatoric Lagrange multipliers
    Eigen::VectorXd lambda_ds = Eigen::VectorXd::Zero(_elements.rows());

    // store positions before constraints are projected
    // this is x-tilde, seen in XPBD eqn 8 - used for computing primary residual
    VerticesMat inertial_positions = _vertices;

    for (unsigned gi = 0; gi < _num_iters; gi++)
    {
        VerticesMat dx = VerticesMat::Zero(_vertices.rows(), 3);

        for (int i = 0; i < _elements.rows(); i++)
        {
            const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);

            // extract masses of each vertex in the current element
            const double b1 = _damping_multiplier * _v_volume[elem(0)] ;
            const double b2 = _damping_multiplier * _v_volume[elem(1)] ;
            const double b3 = _damping_multiplier * _v_volume[elem(2)] ;
            const double b4 = _damping_multiplier * _v_volume[elem(3)] ;

            _computeF(i, _lX, _lF);

            /** DEVIATORIC CONSTRAINT */
            const double C_d = _computeDeviatoricConstraint(_lF, _Q[i], _lC_d_grads);

            // compute the deviatoric alpha
            const double alpha_d = 1/(_material.mu() * _vols[i]);
            const double alpha_d_tilde = alpha_d / dt;

            /** HYDROSTATIC CONSTRAINT */

            const double C_h = _computeHydrostaticConstraint(_lF, _Q[i], _lF_cross, _lC_h_grads);

            // compute the hydrostatic alpha
            const double alpha_h = 1/(_material.lambda() * _vols[i]);
            const double alpha_h_tilde = alpha_h / dt;


            const double inv_b1 = 1/b1;
            const double inv_b2 = 1/b2;
            const double inv_b3 = 1/b3;
            const double inv_b4 = 1/b4;

            // solve the 2x2 system
            const double a11 = inv_b1*_lC_h_grads.col(0).squaredNorm() + 
                               inv_b2*_lC_h_grads.col(1).squaredNorm() + 
                               inv_b3*_lC_h_grads.col(2).squaredNorm() + 
                               inv_b4*_lC_h_grads.col(3).squaredNorm() + 
                               alpha_h_tilde;
            const double a12 = inv_b1*_lC_h_grads.col(0).dot(_lC_d_grads.col(0)) +
                               inv_b2*_lC_h_grads.col(1).dot(_lC_d_grads.col(1)) +
                               inv_b3*_lC_h_grads.col(2).dot(_lC_d_grads.col(2)) +
                               inv_b4*_lC_h_grads.col(3).dot(_lC_d_grads.col(3));
            const double a21 = a12;
            const double a22 = inv_b1*_lC_d_grads.col(0).squaredNorm() + 
                               inv_b2*_lC_d_grads.col(1).squaredNorm() + 
                               inv_b3*_lC_d_grads.col(2).squaredNorm() + 
                               inv_b4*_lC_d_grads.col(3).squaredNorm() + 
                               alpha_d_tilde;
            const double k1 = -C_h - alpha_h_tilde * lambda_hs(i);
            const double k2 = -C_d - alpha_d_tilde * lambda_ds(i);

            const double detA = a11*a22 - a21*a12;

            const double dlam_h = (k1*a22 - k2*a12) / detA;
            const double dlam_d = (a11*k2 - a21*k1) / detA;

            // update Lagrange multipliers
            lambda_hs(i) += dlam_h;
            lambda_ds(i) += dlam_d;


            const double scaling1 = 1.0;///_num_elements_with_position(elem(0));
            const double scaling2 = 1.0;///_num_elements_with_position(elem(1));
            const double scaling3 = 1.0;///_num_elements_with_position(elem(2));
            const double scaling4 = 1.0;///_num_elements_with_position(elem(3));

            // update nodal forces (delC*lambda)
            dx.row(elem(0)) += scaling1 * inv_b1 * (_lC_h_grads.col(0) * dlam_h + _lC_d_grads.col(0) * dlam_d);
            dx.row(elem(1)) += scaling2 * inv_b2 * (_lC_h_grads.col(1) * dlam_h + _lC_d_grads.col(1) * dlam_d);
            dx.row(elem(2)) += scaling3 * inv_b3 * (_lC_h_grads.col(2) * dlam_h + _lC_d_grads.col(2) * dlam_d);
            dx.row(elem(3)) += scaling4 * inv_b4 * (_lC_h_grads.col(3) * dlam_h + _lC_d_grads.col(3) * dlam_d);
        }

        _vertices += dx;
        
        // if the residual policy is to calculate every iteration, do that
        if (_residual_policy == FirstOrderXPBDResidualPolicy::EVERY_ITERATION)
        {
            if (const OutputSimulation* output_sim = dynamic_cast<const OutputSimulation*>(_sim))
            {
                // _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
                // if we are calculating the residuals every Gauss Seidel iteration, it's likely we want to create
                // a residual vs. iteration number plot, so write info to file
                // output_sim->printInfo();
            }
        }
    }

    // if the residual policy is to calculate every substep, do that
    if (_residual_policy == FirstOrderXPBDResidualPolicy::EVERY_SUBSTEP)
    {
        // _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
    }
}

void FirstOrderXPBDMeshObject::_projectConstraintsSimultaneousConvergentJacobi(const double dt)
{
    // accumulated hydrostatic Lagrange multipliers
    Eigen::VectorXd lambda_hs = Eigen::VectorXd::Zero(_elements.rows());
    // accumulated deviatoric Lagrange multipliers
    Eigen::VectorXd lambda_ds = Eigen::VectorXd::Zero(_elements.rows());

    // store positions before constraints are projected
    // this is x-tilde, seen in XPBD eqn 8 - used for computing primary residual
    VerticesMat inertial_positions = _vertices;

    for (unsigned gi = 0; gi < _num_iters; gi++)
    {
        VerticesMat nodal_forces = VerticesMat::Zero(_vertices.rows(), 3);

        for (int i = 0; i < _elements.rows(); i++)
        {
            const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);

            // extract masses of each vertex in the current element
            const double b1 = _v_volume[elem(0)] * _damping_multiplier;
            const double b2 = _v_volume[elem(1)] * _damping_multiplier;
            const double b3 = _v_volume[elem(2)] * _damping_multiplier;
            const double b4 = _v_volume[elem(3)] * _damping_multiplier;

            _computeF(i, _lX, _lF);

            /** DEVIATORIC CONSTRAINT */
            const double C_d = _computeDeviatoricConstraint(_lF, _Q[i], _lC_d_grads);

            // compute the deviatoric alpha
            const double alpha_d = 1/(_material.mu() * _vols[i]);
            const double alpha_d_tilde = alpha_d / dt;

            /** HYDROSTATIC CONSTRAINT */

            const double C_h = _computeHydrostaticConstraint(_lF, _Q[i], _lF_cross, _lC_h_grads);

            // compute the hydrostatic alpha
            const double alpha_h = 1/(_material.lambda() * _vols[i]);
            const double alpha_h_tilde = alpha_h / dt;


            const double inv_b1 = 1/b1;
            const double inv_b2 = 1/b2;
            const double inv_b3 = 1/b3;
            const double inv_b4 = 1/b4;

            // solve the 2x2 system
            const double a11 = inv_b1*_lC_h_grads.col(0).squaredNorm() + 
                               inv_b2*_lC_h_grads.col(1).squaredNorm() + 
                               inv_b3*_lC_h_grads.col(2).squaredNorm() + 
                               inv_b4*_lC_h_grads.col(3).squaredNorm() + 
                               alpha_h_tilde;
            const double a12 = inv_b1*_lC_h_grads.col(0).dot(_lC_d_grads.col(0)) +
                               inv_b2*_lC_h_grads.col(1).dot(_lC_d_grads.col(1)) +
                               inv_b3*_lC_h_grads.col(2).dot(_lC_d_grads.col(2)) +
                               inv_b4*_lC_h_grads.col(3).dot(_lC_d_grads.col(3));
            const double a21 = a12;
            const double a22 = inv_b1*_lC_d_grads.col(0).squaredNorm() + 
                               inv_b2*_lC_d_grads.col(1).squaredNorm() + 
                               inv_b3*_lC_d_grads.col(2).squaredNorm() + 
                               inv_b4*_lC_d_grads.col(3).squaredNorm() + 
                               alpha_d_tilde;
            const double k1 = -C_h - alpha_h_tilde * lambda_hs(i);
            const double k2 = -C_d - alpha_d_tilde * lambda_ds(i);

            const double detA = a11*a22 - a21*a12;

            const double dlam_h = (k1*a22 - k2*a12) / detA;
            const double dlam_d = (a11*k2 - a21*k1) / detA;

            // update Lagrange multipliers
            lambda_hs(i) += dlam_h;
            lambda_ds(i) += dlam_d;


            // update nodal forces (delC*lambda)
            nodal_forces.row(elem(0)) += inv_b1 * (_lC_h_grads.col(0) * lambda_hs(i) + _lC_d_grads.col(0) * lambda_ds(i));
            nodal_forces.row(elem(1)) += inv_b2 * (_lC_h_grads.col(1) * lambda_hs(i) + _lC_d_grads.col(1) * lambda_ds(i));
            nodal_forces.row(elem(2)) += inv_b3 * (_lC_h_grads.col(2) * lambda_hs(i) + _lC_d_grads.col(2) * lambda_ds(i));
            nodal_forces.row(elem(3)) += inv_b4 * (_lC_h_grads.col(3) * lambda_hs(i) + _lC_d_grads.col(3) * lambda_ds(i));
        }

        _vertices = inertial_positions + nodal_forces;
        
        // if the residual policy is to calculate every iteration, do that
        if (_residual_policy == FirstOrderXPBDResidualPolicy::EVERY_ITERATION)
        {
            if (const OutputSimulation* output_sim = dynamic_cast<const OutputSimulation*>(_sim))
            {
                // _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
                // if we are calculating the residuals every Gauss Seidel iteration, it's likely we want to create
                // a residual vs. iteration number plot, so write info to file
                // output_sim->printInfo();
            }
        }
    }

    // if the residual policy is to calculate every substep, do that
    if (_residual_policy == FirstOrderXPBDResidualPolicy::EVERY_SUBSTEP)
    {
        // _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
    }
}

inline void FirstOrderXPBDMeshObject::_computeF(const unsigned elem_index, Eigen::Matrix3d& X, Eigen::Matrix3d& F)
{
    // create the deformed shape matrix from current deformed vertex positions
    X.col(0) = _vertices.row(_elements(elem_index,0)) - _vertices.row(_elements(elem_index,3));
    X.col(1) = _vertices.row(_elements(elem_index,1)) - _vertices.row(_elements(elem_index,3));
    X.col(2) = _vertices.row(_elements(elem_index,2)) - _vertices.row(_elements(elem_index,3));

    // compute F
    F = X * _Q[elem_index];
}

inline double FirstOrderXPBDMeshObject::_computeDeviatoricConstraint(const Eigen::Matrix3d& F, const Eigen::Matrix3d& Q, Eigen::Matrix<double, 3, 4>& C_d_grads)
{
    // compute constraint itself    
    const double C_d = std::sqrt(F.col(0).squaredNorm() + F.col(1).squaredNorm() + F.col(2).squaredNorm());

    // compute constraint gradient
    C_d_grads.block<3,3>(0,0) = 1/C_d * (F * Q.transpose());
    C_d_grads.col(3) = -C_d_grads.col(0) - C_d_grads.col(1) - C_d_grads.col(2);

    return C_d;
}

inline double FirstOrderXPBDMeshObject::_computeHydrostaticConstraint(const Eigen::Matrix3d& F, const Eigen::Matrix3d& Q, Eigen::Matrix3d& F_cross, Eigen::Matrix<double, 3, 4>& C_h_grads)
{
    // compute constraint itself
    const double C_h = F.determinant() - (1 + _material.mu()/_material.lambda());

    // compute constraint gradient
    F_cross.col(0) = F.col(1).cross(F.col(2));
    F_cross.col(1) = F.col(2).cross(F.col(0));
    F_cross.col(2) = F.col(0).cross(F.col(1));
    C_h_grads.block<3,3>(0,0) = F_cross * Q.transpose();
    C_h_grads.col(3) = -C_h_grads.col(0) - C_h_grads.col(1) - C_h_grads.col(2);

    return C_h;
}

void FirstOrderXPBDMeshObject::_calculateForces()
{
    Eigen::Matrix<double, -1, 3> forces = Eigen::MatrixXd::Zero(_vertices.rows(), 3);

    // define Eigen loop variables
    Eigen::Matrix3d X, F, F_cross, _lC_h_grads, _lC_d_grads;
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
        _lC_h_grads = F_cross * _Q[i].transpose();
        // compute the hydrostatic alpha
        const double alpha_h = 1/(_material.lambda() * _vols[i]);

        /** DEVIATORIC CONSTRAINT */
        // compute constraint itself
        const double C_d = std::sqrt(F.col(0).squaredNorm() + F.col(1).squaredNorm() + F.col(2).squaredNorm());

        // compute constraint gradient
        _lC_d_grads = 1/C_d * (F * _Q[i].transpose());
        // compute the deviatoric alpha
        const double alpha_d = 1/(_material.mu() * _vols[i]);

        // by definition, the gradient for vertex 4 in the element is the negative sum of other gradients
        C_h_grad_4 = -_lC_h_grads.col(0) - _lC_h_grads.col(1) - _lC_h_grads.col(2);
        C_d_grad_4 = -_lC_d_grads.col(0) - _lC_d_grads.col(1) - _lC_d_grads.col(2);

        // compute forces per vertex
        f_x1 = _lC_h_grads.col(0) / alpha_h * C_h + _lC_d_grads.col(0) / alpha_d * C_d;
        f_x2 = _lC_h_grads.col(1) / alpha_h * C_h + _lC_d_grads.col(1) / alpha_d * C_d;
        f_x3 = _lC_h_grads.col(2) / alpha_h * C_h + _lC_d_grads.col(2) / alpha_d * C_d;
        f_x4 = C_h_grad_4 / alpha_h * C_h + C_d_grad_4 / alpha_d * C_d;

        // compute forces
        forces.row(elem(0)) += f_x1;
        forces.row(elem(1)) += f_x2;
        forces.row(elem(2)) += f_x3;
        forces.row(elem(3)) += f_x4;
    }

    const double forces_abs_mean = forces.cwiseAbs().mean();
    const double forces_rms = std::sqrt(forces.squaredNorm()/forces.size());

    std::cout << name() << " Forces:" << std::endl;
    std::cout << "\tRMS:\t" << forces_rms << "\t\tAverage of abs coeffs:\t" << forces_abs_mean << std::endl;

}

void FirstOrderXPBDMeshObject::_projectCollisionConstraints(const double dt)
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

    // update vertex drivers
    for (const auto& driver : _vertex_drivers)
    {
        _vertices.row(driver->vertexIndex()) = driver->evaluate(_sim->time());
    }
}


void FirstOrderXPBDMeshObject::_updateVelocities(const double dt)
{
    // velocities are simply (cur_pos - last_pos) / deltaT
    _v = (_vertices - _x_prev) / dt;
    // set _x_prev to be ready for the next substep
    _x_prev = _vertices;
}