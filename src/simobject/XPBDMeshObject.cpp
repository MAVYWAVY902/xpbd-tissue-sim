#include "XPBDMeshObject.hpp"
#include "Simulation.hpp"
#include "OutputSimulation.hpp"

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

    _residual_policy = config->residualPolicy().value();

    _mass_to_damping_multiplier = config->massToDampingMultiplier().value();

    _g_scaling = config->gScaling().value();

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
    // initialize loop variables to zeroes
    _lX = Eigen::Matrix3d::Zero();
    _lF = Eigen::Matrix3d::Zero();
    _lF_cross = Eigen::Matrix3d::Zero();
    _lC_h_grads = Eigen::Matrix<double, 3, 4>::Zero();
    _lC_d_grads = Eigen::Matrix<double, 3, 4>::Zero();

    _lCA_inv = Eigen::VectorXd::Zero(9);
    _lB = Eigen::VectorXd::Zero(9);
    _lb = Eigen::VectorXd::Zero(10);
    _lA_invB = Eigen::VectorXd::Zero(9);
    _lM = Eigen::MatrixXd::Zero(10,10);
    _ldlam = Eigen::VectorXd::Zero(10);

    _initial_lambda_ds = Eigen::VectorXd::Zero(_elements.rows());
    _initial_lambda_hs = Eigen::VectorXd::Zero(_elements.rows());

    _primary_residual = VerticesMat::Zero(_vertices.rows(), 3);
    _dynamics_residual_rms = 0;
    _constraint_residual_rms = 0;
    _primary_residual_rms = 0;
    _vol_ratio = 1;
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

    if (_solve_mode == XPBDSolveMode::SEQUENTIAL_INIT_LAMBDA || _solve_mode == XPBDSolveMode::SIMULTANEOUS_INIT_LAMBDA)
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

    if (_solve_mode == XPBDSolveMode::SPLIT_DEVIATORIC_SIMULTANEOUS9 || _solve_mode == XPBDSolveMode::SPLIT_DEVIATORIC_SIMULTANEOUS10 ||
        _solve_mode == XPBDSolveMode::SPLIT_DEVIATORIC_SIMULTANEOUS9_G || _solve_mode == XPBDSolveMode::SPLIT_DEVIATORIC_SIMULTANEOUS10_G)
    {
        _A.resize(_elements.rows());
        _A_inv.resize(_elements.rows());
        Eigen::Matrix3d A;
        for (int i = 0; i < _elements.rows(); i++)
        {
            const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);

            const double inv_m1 = 1.0/_m[elem(0)];
            const double inv_m2 = 1.0/_m[elem(1)];
            const double inv_m3 = 1.0/_m[elem(2)];
            const double inv_m4 = 1.0/_m[elem(3)];

            const double alpha_d = 1/(_material.mu() * _vols[i]);

            A(0,0) =    inv_m1*_Q[i](0,0)*_Q[i](0,0) + 
                        inv_m2*_Q[i](1,0)*_Q[i](1,0) + 
                        inv_m3*_Q[i](2,0)*_Q[i](2,0) + 
                        inv_m4*(-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0))*(-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0)) + alpha_d/(_dt*_dt);
            
            A(0,1) =    inv_m1*_Q[i](0,0)*_Q[i](0,1) +
                        inv_m2*_Q[i](1,0)*_Q[i](1,1) +
                        inv_m3*_Q[i](2,0)*_Q[i](2,1) +
                        inv_m4*(-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0))*(-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1));
            
            A(0,2) =    inv_m1*_Q[i](0,0)*_Q[i](0,2) +
                        inv_m2*_Q[i](1,0)*_Q[i](1,2) + 
                        inv_m3*_Q[i](2,0)*_Q[i](2,2) +
                        inv_m4*(-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0))*(-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2));
            
            A(1,0) = A(0,1);
            
            A(1,1) =    inv_m1*_Q[i](0,1)*_Q[i](0,1) +
                        inv_m2*_Q[i](1,1)*_Q[i](1,1) +
                        inv_m3*_Q[i](2,1)*_Q[i](2,1) +
                        inv_m4*(-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1))*(-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1)) + alpha_d/(_dt*_dt);
            
            A(1,2) =    inv_m1*_Q[i](0,1)*_Q[i](0,2) +
                        inv_m2*_Q[i](1,1)*_Q[i](1,2) +
                        inv_m3*_Q[i](2,1)*_Q[i](2,2) +
                        inv_m4*(-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1))*(-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2));

            A(2,0) = A(0,2);
            A(2,1) = A(1,2);

            A(2,2) =    inv_m1*_Q[i](0,2)*_Q[i](0,2) +
                        inv_m2*_Q[i](1,2)*_Q[i](1,2) +
                        inv_m3*_Q[i](2,2)*_Q[i](2,2) +
                        inv_m4*(-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2))*(-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2)) + alpha_d/(_dt*_dt);

            _A.at(i) = A;
            _A_inv.at(i) = A.inverse(); 

        }
    }

    if (_solve_mode == XPBDSolveMode::SPLIT_DEVIATORIC_SIMULTANEOUS10_LLT)
    {
        _A.resize(_elements.rows());
        Eigen::Matrix3d A;
        for (int i = 0; i < _elements.rows(); i++)
        {
            const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);

            const double inv_m1 = 1.0/_m[elem(0)];
            const double inv_m2 = 1.0/_m[elem(1)];
            const double inv_m3 = 1.0/_m[elem(2)];
            const double inv_m4 = 1.0/_m[elem(3)];

            const double alpha_d = 1/(_material.mu() * _vols[i]);

            A(0,0) =    inv_m1*_Q[i](0,0)*_Q[i](0,0) + 
                        inv_m2*_Q[i](1,0)*_Q[i](1,0) + 
                        inv_m3*_Q[i](2,0)*_Q[i](2,0) + 
                        inv_m4*(-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0))*(-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0)) + alpha_d/(_dt*_dt);
            
            A(0,1) =    inv_m1*_Q[i](0,0)*_Q[i](0,1) +
                        inv_m2*_Q[i](1,0)*_Q[i](1,1) +
                        inv_m3*_Q[i](2,0)*_Q[i](2,1) +
                        inv_m4*(-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0))*(-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1));
            
            A(0,2) =    inv_m1*_Q[i](0,0)*_Q[i](0,2) +
                        inv_m2*_Q[i](1,0)*_Q[i](1,2) + 
                        inv_m3*_Q[i](2,0)*_Q[i](2,2) +
                        inv_m4*(-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0))*(-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2));
            
            A(1,0) = A(0,1);
            
            A(1,1) =    inv_m1*_Q[i](0,1)*_Q[i](0,1) +
                        inv_m2*_Q[i](1,1)*_Q[i](1,1) +
                        inv_m3*_Q[i](2,1)*_Q[i](2,1) +
                        inv_m4*(-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1))*(-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1)) + alpha_d/(_dt*_dt);
            
            A(1,2) =    inv_m1*_Q[i](0,1)*_Q[i](0,2) +
                        inv_m2*_Q[i](1,1)*_Q[i](1,2) +
                        inv_m3*_Q[i](2,1)*_Q[i](2,2) +
                        inv_m4*(-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1))*(-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2));

            A(2,0) = A(0,2);
            A(2,1) = A(1,2);

            A(2,2) =    inv_m1*_Q[i](0,2)*_Q[i](0,2) +
                        inv_m2*_Q[i](1,2)*_Q[i](1,2) +
                        inv_m3*_Q[i](2,2)*_Q[i](2,2) +
                        inv_m4*(-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2))*(-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2)) + alpha_d/(_dt*_dt);


            _A.at(i) = A; 

        }
    }

    if (_solve_mode == XPBDSolveMode::TRUE_GAUSS_SEIDEL)
    {
        _neighbor_elements.resize(_elements.rows());
        // lazy O(n^2) loop looking for neighboring elements
        for (int i = 0; i < _elements.rows(); i++)
        {
            for (int j = 0; j < _elements.rows(); j++)
            {
                if (i == j) continue;
                // dumb check for shared vertices
                if (_elements(j,0) == _elements(i,0) ||
                    _elements(j,1) == _elements(i,0) ||
                    _elements(j,2) == _elements(i,0) ||
                    _elements(j,3) == _elements(i,0) ||
                    _elements(j,0) == _elements(i,1) ||
                    _elements(j,1) == _elements(i,1) ||
                    _elements(j,2) == _elements(i,1) ||
                    _elements(j,3) == _elements(i,1) ||
                    _elements(j,0) == _elements(i,2) ||
                    _elements(j,1) == _elements(i,2) ||
                    _elements(j,2) == _elements(i,2) ||
                    _elements(j,3) == _elements(i,2) ||
                    _elements(j,0) == _elements(i,3) ||
                    _elements(j,1) == _elements(i,3) ||
                    _elements(j,2) == _elements(i,3) ||
                    _elements(j,3) == _elements(i,3))
                {
                    _neighbor_elements[i].push_back(j);
                }
            }
        }

        double avg_mesh_connectivity = 0;
        for (const auto& neighbors : _neighbor_elements)
        {
            avg_mesh_connectivity += neighbors.size();
        }
        avg_mesh_connectivity /= _elements.rows();
        std::cout << "Avg mesh connectivity: " << avg_mesh_connectivity << std::endl;

        for (const auto& e : _neighbor_elements[0])
        {
            std::cout << e << ", ";
        }
        std::cout << std::endl;
    }

    if (_solve_mode == XPBDSolveMode::SEQUENTIAL_DISTRIBUTED_G || _solve_mode == XPBDSolveMode::SIMULTANEOUS_DISTRIBUTED_G || _solve_mode == XPBDSolveMode::SIMULTANEOUS_DISTRIBUTED_G_INIT_LAMBDA ||
        _solve_mode == XPBDSolveMode::SIMULTANEOUS_JACOBI || _solve_mode == XPBDSolveMode::SIMULTANEOUS_CONVERGENT_JACOBI)
    {
        _num_elements_with_position = Eigen::VectorXd::Zero(_vertices.rows());
        for (int i = 0; i < _elements.rows(); i++)
        {
            _num_elements_with_position(_elements(i,0))++;
            _num_elements_with_position(_elements(i,1))++;
            _num_elements_with_position(_elements(i,2))++;
            _num_elements_with_position(_elements(i,3))++;
        }
        std::cout << "Average num elements per node: " << _num_elements_with_position.mean() << "\tMin: " << _num_elements_with_position.minCoeff() << "\tMax: " << _num_elements_with_position.maxCoeff() << std::endl;
    }

    if (_solve_mode == XPBDSolveMode::SIMULTANEOUS_DISTRIBUTED_G_ORDERED)
    {
        _num_elements_with_position = Eigen::VectorXd::Zero(_vertices.rows());
        for (int i = 0; i < _elements.rows(); i++)
        {
            _num_elements_with_position(_elements(i,0))++;
            _num_elements_with_position(_elements(i,1))++;
            _num_elements_with_position(_elements(i,2))++;
            _num_elements_with_position(_elements(i,3))++;
        }

        _element_order.resize(_elements.rows());
        std::iota(_element_order.begin(), _element_order.end(), 0);
        std::vector<int> element_connectivity(_elements.rows());
        for (int i = 0; i < _elements.rows(); i++)
        {
            element_connectivity.at(i) = _num_elements_with_position(_elements(i,0)) +
                                         _num_elements_with_position(_elements(i,1)) +
                                         _num_elements_with_position(_elements(i,2)) +
                                         _num_elements_with_position(_elements(i,3));
        }
        // sort from least to greatest
        std::sort(_element_order.begin(), _element_order.end(),
            [&](int i, int j) -> bool {
                return element_connectivity[i] < element_connectivity[j]; 
            });

        // sort from greatest to least
        // std::sort(_element_order.begin(), _element_order.end(),
        //     [&](int i, int j) -> bool {
        //         return element_connectivity[i] > element_connectivity[j]; 
        //     });

        std::cout << "After sorting: first element in indices list is " << _element_order[0] << " which has connectivity " << element_connectivity[_element_order[0]] << std::endl;
        std::cout << "\t last element in indices list is " << _element_order.back() << " which has connectivity " << element_connectivity[_element_order.back()] << std::endl;

    }


    double max_q = 0;
    for (int i = 0; i < _elements.rows(); i++)
    {
        double q = _Q.at(i).maxCoeff();
        if (q > max_q)
            max_q = q;
    }
    std::cout << "Max of all Q: " << max_q << std::endl;

    std::cout << "Precomputed quantities" << std::endl;

    if (_solve_mode == XPBDSolveMode::SPLIT_DEVIATORIC_SEQUENTIAL)
        _calculateForcesSplitDeviatoric();
    else
        _calculateForces();
    // _calculateForces();
}

std::string XPBDMeshObject::toString() const
{
    return ElasticMeshObject::toString() + "\n\tSolve mode: " + solveMode() +
        "\n\tNum solver iterations: " + std::to_string(_num_iters) + "\n\tDamping stiffness: " + std::to_string(_damping_stiffness);
}

void XPBDMeshObject::update(const double dt, const double g_accel)
{
    // auto t1 = std::chrono::steady_clock::now();
    _movePositionsIntertially(dt, g_accel);
    // auto t2 = std::chrono::steady_clock::now();
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
    else if (_solve_mode == XPBDSolveMode::SIMULTANEOUS_INIT_LAMBDA)
        _projectConstraintsSimultaneousInitLambda(dt);
    else if (_solve_mode == XPBDSolveMode::RUCKER_FULL)
        _projectConstraintsRuckerFull(dt);
    else if (_solve_mode == XPBDSolveMode::SPLIT_DEVIATORIC_SEQUENTIAL)
        _projectConstraintsSplitDeviatoricSequential(dt);
    else if (_solve_mode == XPBDSolveMode::SPLIT_DEVIATORIC_SIMULTANEOUS9)
        _projectConstraintsSplitDeviatoricSimultaneous9(dt);
    else if (_solve_mode == XPBDSolveMode::SPLIT_DEVIATORIC_SIMULTANEOUS10)
        _projectConstraintsSplitDeviatoricSimultaneous10(dt);
    else if (_solve_mode == XPBDSolveMode::SPLIT_DEVIATORIC_SIMULTANEOUS10_LLT)
        _projectConstraintsSplitDeviatoricSimultaneous10LLT(dt);
    else if (_solve_mode == XPBDSolveMode::FIRST_ORDER_SIMULTANEOUS)
        _projectConstraintsFirstOrderSimultaneous(dt);
    else if (_solve_mode == XPBDSolveMode::SEQUENTIAL_G)
        _projectConstraintsSequentialG(dt);
    else if (_solve_mode == XPBDSolveMode::SIMULTANEOUS_G)
        _projectConstraintsSimultaneousG(dt);
    else if (_solve_mode == XPBDSolveMode::SPLIT_DEVIATORIC_SIMULTANEOUS9_G)
        _projectConstraintsSplitDeviatoricSimultaneous9G(dt);
    else if (_solve_mode == XPBDSolveMode::SPLIT_DEVIATORIC_SIMULTANEOUS10_G)
        _projectConstraintsSplitDeviatoricSimultaneous10G(dt);
    else if (_solve_mode == XPBDSolveMode::TRUE_GAUSS_SEIDEL)
        _projectConstraintsTrueGaussSeidel(dt);
    else if (_solve_mode == XPBDSolveMode::SEQUENTIAL_DISTRIBUTED_G)
        _projectConstraintsSequentialDistributedG(dt);
    else if (_solve_mode == XPBDSolveMode::SIMULTANEOUS_DISTRIBUTED_G)
        _projectConstraintsSimultaneousDistributedG(dt);
    else if (_solve_mode == XPBDSolveMode::SIMULTANEOUS_DISTRIBUTED_G_INIT_LAMBDA)
        _projectConstraintsSimultaneousDistributedGInitLambda(dt);
    else if (_solve_mode == XPBDSolveMode::SIMULTANEOUS_DAMPED_NEWTON)
        _projectConstraintsSimultaneousDampedNewton(dt);
    else if (_solve_mode == XPBDSolveMode::SIMULTANEOUS_DISTRIBUTED_G_ORDERED)
        _projectConstraintsSimultaneousDistributedGOrdered(dt);
    else if (_solve_mode == XPBDSolveMode::SIMULTANEOUS_JACOBI)
        _projectConstraintsSimultaneousJacobi(dt);
    else if (_solve_mode == XPBDSolveMode::SIMULTANEOUS_CONVERGENT_JACOBI)
        _projectConstraintsSimultaneousConvergentJacobi(dt);
    else if (_solve_mode == XPBDSolveMode::SIMULTANEOUS_CONVERGENT_JACOBI_LINE_SEARCH)
        _projectConstraintsSimultaneousConvergentJacobiLineSearch(dt);

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

std::string XPBDMeshObject::solveMode() const
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
    if (_solve_mode == XPBDSolveMode::SIMULTANEOUS_INIT_LAMBDA)
        return "Simultaneous-Init-Lambda";
    if (_solve_mode == XPBDSolveMode::SPLIT_DEVIATORIC_SEQUENTIAL)
        return "Split-Deviatoric-Sequential";
    if (_solve_mode == XPBDSolveMode::SPLIT_DEVIATORIC_SIMULTANEOUS9)
        return "Split-Deviatoric-Simultaneous9";
    if (_solve_mode == XPBDSolveMode::SPLIT_DEVIATORIC_SIMULTANEOUS10)
        return "Split-Deviatoric-Simultaneous10";
    if (_solve_mode == XPBDSolveMode::SPLIT_DEVIATORIC_SIMULTANEOUS10_LLT)
        return "Split-Deviatoric-Simultaneous10-LLT";
    if (_solve_mode == XPBDSolveMode::FIRST_ORDER_SIMULTANEOUS)
        return "First-Order-Simultaneous";
    if (_solve_mode == XPBDSolveMode::SEQUENTIAL_G)
        return "Sequential-g";
    if (_solve_mode == XPBDSolveMode::SIMULTANEOUS_G)
        return "Simultaneous-g";
    if (_solve_mode == XPBDSolveMode::SPLIT_DEVIATORIC_SIMULTANEOUS9_G)
        return "Split-Deviatoric-Simultaneous9-g";
    if (_solve_mode == XPBDSolveMode::SPLIT_DEVIATORIC_SIMULTANEOUS10_G)
        return "Split-Deviatoric-Simultaneous10-g";
    if (_solve_mode == XPBDSolveMode::TRUE_GAUSS_SEIDEL)
        return "True-Gauss-Seidel";
    if (_solve_mode == XPBDSolveMode::SEQUENTIAL_DISTRIBUTED_G)
        return "Sequential-Distributed-g";
    if (_solve_mode == XPBDSolveMode::SIMULTANEOUS_DISTRIBUTED_G)
        return "Simultaneous-Distributed-g";
    if (_solve_mode == XPBDSolveMode::SIMULTANEOUS_DISTRIBUTED_G_INIT_LAMBDA)
        return "Simultaneous-Distributed-g-Init-Lambda";
    if (_solve_mode == XPBDSolveMode::SIMULTANEOUS_DAMPED_NEWTON)
        return "Simultaneous-Damped-Newton";
    if (_solve_mode == XPBDSolveMode::SIMULTANEOUS_DISTRIBUTED_G_ORDERED)
        return "Simultaneous-Distributed-g-Ordered";
    if (_solve_mode == XPBDSolveMode::SIMULTANEOUS_JACOBI)
        return "Simultaneous-Jacobi";
    if (_solve_mode == XPBDSolveMode::SIMULTANEOUS_CONVERGENT_JACOBI)
        return "Simultaneous-Convergent-Jacobi";
    if (_solve_mode == XPBDSolveMode::SIMULTANEOUS_CONVERGENT_JACOBI_LINE_SEARCH)
        return "Simultaneous-Jacobi-Line-Search";
    
    return "";
}

void XPBDMeshObject::_movePositionsIntertially(const double dt, const double g_accel)
{
    if (_solve_mode == XPBDSolveMode::FIRST_ORDER_SIMULTANEOUS)
    {
        for (int i = 0; i < _vertices.rows(); i++)
        {
            _vertices(i,2) += -g_accel * dt / _mass_to_damping_multiplier;
        }   
        
    }
    else
    {
        // move vertices according to their velocity
        _vertices += dt*_v;
        // external forces (right now just gravity, which acts in -z direction)
        for (int i = 0; i < _vertices.rows(); i++)
        {
            _vertices(i,2) += -g_accel * dt * dt;
        }
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


    for (unsigned gi = 0; gi < _num_iters; gi++)
    {
        for (int i = 0; i < _elements.rows(); i++)
        {
            const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);
            // extract masses of each vertex in the current element
            const double inv_m1 = 1.0/_m[elem(0)];
            const double inv_m2 = 1.0/_m[elem(1)];
            const double inv_m3 = 1.0/_m[elem(2)];
            const double inv_m4 = 1.0/_m[elem(3)];

            /** DEVIATORIC CONSTRAINT */

            _computeF(i, _lX, _lF);
            const double C_d = _computeDeviatoricConstraint(_lF, _Q[i], _lC_d_grads);

            // compute the deviatoric alpha
            const double alpha_d = 1/(_material.mu() * _vols[i]);

            // const double beta_tilde = _damping_stiffness * (dt*dt);
            // const double gamma_d = alpha_d / (dt*dt) * beta_tilde / dt;
            
            // const double delC_x_prev_d =  _lC_d_grads.col(0).dot(_vertices.row(elem(0)) - _x_prev.row(elem(0))) +
            //                             _lC_d_grads.col(1).dot(_vertices.row(elem(1)) - _x_prev.row(elem(1))) +
            //                             _lC_d_grads.col(2).dot(_vertices.row(elem(2)) - _x_prev.row(elem(2))) +
            //                             _lC_d_grads.col(3).dot(_vertices.row(elem(3)) - _x_prev.row(elem(3)));

            // const double dlam_d = (-C_d - alpha_d * lambda_ds(i) / (dt*dt) - gamma_d * delC_x_prev_d) /
            //     ((1 + gamma_d) * (
            //         (1/m1)*_lC_d_grads.col(0).squaredNorm() + 
            //         (1/m2)*_lC_d_grads.col(1).squaredNorm() + 
            //         (1/m3)*_lC_d_grads.col(2).squaredNorm() + 
            //         (1/m4)*_lC_d_grads.col(3).squaredNorm()
            //      ) + alpha_d/(dt*dt)); 

            const double dlam_d = (-C_d - alpha_d * lambda_ds(i) / (dt*dt)) / ((
                    (inv_m1)*_lC_d_grads.col(0).squaredNorm() + 
                    (inv_m2)*_lC_d_grads.col(1).squaredNorm() + 
                    (inv_m3)*_lC_d_grads.col(2).squaredNorm() + 
                    (inv_m4)*_lC_d_grads.col(3).squaredNorm()
                 ) + alpha_d/(dt*dt));

            _vertices.row(elem(0)) += _lC_d_grads.col(0) * dlam_d * inv_m1;
            _vertices.row(elem(1)) += _lC_d_grads.col(1) * dlam_d * inv_m2;
            _vertices.row(elem(2)) += _lC_d_grads.col(2) * dlam_d * inv_m3;
            _vertices.row(elem(3)) += _lC_d_grads.col(3) * dlam_d * inv_m4;


            /** HYDROSTATIC CONSTRAINT */

            _computeF(i, _lX, _lF);
            const double C_h = _computeHydrostaticConstraint(_lF, _Q[i], _lF_cross, _lC_h_grads);
            // compute the hydrostatic alpha
            const double alpha_h = 1/(_material.lambda() * _vols[i]);
            
            // const double gamma_h = alpha_h / (dt*dt) * beta_tilde / dt;

            // const double delC_x_prev_h =  _lC_h_grads.col(0).dot(_vertices.row(elem(0)) - _x_prev.row(elem(0))) +
            //                             _lC_h_grads.col(1).dot(_vertices.row(elem(1)) - _x_prev.row(elem(1))) +
            //                             _lC_h_grads.col(2).dot(_vertices.row(elem(2)) - _x_prev.row(elem(2))) +
            //                             _lC_h_grads.col(3).dot(_vertices.row(elem(3)) - _x_prev.row(elem(3)));

            // const double dlam_h = (-C_h - alpha_h * lambda_hs(i) / (dt*dt) - gamma_h * delC_x_prev_h) / 
            //     ((1 + gamma_h) * (
            //         (1/m1)*_lC_h_grads.col(0).squaredNorm() + 
            //         (1/m2)*_lC_h_grads.col(1).squaredNorm() + 
            //         (1/m3)*_lC_h_grads.col(2).squaredNorm() + 
            //         (1/m4)*_lC_h_grads.col(3).squaredNorm()
            //      ) + alpha_h/(dt*dt));
            
            const double dlam_h = (-C_h - alpha_h * lambda_hs(i) / (dt*dt)) / ((
                    (inv_m1)*_lC_h_grads.col(0).squaredNorm() + 
                    (inv_m2)*_lC_h_grads.col(1).squaredNorm() + 
                    (inv_m3)*_lC_h_grads.col(2).squaredNorm() + 
                    (inv_m4)*_lC_h_grads.col(3).squaredNorm()
                 ) + alpha_h/(dt*dt));

            _vertices.row(elem(0)) += _lC_h_grads.col(0) * dlam_h * inv_m1;
            _vertices.row(elem(1)) += _lC_h_grads.col(1) * dlam_h * inv_m2;
            _vertices.row(elem(2)) += _lC_h_grads.col(2) * dlam_h * inv_m3;
            _vertices.row(elem(3)) += _lC_h_grads.col(3) * dlam_h * inv_m4;

            // update Lagrange multipliers
            lambda_hs(i) += dlam_h;
            lambda_ds(i) += dlam_d;
        }

        // if the residual policy is to calculate every iteration, do that
        if (_residual_policy == XPBDResidualPolicy::EVERY_ITERATION)
        {
            if (const OutputSimulation* output_sim = dynamic_cast<const OutputSimulation*>(_sim))
            {
                _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
                // if we are calculating the residuals every Gauss Seidel iteration, it's likely we want to create
                // a residual vs. iteration number plot, so write info to file
                output_sim->printInfo();
            }
        }
    }

    // if the residual policy is to calculate every substep, do that
    if (_residual_policy == XPBDResidualPolicy::EVERY_SUBSTEP)
    {
        _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
    }
    
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

            _computeF(i, _lX, _lF);
            const double C_d = _computeDeviatoricConstraint(_lF, _Q[i], _lC_d_grads);

            // compute the deviatoric alpha
            const double alpha_d = 1/(_material.mu() * _vols[i]);

            const double beta_tilde = _damping_stiffness * (dt*dt);
            const double gamma_d = alpha_d / (dt*dt) * beta_tilde / dt;
            
            // const double denom_d = ((1/m1)*_lC_d_grads.col(0).squaredNorm() + (1/m2)*_lC_d_grads.col(1).squaredNorm() + (1/m3)*_lC_d_grads.col(2).squaredNorm() + (1/m4)*C_d_grad_4.squaredNorm() + alpha_d/(dt*dt));
            const double delC_x_prev_d =  _lC_d_grads.col(0).dot(_vertices.row(elem(0)) - _x_prev.row(elem(0))) +
                                        _lC_d_grads.col(1).dot(_vertices.row(elem(1)) - _x_prev.row(elem(1))) +
                                        _lC_d_grads.col(2).dot(_vertices.row(elem(2)) - _x_prev.row(elem(2))) +
                                        _lC_d_grads.col(3).dot(_vertices.row(elem(3)) - _x_prev.row(elem(3)));

            const double dlam_d = (-C_d - alpha_d * lambda_ds(i) / (dt*dt) - gamma_d * delC_x_prev_d) /
                ((1 + gamma_d) * (
                    (1/m1)*_lC_d_grads.col(0).squaredNorm() + 
                    (1/m2)*_lC_d_grads.col(1).squaredNorm() + 
                    (1/m3)*_lC_d_grads.col(2).squaredNorm() + 
                    (1/m4)*_lC_d_grads.col(3).squaredNorm()
                 ) + alpha_d/(dt*dt)); 
                // std::cout << "C_h: " << C_h << "\talpha_h: " << alpha_h <<"\tdlam_h: " << dlam_h << std::endl;
                // std::cout << "C_d: " << C_d << "\talpha_d: " << alpha_d << "\tdenominator: " <<  denom_d << "\tdlam_d: " << dlam_d << std::endl;
            
            _vertices.row(elem(0)) += _lC_d_grads.col(0) * dlam_d/m1;
            _vertices.row(elem(1)) += _lC_d_grads.col(1) * dlam_d/m2;
            _vertices.row(elem(2)) += _lC_d_grads.col(2) * dlam_d/m3;
            _vertices.row(elem(3)) += _lC_d_grads.col(3) * dlam_d/m4;


            /** HYDROSTATIC CONSTRAINT */

            _computeF(i, _lX, _lF);
            const double C_h = _computeHydrostaticConstraint(_lF, _Q[i], _lF_cross, _lC_h_grads);
            // compute the hydrostatic alpha
            const double alpha_h = 1/(_material.lambda() * _vols[i]);
            
            const double gamma_h = alpha_h / (dt*dt) * beta_tilde / dt;

            const double delC_x_prev_h =  _lC_h_grads.col(0).dot(_vertices.row(elem(0)) - _x_prev.row(elem(0))) +
                                        _lC_h_grads.col(1).dot(_vertices.row(elem(1)) - _x_prev.row(elem(1))) +
                                        _lC_h_grads.col(2).dot(_vertices.row(elem(2)) - _x_prev.row(elem(2))) +
                                        _lC_h_grads.col(3).dot(_vertices.row(elem(3)) - _x_prev.row(elem(3)));

            const double dlam_h = (-C_h - alpha_h * lambda_hs(i) / (dt*dt) - gamma_h * delC_x_prev_h) / 
                ((1 + gamma_h) * (
                    (1/m1)*_lC_h_grads.col(0).squaredNorm() + 
                    (1/m2)*_lC_h_grads.col(1).squaredNorm() + 
                    (1/m3)*_lC_h_grads.col(2).squaredNorm() + 
                    (1/m4)*_lC_h_grads.col(3).squaredNorm()
                 ) + alpha_h/(dt*dt));

            _vertices.row(elem(0)) += _lC_h_grads.col(0) * dlam_h/m1;
            _vertices.row(elem(1)) += _lC_h_grads.col(1) * dlam_h/m2;
            _vertices.row(elem(2)) += _lC_h_grads.col(2) * dlam_h/m3;
            _vertices.row(elem(3)) += _lC_h_grads.col(3) * dlam_h/m4;

            // update Lagrange multipliers
            lambda_hs(i) += dlam_h;
            lambda_ds(i) += dlam_d;
        }

        // if the residual policy is to calculate every iteration, do that
        if (_residual_policy == XPBDResidualPolicy::EVERY_ITERATION)
        {
            if (const OutputSimulation* output_sim = dynamic_cast<const OutputSimulation*>(_sim))
            {
                _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
                // if we are calculating the residuals every Gauss Seidel iteration, it's likely we want to create
                // a residual vs. iteration number plot, so write info to file
                output_sim->printInfo();
            }
        }
    }

    // if the residual policy is to calculate every substep, do that
    if (_residual_policy == XPBDResidualPolicy::EVERY_SUBSTEP)
    {
        _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
    }
}

void XPBDMeshObject::_projectConstraintsSequentialG(const double dt)
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

    VerticesMat x_minus_x_tilde = _vertices - inertial_positions;


    for (unsigned gi = 0; gi < _num_iters; gi++)
    {
        x_minus_x_tilde = _vertices - inertial_positions;
        for (int i = 0; i < _elements.rows(); i++)
        {
            const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);
            // extract masses of each vertex in the current element
            const double inv_m1 = 1.0/_m[elem(0)];
            const double inv_m2 = 1.0/_m[elem(1)];
            const double inv_m3 = 1.0/_m[elem(2)];
            const double inv_m4 = 1.0/_m[elem(3)];


            /** DEVIATORIC CONSTRAINT */

            _computeF(i, _lX, _lF);
            const double C_d = _computeDeviatoricConstraint(_lF, _Q[i], _lC_d_grads);

            // compute the deviatoric alpha
            const double alpha_d = 1/(_material.mu() * _vols[i]);

            const double delC_Minv_delC_d = (inv_m1)*_lC_d_grads.col(0).squaredNorm() + 
                    (inv_m2)*_lC_d_grads.col(1).squaredNorm() + 
                    (inv_m3)*_lC_d_grads.col(2).squaredNorm() + 
                    (inv_m4)*_lC_d_grads.col(3).squaredNorm(); 
            const double delC_Minv_delC_alpha_d = delC_Minv_delC_d + alpha_d/(dt*dt);
            
            const double delC_x_d = _lC_d_grads.col(0).dot(x_minus_x_tilde.row(elem(0))) +
                                    _lC_d_grads.col(1).dot(x_minus_x_tilde.row(elem(1))) +
                                    _lC_d_grads.col(2).dot(x_minus_x_tilde.row(elem(2))) +
                                    _lC_d_grads.col(3).dot(x_minus_x_tilde.row(elem(3)));

            const double dlam_d = (-C_d - (_g_scaling * delC_Minv_delC_d + alpha_d/(dt*dt)) * lambda_ds(i) + _g_scaling * delC_x_d) / delC_Minv_delC_alpha_d;

            _vertices.row(elem(0)) += _lC_d_grads.col(0) * (dlam_d + lambda_ds(i)) * inv_m1 - _g_scaling * x_minus_x_tilde.row(elem(0)).transpose();
            _vertices.row(elem(1)) += _lC_d_grads.col(1) * (dlam_d + lambda_ds(i)) * inv_m2 - _g_scaling * x_minus_x_tilde.row(elem(1)).transpose();
            _vertices.row(elem(2)) += _lC_d_grads.col(2) * (dlam_d + lambda_ds(i)) * inv_m3 - _g_scaling * x_minus_x_tilde.row(elem(2)).transpose();
            _vertices.row(elem(3)) += _lC_d_grads.col(3) * (dlam_d + lambda_ds(i)) * inv_m4 - _g_scaling * x_minus_x_tilde.row(elem(3)).transpose();


            /** HYDROSTATIC CONSTRAINT */

            _computeF(i, _lX, _lF);
            const double C_h = _computeHydrostaticConstraint(_lF, _Q[i], _lF_cross, _lC_h_grads);
            // compute the hydrostatic alpha
            const double alpha_h = 1/(_material.lambda() * _vols[i]);
            
            const double delC_Minv_delC_h = (inv_m1)*_lC_h_grads.col(0).squaredNorm() + 
                    (inv_m2)*_lC_h_grads.col(1).squaredNorm() + 
                    (inv_m3)*_lC_h_grads.col(2).squaredNorm() + 
                    (inv_m4)*_lC_h_grads.col(3).squaredNorm();
            const double delC_Minv_delC_alpha_h = delC_Minv_delC_h + alpha_h/(dt*dt);

            const double delC_x_h = _lC_h_grads.col(0).dot(x_minus_x_tilde.row(elem(0))) +
                                    _lC_h_grads.col(1).dot(x_minus_x_tilde.row(elem(1))) +
                                    _lC_h_grads.col(2).dot(x_minus_x_tilde.row(elem(2))) +
                                    _lC_h_grads.col(3).dot(x_minus_x_tilde.row(elem(3)));

            const double dlam_h = (-C_h - (_g_scaling*delC_Minv_delC_h + alpha_h/(dt*dt)) * lambda_hs(i) + _g_scaling * delC_x_h) / delC_Minv_delC_alpha_h;
                    

            _vertices.row(elem(0)) += _lC_h_grads.col(0) * (dlam_h + _g_scaling * lambda_hs(i)) * inv_m1 - _g_scaling * x_minus_x_tilde.row(elem(0)).transpose();
            _vertices.row(elem(1)) += _lC_h_grads.col(1) * (dlam_h + _g_scaling * lambda_hs(i)) * inv_m2 - _g_scaling * x_minus_x_tilde.row(elem(1)).transpose();
            _vertices.row(elem(2)) += _lC_h_grads.col(2) * (dlam_h + _g_scaling * lambda_hs(i)) * inv_m3 - _g_scaling * x_minus_x_tilde.row(elem(2)).transpose();
            _vertices.row(elem(3)) += _lC_h_grads.col(3) * (dlam_h + _g_scaling * lambda_hs(i)) * inv_m4 - _g_scaling * x_minus_x_tilde.row(elem(3)).transpose();

            // update Lagrange multipliers
            lambda_hs(i) += dlam_h;
            lambda_ds(i) += dlam_d;
        }

        // if the residual policy is to calculate every iteration, do that
        if (_residual_policy == XPBDResidualPolicy::EVERY_ITERATION)
        {
            if (const OutputSimulation* output_sim = dynamic_cast<const OutputSimulation*>(_sim))
            {
                _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
                // if we are calculating the residuals every Gauss Seidel iteration, it's likely we want to create
                // a residual vs. iteration number plot, so write info to file
                output_sim->printInfo();
            }
        }
    }

    // if the residual policy is to calculate every substep, do that
    if (_residual_policy == XPBDResidualPolicy::EVERY_SUBSTEP)
    {
        _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
    }
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

            _computeF(i, _lX, _lF);

            /** DEVIATORIC CONSTRAINT */
            const double C_d = _computeDeviatoricConstraint(_lF, _Q[i], _lC_d_grads);

            // compute the deviatoric alpha
            const double alpha_d = 1/(_material.mu() * _vols[i]);

            /** HYDROSTATIC CONSTRAINT */

            const double C_h = _computeHydrostaticConstraint(_lF, _Q[i], _lF_cross, _lC_h_grads);

            // compute the hydrostatic alpha
            const double alpha_h = 1/(_material.lambda() * _vols[i]);


            const double inv_m1 = 1/m1;
            const double inv_m2 = 1/m2;
            const double inv_m3 = 1/m3;
            const double inv_m4 = 1/m4;

            // solve the 2x2 system
            const double a11 = inv_m1*_lC_h_grads.col(0).squaredNorm() + 
                               inv_m2*_lC_h_grads.col(1).squaredNorm() + 
                               inv_m3*_lC_h_grads.col(2).squaredNorm() + 
                               inv_m4*_lC_h_grads.col(3).squaredNorm() + 
                               alpha_h/(dt*dt);
            const double a12 = inv_m1*_lC_h_grads.col(0).dot(_lC_d_grads.col(0)) +
                               inv_m2*_lC_h_grads.col(1).dot(_lC_d_grads.col(1)) +
                               inv_m3*_lC_h_grads.col(2).dot(_lC_d_grads.col(2)) +
                               inv_m4*_lC_h_grads.col(3).dot(_lC_d_grads.col(3));
            const double a21 = a12;
            const double a22 = inv_m1*_lC_d_grads.col(0).squaredNorm() + 
                               inv_m2*_lC_d_grads.col(1).squaredNorm() + 
                               inv_m3*_lC_d_grads.col(2).squaredNorm() + 
                               inv_m4*_lC_d_grads.col(3).squaredNorm() + 
                               alpha_d/(dt*dt);
            const double k1 = -C_h - alpha_h / (dt*dt) * lambda_hs(i);
            const double k2 = -C_d - alpha_d / (dt*dt) * lambda_ds(i);

            const double detA = a11*a22 - a21*a12;

            const double dlam_h = (k1*a22 - k2*a12) / detA;
            const double dlam_d = (a11*k2 - a21*k1) / detA;


            // update vertex positions
            _vertices.row(elem(0)) += _lC_h_grads.col(0) * dlam_h * inv_m1 + _lC_d_grads.col(0) * dlam_d * inv_m1;
            _vertices.row(elem(1)) += _lC_h_grads.col(1) * dlam_h * inv_m2 + _lC_d_grads.col(1) * dlam_d * inv_m2;
            _vertices.row(elem(2)) += _lC_h_grads.col(2) * dlam_h * inv_m3 + _lC_d_grads.col(2) * dlam_d * inv_m3;
            _vertices.row(elem(3)) += _lC_h_grads.col(3) * dlam_h * inv_m4 + _lC_d_grads.col(3) * dlam_d * inv_m4;
            

            // update Lagrange multipliers
            lambda_hs(i) += dlam_h;
            lambda_ds(i) += dlam_d;
        }
        
        // if the residual policy is to calculate every iteration, do that
        if (_residual_policy == XPBDResidualPolicy::EVERY_ITERATION)
        {
            if (const OutputSimulation* output_sim = dynamic_cast<const OutputSimulation*>(_sim))
            {
                _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
                // if we are calculating the residuals every Gauss Seidel iteration, it's likely we want to create
                // a residual vs. iteration number plot, so write info to file
                output_sim->printInfo();
            }
        }
    }

    // if the residual policy is to calculate every substep, do that
    if (_residual_policy == XPBDResidualPolicy::EVERY_SUBSTEP)
    {
        _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
    }

}

void XPBDMeshObject::_projectConstraintsSimultaneousG(const double dt)
{
    // accumulated hydrostatic Lagrange multipliers
    Eigen::VectorXd lambda_hs = Eigen::VectorXd::Zero(_elements.rows());
    // accumulated deviatoric Lagrange multipliers
    Eigen::VectorXd lambda_ds = Eigen::VectorXd::Zero(_elements.rows());

    // store positions before constraints are projected
    // this is x-tilde, seen in XPBD eqn 8 - used for computing primary residual
    VerticesMat inertial_positions = _vertices;

    VerticesMat x_minus_x_tilde = _vertices - inertial_positions;

    for (unsigned gi = 0; gi < _num_iters; gi++)
    {
        x_minus_x_tilde = _vertices - inertial_positions;
        for (int i = 0; i < _elements.rows(); i++)
        {
            const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);

            // extract masses of each vertex in the current element
            const double m1 = _m[elem(0)];
            const double m2 = _m[elem(1)];
            const double m3 = _m[elem(2)];
            const double m4 = _m[elem(3)];

            _computeF(i, _lX, _lF);

            /** DEVIATORIC CONSTRAINT */
            const double C_d = _computeDeviatoricConstraint(_lF, _Q[i], _lC_d_grads);

            // compute the deviatoric alpha
            const double alpha_d = 1/(_material.mu() * _vols[i]);

            /** HYDROSTATIC CONSTRAINT */

            const double C_h = _computeHydrostaticConstraint(_lF, _Q[i], _lF_cross, _lC_h_grads);

            // compute the hydrostatic alpha
            const double alpha_h = 1/(_material.lambda() * _vols[i]);


            const double inv_m1 = 1/m1;
            const double inv_m2 = 1/m2;
            const double inv_m3 = 1/m3;
            const double inv_m4 = 1/m4;

            // const Eigen::Vector3d x_minus_x_tilde1 = _vertices.row(elem(0)) - inertial_positions.row(elem(0));
            // const Eigen::Vector3d x_minus_x_tilde2 = _vertices.row(elem(1)) - inertial_positions.row(elem(1));
            // const Eigen::Vector3d x_minus_x_tilde3 = _vertices.row(elem(2)) - inertial_positions.row(elem(2));
            // const Eigen::Vector3d x_minus_x_tilde4 = _vertices.row(elem(3)) - inertial_positions.row(elem(3));

            // solve the 2x2 system
            const double delC_Minv_delC_h = inv_m1*_lC_h_grads.col(0).squaredNorm() + 
                               inv_m2*_lC_h_grads.col(1).squaredNorm() + 
                               inv_m3*_lC_h_grads.col(2).squaredNorm() + 
                               inv_m4*_lC_h_grads.col(3).squaredNorm();

            const double delC_Minv_delC_d = inv_m1*_lC_d_grads.col(0).squaredNorm() + 
                               inv_m2*_lC_d_grads.col(1).squaredNorm() + 
                               inv_m3*_lC_d_grads.col(2).squaredNorm() + 
                               inv_m4*_lC_d_grads.col(3).squaredNorm();

            const double a11 = delC_Minv_delC_h + 
                               alpha_h/(dt*dt);
            const double a12 = inv_m1*_lC_h_grads.col(0).dot(_lC_d_grads.col(0)) +
                               inv_m2*_lC_h_grads.col(1).dot(_lC_d_grads.col(1)) +
                               inv_m3*_lC_h_grads.col(2).dot(_lC_d_grads.col(2)) +
                               inv_m4*_lC_h_grads.col(3).dot(_lC_d_grads.col(3));
            const double a21 = a12;
            const double a22 = delC_Minv_delC_d + 
                               alpha_d/(dt*dt);

            const double delC_x_d = _lC_d_grads.col(0).dot(x_minus_x_tilde.row(elem(0))) +
                                    _lC_d_grads.col(1).dot(x_minus_x_tilde.row(elem(1))) +
                                    _lC_d_grads.col(2).dot(x_minus_x_tilde.row(elem(2))) +
                                    _lC_d_grads.col(3).dot(x_minus_x_tilde.row(elem(3)));

            const double delC_x_h = _lC_h_grads.col(0).dot(x_minus_x_tilde.row(elem(0))) +
                                    _lC_h_grads.col(1).dot(x_minus_x_tilde.row(elem(1))) +
                                    _lC_h_grads.col(2).dot(x_minus_x_tilde.row(elem(2))) +
                                    _lC_h_grads.col(3).dot(x_minus_x_tilde.row(elem(3)));

            const double k1 = -C_h - ((_g_scaling*delC_Minv_delC_h + alpha_h/(dt*dt))*lambda_hs(i) + _g_scaling*a12*lambda_ds(i)) + _g_scaling*delC_x_h;
            const double k2 = -C_d - (_g_scaling*a21*lambda_hs(i) + (_g_scaling*delC_Minv_delC_d + alpha_d/(dt*dt))*lambda_ds(i)) + _g_scaling*delC_x_d;

            // const double k1 = -C_h - alpha_h/(dt*dt)*lambda_hs(i);
            // const double k2 = -C_d - alpha_d/(dt*dt)*lambda_ds(i);

            const double detA = a11*a22 - a21*a12;

            const double dlam_h = (k1*a22 - k2*a12) / detA;
            const double dlam_d = (a11*k2 - a21*k1) / detA;


            // update vertex positions
            _vertices.row(elem(0)) += _lC_h_grads.col(0) * (dlam_h + _g_scaling*lambda_hs(i)) * inv_m1 + _lC_d_grads.col(0) * (dlam_d + _g_scaling*lambda_ds(i)) * inv_m1 - _g_scaling*x_minus_x_tilde.row(elem(0)).transpose();
            _vertices.row(elem(1)) += _lC_h_grads.col(1) * (dlam_h + _g_scaling*lambda_hs(i)) * inv_m2 + _lC_d_grads.col(1) * (dlam_d + _g_scaling*lambda_ds(i)) * inv_m2 - _g_scaling*x_minus_x_tilde.row(elem(1)).transpose();
            _vertices.row(elem(2)) += _lC_h_grads.col(2) * (dlam_h + _g_scaling*lambda_hs(i)) * inv_m3 + _lC_d_grads.col(2) * (dlam_d + _g_scaling*lambda_ds(i)) * inv_m3 - _g_scaling*x_minus_x_tilde.row(elem(2)).transpose();
            _vertices.row(elem(3)) += _lC_h_grads.col(3) * (dlam_h + _g_scaling*lambda_hs(i)) * inv_m4 + _lC_d_grads.col(3) * (dlam_d + _g_scaling*lambda_ds(i)) * inv_m4 - _g_scaling*x_minus_x_tilde.row(elem(3)).transpose();
            

            // update Lagrange multipliers
            lambda_hs(i) += dlam_h;
            lambda_ds(i) += dlam_d;
        }
        
        // if the residual policy is to calculate every iteration, do that
        if (_residual_policy == XPBDResidualPolicy::EVERY_ITERATION)
        {
            if (const OutputSimulation* output_sim = dynamic_cast<const OutputSimulation*>(_sim))
            {
                _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
                // if we are calculating the residuals every Gauss Seidel iteration, it's likely we want to create
                // a residual vs. iteration number plot, so write info to file
                output_sim->printInfo();
            }
        }
    }

    // if the residual policy is to calculate every substep, do that
    if (_residual_policy == XPBDResidualPolicy::EVERY_SUBSTEP)
    {
        _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
    }
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

    for (int i = 0; i < _elements.rows(); i++)
    {
        const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);

        _computeF(i, _lX, _lF);

        /** DEVIATORIC CONSTRAINT */
        const double C_d = _computeDeviatoricConstraint(_lF, _Q[i], _lC_d_grads);

        // compute the deviatoric alpha
        const double alpha_d = 1/(_material.mu() * _vols[i]);

        /** HYDROSTATIC CONSTRAINT */

        const double C_h = _computeHydrostaticConstraint(_lF, _Q[i], _lF_cross, _lC_h_grads);

        // compute the hydrostatic alpha
        const double alpha_h = 1/(_material.lambda() * _vols[i]);

        // populate C
        C(2*i) = C_h;
        C(2*i+1) = C_d;


        // populate delC
        delC(2*i, Eigen::seq(0,2)) = _lC_h_grads.col(0);
        delC(2*i, Eigen::seq(3,5)) = _lC_h_grads.col(1);
        delC(2*i, Eigen::seq(6,8)) = _lC_h_grads.col(2);
        delC(2*i, Eigen::seq(9,11)) = _lC_h_grads.col(3);
        delC(2*i+1, Eigen::seq(0,2)) = _lC_d_grads.col(0);
        delC(2*i+1, Eigen::seq(3,5)) = _lC_d_grads.col(1);
        delC(2*i+1, Eigen::seq(6,8)) = _lC_d_grads.col(2);
        delC(2*i+1, Eigen::seq(9,11)) = _lC_d_grads.col(3);


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

    // if the residual policy is to calculate every substep, do that
    if (_residual_policy != XPBDResidualPolicy::NEVER)
    {
        _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
    }
}

void XPBDMeshObject::_projectConstraintsSequentialInitLambda(const double dt)
{
    // accumulated hydrostatic Lagrange multipliers
    Eigen::VectorXd lambda_hs = _initial_lambda_hs*(dt*dt);;
    // accumulated deviatoric Lagrange multipliers
    Eigen::VectorXd lambda_ds = _initial_lambda_ds*(dt*dt);

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
            const double m1 = _m[elem(0)];
            const double m2 = _m[elem(1)];
            const double m3 = _m[elem(2)];
            const double m4 = _m[elem(3)];

            /** DEVIATORIC CONSTRAINT */

            _computeF(i, _lX, _lF);
            const double C_d = _computeDeviatoricConstraint(_lF, _Q[i], _lC_d_grads);

            // compute the deviatoric alpha
            const double alpha_d = 1/(_material.mu() * _vols[i]);

            const double beta_tilde = _damping_stiffness * (dt*dt);
            const double gamma_d = alpha_d / (dt*dt) * beta_tilde / dt;
            
            // const double denom_d = ((1/m1)*_lC_d_grads.col(0).squaredNorm() + (1/m2)*_lC_d_grads.col(1).squaredNorm() + (1/m3)*_lC_d_grads.col(2).squaredNorm() + (1/m4)*C_d_grad_4.squaredNorm() + alpha_d/(dt*dt));
            const double delC_x_prev_d =  _lC_d_grads.col(0).dot(_vertices.row(elem(0)) - _x_prev.row(elem(0))) +
                                        _lC_d_grads.col(1).dot(_vertices.row(elem(1)) - _x_prev.row(elem(1))) +
                                        _lC_d_grads.col(2).dot(_vertices.row(elem(2)) - _x_prev.row(elem(2))) +
                                        _lC_d_grads.col(3).dot(_vertices.row(elem(3)) - _x_prev.row(elem(3)));

            const double dlam_d = (-C_d - alpha_d * lambda_ds(i) / (dt*dt) - gamma_d * delC_x_prev_d) /
                ((1 + gamma_d) * (
                    (1/m1)*_lC_d_grads.col(0).squaredNorm() + 
                    (1/m2)*_lC_d_grads.col(1).squaredNorm() + 
                    (1/m3)*_lC_d_grads.col(2).squaredNorm() + 
                    (1/m4)*_lC_d_grads.col(3).squaredNorm()
                 ) + alpha_d/(dt*dt)); 
                // std::cout << "C_h: " << C_h << "\talpha_h: " << alpha_h <<"\tdlam_h: " << dlam_h << std::endl;
                // std::cout << "C_d: " << C_d << "\talpha_d: " << alpha_d << "\tdenominator: " <<  denom_d << "\tdlam_d: " << dlam_d << std::endl;
            
            _vertices.row(elem(0)) += _lC_d_grads.col(0) * dlam_d/m1;
            _vertices.row(elem(1)) += _lC_d_grads.col(1) * dlam_d/m2;
            _vertices.row(elem(2)) += _lC_d_grads.col(2) * dlam_d/m3;
            _vertices.row(elem(3)) += _lC_d_grads.col(3) * dlam_d/m4;


            /** HYDROSTATIC CONSTRAINT */

            _computeF(i, _lX, _lF);
            const double C_h = _computeHydrostaticConstraint(_lF, _Q[i], _lF_cross, _lC_h_grads);
            // compute the hydrostatic alpha
            const double alpha_h = 1/(_material.lambda() * _vols[i]);
            
            const double gamma_h = alpha_h / (dt*dt) * beta_tilde / dt;

            const double delC_x_prev_h =  _lC_h_grads.col(0).dot(_vertices.row(elem(0)) - _x_prev.row(elem(0))) +
                                        _lC_h_grads.col(1).dot(_vertices.row(elem(1)) - _x_prev.row(elem(1))) +
                                        _lC_h_grads.col(2).dot(_vertices.row(elem(2)) - _x_prev.row(elem(2))) +
                                        _lC_h_grads.col(3).dot(_vertices.row(elem(3)) - _x_prev.row(elem(3)));

            const double dlam_h = (-C_h - alpha_h * lambda_hs(i) / (dt*dt) - gamma_h * delC_x_prev_h) / 
                ((1 + gamma_h) * (
                    (1/m1)*_lC_h_grads.col(0).squaredNorm() + 
                    (1/m2)*_lC_h_grads.col(1).squaredNorm() + 
                    (1/m3)*_lC_h_grads.col(2).squaredNorm() + 
                    (1/m4)*_lC_h_grads.col(3).squaredNorm()
                 ) + alpha_h/(dt*dt));

            _vertices.row(elem(0)) += _lC_h_grads.col(0) * dlam_h/m1;
            _vertices.row(elem(1)) += _lC_h_grads.col(1) * dlam_h/m2;
            _vertices.row(elem(2)) += _lC_h_grads.col(2) * dlam_h/m3;
            _vertices.row(elem(3)) += _lC_h_grads.col(3) * dlam_h/m4;

            // update Lagrange multipliers
            lambda_hs(i) += dlam_h;
            lambda_ds(i) += dlam_d;
        }

        // if the residual policy is to calculate every iteration, do that
        if (_residual_policy == XPBDResidualPolicy::EVERY_ITERATION)
        {
            if (const OutputSimulation* output_sim = dynamic_cast<const OutputSimulation*>(_sim))
            {
                _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
                // if we are calculating the residuals every Gauss Seidel iteration, it's likely we want to create
                // a residual vs. iteration number plot, so write info to file
                output_sim->printInfo();
            }
        }
    }

    // if the residual policy is to calculate every substep, do that
    if (_residual_policy == XPBDResidualPolicy::EVERY_SUBSTEP)
    {
        _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
    }
}

void XPBDMeshObject::_projectConstraintsSimultaneousInitLambda(const double dt)
{
    // accumulated hydrostatic Lagrange multipliers
    Eigen::VectorXd lambda_hs = _initial_lambda_hs*(dt*dt);//Eigen::VectorXd::Zero(_elements.rows());
    // accumulated deviatoric Lagrange multipliers
    Eigen::VectorXd lambda_ds = _initial_lambda_ds*(dt*dt);//Eigen::VectorXd::Zero(_elements.rows());

    // store positions before constraints are projected
    // this is x-tilde, seen in XPBD eqn 8 - used for computing primary residual
    VerticesMat inertial_positions = _vertices;

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

            _computeF(i, _lX, _lF);

            /** DEVIATORIC CONSTRAINT */
            const double C_d = _computeDeviatoricConstraint(_lF, _Q[i], _lC_d_grads);

            // compute the deviatoric alpha
            const double alpha_d = 1/(_material.mu() * _vols[i]);

            /** HYDROSTATIC CONSTRAINT */

            const double C_h = _computeHydrostaticConstraint(_lF, _Q[i], _lF_cross, _lC_h_grads);

            // compute the hydrostatic alpha
            const double alpha_h = 1/(_material.lambda() * _vols[i]);


            const double inv_m1 = 1/m1;
            const double inv_m2 = 1/m2;
            const double inv_m3 = 1/m3;
            const double inv_m4 = 1/m4;

            // solve the 2x2 system
            const double a11 = inv_m1*_lC_h_grads.col(0).squaredNorm() + 
                               inv_m2*_lC_h_grads.col(1).squaredNorm() + 
                               inv_m3*_lC_h_grads.col(2).squaredNorm() + 
                               inv_m4*_lC_h_grads.col(3).squaredNorm() + 
                               alpha_h/(dt*dt);
            const double a12 = inv_m1*_lC_h_grads.col(0).dot(_lC_d_grads.col(0)) +
                               inv_m2*_lC_h_grads.col(1).dot(_lC_d_grads.col(1)) +
                               inv_m3*_lC_h_grads.col(2).dot(_lC_d_grads.col(2)) +
                               inv_m4*_lC_h_grads.col(3).dot(_lC_d_grads.col(3));
            const double a21 = a12;
            const double a22 = inv_m1*_lC_d_grads.col(0).squaredNorm() + 
                               inv_m2*_lC_d_grads.col(1).squaredNorm() + 
                               inv_m3*_lC_d_grads.col(2).squaredNorm() + 
                               inv_m4*_lC_d_grads.col(3).squaredNorm() + 
                               alpha_d/(dt*dt);
            const double k1 = -C_h - alpha_h / (dt*dt) * lambda_hs(i);
            const double k2 = -C_d - alpha_d / (dt*dt) * lambda_ds(i);

            const double detA = a11*a22 - a21*a12;

            const double dlam_h = (k1*a22 - k2*a12) / detA;
            const double dlam_d = (a11*k2 - a21*k1) / detA;


            // update vertex positions
            _vertices.row(elem(0)) += _lC_h_grads.col(0) * dlam_h * inv_m1 + _lC_d_grads.col(0) * dlam_d * inv_m1;
            _vertices.row(elem(1)) += _lC_h_grads.col(1) * dlam_h * inv_m2 + _lC_d_grads.col(1) * dlam_d * inv_m2;
            _vertices.row(elem(2)) += _lC_h_grads.col(2) * dlam_h * inv_m3 + _lC_d_grads.col(2) * dlam_d * inv_m3;
            _vertices.row(elem(3)) += _lC_h_grads.col(3) * dlam_h * inv_m4 + _lC_d_grads.col(3) * dlam_d * inv_m4;
            

            // update Lagrange multipliers
            lambda_hs(i) += dlam_h;
            lambda_ds(i) += dlam_d;
        }

        // if the residual policy is to calculate every iteration, do that
        if (_residual_policy == XPBDResidualPolicy::EVERY_ITERATION)
        {
            if (const OutputSimulation* output_sim = dynamic_cast<const OutputSimulation*>(_sim))
            {
                _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
                // if we are calculating the residuals every Gauss Seidel iteration, it's likely we want to create
                // a residual vs. iteration number plot, so write info to file
                output_sim->printInfo();
            }
        }
    }

    // if the residual policy is to calculate every substep, do that
    if (_residual_policy == XPBDResidualPolicy::EVERY_SUBSTEP)
    {
        _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
    }
}

void XPBDMeshObject::_projectConstraintsRuckerFull(const double dt)
{
    VerticesMat inertial_positions = _vertices;

    std::vector<Eigen::Matrix<double, 3, 4>> C_grads(_elements.rows()*2);
    Eigen::VectorXd alpha_tildes(_elements.rows()*2);

    // compute RHS, which is just -C(x_tilde)
    Eigen::VectorXd neg_C(_elements.rows()*2);

    for (int i = 0; i < _elements.rows(); i++)
    {
        const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);

        _computeF(i, _lX, _lF);

        /** DEVIATORIC CONSTRAINT */
        const double C_d = _computeDeviatoricConstraint(_lF, _Q[i], _lC_d_grads);

        // compute the deviatoric alpha
        const double alpha_d = 1/(_material.mu() * _vols[i]);

        /** HYDROSTATIC CONSTRAINT */

        const double C_h = _computeHydrostaticConstraint(_lF, _Q[i], _lF_cross, _lC_h_grads);

        // compute the hydrostatic alpha
        const double alpha_h = 1/(_material.lambda() * _vols[i]);


        alpha_tildes(2*i) = alpha_h / (dt*dt);
        alpha_tildes(2*i+1) = alpha_d / (dt*dt);

        C_grads[2*i].col(0) = _lC_h_grads.col(0);
        C_grads[2*i].col(1) = _lC_h_grads.col(1);
        C_grads[2*i].col(2) = _lC_h_grads.col(2);
        C_grads[2*i].col(3) = _lC_h_grads.col(3);

        C_grads[2*i+1].col(0) = _lC_d_grads.col(0);
        C_grads[2*i+1].col(1) = _lC_d_grads.col(1);
        C_grads[2*i+1].col(2) = _lC_d_grads.col(2);
        C_grads[2*i+1].col(3) = _lC_d_grads.col(3);

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


    // if the residual policy is to calculate every substep, do that
    if (_residual_policy != XPBDResidualPolicy::NEVER)
    {
        _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
    }
    
}

void XPBDMeshObject::_projectConstraintsSplitDeviatoricSequential(const double dt)
{
    // Eigen::VectorXd lambda_hs = Eigen::VectorXd::Zero(_elements.rows());
    // accumulated deviatoric Lagrange multipliers
    // Eigen::VectorXd lambda_ds = Eigen::VectorXd::Zero(_elements.rows());

    Eigen::MatrixXd lambdas = Eigen::MatrixXd::Zero(_elements.rows(), 10);

    // dlams
    // Eigen::VectorXd dlam_hs = Eigen::VectorXd::Zero(_elements.rows());
    // Eigen::VectorXd dlam_ds = Eigen::VectorXd::Zero(_elements.rows());

    // store positions before constraints are projected
    // this is x-tilde, seen in XPBD eqn 8 - used for computing primary residual
    VerticesMat inertial_positions = _vertices;


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

            const double inv_m1 = 1.0/m1;
            const double inv_m2 = 1.0/m2;
            const double inv_m3 = 1.0/m3;
            const double inv_m4 = 1.0/m4;

            // compute the deviatoric alpha
            const double alpha_d = 1/(_material.mu() * _vols[i]);

            double dlam = 0;

            const double denom_1 =  inv_m1 * _Q[i](0,0) * _Q[i](0,0) + 
                                    inv_m2 * _Q[i](1,0) * _Q[i](1,0) + 
                                    inv_m3 * _Q[i](2,0) * _Q[i](2,0) +
                                    inv_m4 * (-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0)) * (-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0)) +
                                    alpha_d/(dt*dt);
            
            const double denom_2 =  inv_m1 * _Q[i](0,1) * _Q[i](0,1) +
                                    inv_m2 * _Q[i](1,1) * _Q[i](1,1) +
                                    inv_m3 * _Q[i](2,1) * _Q[i](2,1) +
                                    inv_m4 * (-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1)) * (-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1)) +
                                    alpha_d/(dt*dt);
            
            const double denom_3 =  inv_m1 * _Q[i](0,2) * _Q[i](0,2) +
                                    inv_m2 * _Q[i](1,2) * _Q[i](1,2) + 
                                    inv_m3 * _Q[i](2,2) * _Q[i](2,2) +
                                    inv_m4 * (-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2)) * (-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2)) +
                                    alpha_d/(dt*dt);

            // F_11
            _computeF(i, _lX, _lF);
            dlam = (-_lF(0,0) - alpha_d/(dt*dt) * lambdas(i, 0)) / denom_1;

            _vertices(elem(0), 0) += inv_m1 * _Q[i](0,0) * dlam;
            _vertices(elem(1), 0) += inv_m2 * _Q[i](1,0) * dlam;
            _vertices(elem(2), 0) += inv_m3 * _Q[i](2,0) * dlam;
            _vertices(elem(3), 0) += inv_m4 * (-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0)) * dlam;

            lambdas(i,0) += dlam;

            // F_12
            _computeF(i, _lX, _lF);
            dlam = (-_lF(0,1) - alpha_d/(dt*dt) * lambdas(i, 1)) / denom_2;

            _vertices(elem(0), 0) += inv_m1 * _Q[i](0,1) * dlam;
            _vertices(elem(1), 0) += inv_m2 * _Q[i](1,1) * dlam;
            _vertices(elem(2), 0) += inv_m3 * _Q[i](2,1) * dlam;
            _vertices(elem(3), 0) += inv_m4 * (-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1)) * dlam;

            lambdas(i,1) += dlam;

            // F_13
            _computeF(i, _lX, _lF);
            dlam = (-_lF(0,2) - alpha_d/(dt*dt) * lambdas(i, 2)) / denom_3;

            _vertices(elem(0), 0) += inv_m1 * _Q[i](0,2) * dlam;
            _vertices(elem(1), 0) += inv_m2 * _Q[i](1,2) * dlam;
            _vertices(elem(2), 0) += inv_m3 * _Q[i](2,2) * dlam;
            _vertices(elem(3), 0) += inv_m4 * (-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2)) * dlam;

            lambdas(i,2) += dlam;



            // F_21
            _computeF(i, _lX, _lF);
            dlam = (-_lF(1,0) - alpha_d/(dt*dt) * lambdas(i, 3)) / denom_1;

            _vertices(elem(0), 1) += inv_m1 * _Q[i](0,0) * dlam;
            _vertices(elem(1), 1) += inv_m2 * _Q[i](1,0) * dlam;
            _vertices(elem(2), 1) += inv_m3 * _Q[i](2,0) * dlam;
            _vertices(elem(3), 1) += inv_m4 * (-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0)) * dlam;

            lambdas(i,3) += dlam;

            // F_22
            _computeF(i, _lX, _lF);
            dlam = (-_lF(1,1) - alpha_d/(dt*dt) * lambdas(i, 4)) / denom_2;

            _vertices(elem(0), 1) += inv_m1 * _Q[i](0,1) * dlam;
            _vertices(elem(1), 1) += inv_m2 * _Q[i](1,1) * dlam;
            _vertices(elem(2), 1) += inv_m3 * _Q[i](2,1) * dlam;
            _vertices(elem(3), 1) += inv_m4 * (-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1)) * dlam;

            lambdas(i,4) += dlam;

            // F_23
            _computeF(i, _lX, _lF);
            dlam = (-_lF(1,2) - alpha_d/(dt*dt) * lambdas(i, 5)) / denom_3;

            _vertices(elem(0), 1) += inv_m1 * _Q[i](0,2) * dlam;
            _vertices(elem(1), 1) += inv_m2 * _Q[i](1,2) * dlam;
            _vertices(elem(2), 1) += inv_m3 * _Q[i](2,2) * dlam;
            _vertices(elem(3), 1) += inv_m4 * (-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2)) * dlam;

            lambdas(i,5) += dlam;



            // F_31
            _computeF(i, _lX, _lF);
            dlam = (-_lF(2,0) - alpha_d/(dt*dt) * lambdas(i, 6)) / denom_1;

            _vertices(elem(0), 2) += inv_m1 * _Q[i](0,0) * dlam;
            _vertices(elem(1), 2) += inv_m2 * _Q[i](1,0) * dlam;
            _vertices(elem(2), 2) += inv_m3 * _Q[i](2,0) * dlam;
            _vertices(elem(3), 2) += inv_m4 * (-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0)) * dlam;

            lambdas(i,6) += dlam;

            // F_32
            _computeF(i, _lX, _lF);
            dlam = (-_lF(2,1) - alpha_d/(dt*dt) * lambdas(i, 7)) / denom_2;

            _vertices(elem(0), 2) += inv_m1 * _Q[i](0,1) * dlam;
            _vertices(elem(1), 2) += inv_m2 * _Q[i](1,1) * dlam;
            _vertices(elem(2), 2) += inv_m3 * _Q[i](2,1) * dlam;
            _vertices(elem(3), 2) += inv_m4 * (-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1)) * dlam;

            lambdas(i,7) += dlam;

            // F_33
            _computeF(i, _lX, _lF);
            dlam = (-_lF(2,2) - alpha_d/(dt*dt) * lambdas(i, 8)) / denom_3;

            _vertices(elem(0), 2) += inv_m1 * _Q[i](0,2) * dlam;
            _vertices(elem(1), 2) += inv_m2 * _Q[i](1,2) * dlam;
            _vertices(elem(2), 2) += inv_m3 * _Q[i](2,2) * dlam;
            _vertices(elem(3), 2) += inv_m4 * (-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2)) * dlam;

            lambdas(i,8) += dlam;

            /** HYDROSTATIC CONSTRAINT */

            _computeF(i, _lX, _lF);
            const double C_h = _computeHydrostaticConstraint(_lF, _Q[i], _lF_cross, _lC_h_grads);
            // compute the hydrostatic alpha
            const double alpha_h = 1/(_material.lambda() * _vols[i]);

            const double dlam_h = (-C_h - alpha_h * lambdas(i,9) / (dt*dt)) / 
                ((
                    inv_m1*_lC_h_grads.col(0).squaredNorm() + 
                    inv_m2*_lC_h_grads.col(1).squaredNorm() + 
                    inv_m3*_lC_h_grads.col(2).squaredNorm() + 
                    inv_m4*_lC_h_grads.col(3).squaredNorm()
                 ) + alpha_h/(dt*dt));

            _vertices.row(elem(0)) += _lC_h_grads.col(0) * dlam_h * inv_m1;
            _vertices.row(elem(1)) += _lC_h_grads.col(1) * dlam_h * inv_m2;
            _vertices.row(elem(2)) += _lC_h_grads.col(2) * dlam_h * inv_m3;
            _vertices.row(elem(3)) += _lC_h_grads.col(3) * dlam_h * inv_m4;

            // update Lagrange multipliers
            // lambda_hs(i) += dlam_h;
            lambdas(i,9) += dlam_h;

        }

        // if the residual policy is to calculate every iteration, do that
        if (_residual_policy == XPBDResidualPolicy::EVERY_ITERATION)
        {
            if (const OutputSimulation* output_sim = dynamic_cast<const OutputSimulation*>(_sim))
            {
                _calculateResidualsSplitDeviatoric(dt, inertial_positions, lambdas);
                // if we are calculating the residuals every Gauss Seidel iteration, it's likely we want to create
                // a residual vs. iteration number plot, so write info to file
                output_sim->printInfo();
            }
        }
    }

    // if the residual policy is to calculate every substep, do that
    if (_residual_policy == XPBDResidualPolicy::EVERY_SUBSTEP)
    {
        _calculateResidualsSplitDeviatoric(dt, inertial_positions, lambdas);
    }
}

void XPBDMeshObject::_projectConstraintsSplitDeviatoricSimultaneous9(const double dt)
{
    Eigen::MatrixXd lambdas = Eigen::MatrixXd::Zero(_elements.rows(), 10);

    VerticesMat inertial_positions = _vertices;

    Eigen::VectorXd b(9);
    Eigen::VectorXd dlam(9);
    for (unsigned gi = 0; gi < _num_iters; gi++)
    {
        for (int i = 0; i < _elements.rows(); i++)
        {
            const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);
            // extract masses of each vertex in the current element

            const double inv_m1 = 1.0/_m[elem(0)];
            const double inv_m2 = 1.0/_m[elem(1)];
            const double inv_m3 = 1.0/_m[elem(2)];
            const double inv_m4 = 1.0/_m[elem(3)];

            // compute the deviatoric alpha
            const double alpha_d = 1/(_material.mu() * _vols[i]);
            const double alpha_d_tilde = alpha_d / (dt*dt);

            // compute the deformation gradient
            _computeF(i, _lX, _lF);

            // calculate the RHS of the matrix eq (i.e. -C - alpha_tilde * lambda)
            b(0) = -_lF(0,0) - alpha_d_tilde * lambdas(i,0);
            b(1) = -_lF(0,1) - alpha_d_tilde * lambdas(i,1);
            b(2) = -_lF(0,2) - alpha_d_tilde * lambdas(i,2);
            b(3) = -_lF(1,0) - alpha_d_tilde * lambdas(i,3);
            b(4) = -_lF(1,1) - alpha_d_tilde * lambdas(i,4);
            b(5) = -_lF(1,2) - alpha_d_tilde * lambdas(i,5);
            b(6) = -_lF(2,0) - alpha_d_tilde * lambdas(i,6);
            b(7) = -_lF(2,1) - alpha_d_tilde * lambdas(i,7);
            b(8) = -_lF(2,2) - alpha_d_tilde * lambdas(i,8);

            // solve for dlams
            dlam(Eigen::seq(0,2)) = _A_inv[i] * b(Eigen::seq(0,2));
            dlam(Eigen::seq(3,5)) = _A_inv[i] * b(Eigen::seq(3,5));
            dlam(Eigen::seq(6,8)) = _A_inv[i] * b(Eigen::seq(6,8));

            // apply position updates
            _vertices(elem(0), 0) += inv_m1 * (dlam(0)*_Q[i](0,0) + dlam(1)*_Q[i](0,1) + dlam(2)*_Q[i](0,2));
            _vertices(elem(0), 1) += inv_m1 * (dlam(3)*_Q[i](0,0) + dlam(4)*_Q[i](0,1) + dlam(5)*_Q[i](0,2));
            _vertices(elem(0), 2) += inv_m1 * (dlam(6)*_Q[i](0,0) + dlam(7)*_Q[i](0,1) + dlam(8)*_Q[i](0,2));

            _vertices(elem(1), 0) += inv_m2 * (dlam(0)*_Q[i](1,0) + dlam(1)*_Q[i](1,1) + dlam(2)*_Q[i](1,2));
            _vertices(elem(1), 1) += inv_m2 * (dlam(3)*_Q[i](1,0) + dlam(4)*_Q[i](1,1) + dlam(5)*_Q[i](1,2));
            _vertices(elem(1), 2) += inv_m2 * (dlam(6)*_Q[i](1,0) + dlam(7)*_Q[i](1,1) + dlam(8)*_Q[i](1,2));

            _vertices(elem(2), 0) += inv_m3 * (dlam(0)*_Q[i](2,0) + dlam(1)*_Q[i](2,1) + dlam(2)*_Q[i](2,2));
            _vertices(elem(2), 1) += inv_m3 * (dlam(3)*_Q[i](2,0) + dlam(4)*_Q[i](2,1) + dlam(5)*_Q[i](2,2));
            _vertices(elem(2), 2) += inv_m3 * (dlam(6)*_Q[i](2,0) + dlam(7)*_Q[i](2,1) + dlam(8)*_Q[i](2,2));

            _vertices(elem(3), 0) += inv_m4 * (dlam(0)*(-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0)) + dlam(1)*(-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1)) + dlam(2)*(-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2)));
            _vertices(elem(3), 1) += inv_m4 * (dlam(3)*(-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0)) + dlam(4)*(-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1)) + dlam(5)*(-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2))); 
            _vertices(elem(3), 2) += inv_m4 * (dlam(6)*(-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0)) + dlam(7)*(-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1)) + dlam(8)*(-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2)));

            lambdas(i, Eigen::seq(0,8)) += dlam;

            

            /** HYDROSTATIC CONSTRAINT */

            _computeF(i, _lX, _lF);
            const double C_h = _computeHydrostaticConstraint(_lF, _Q[i], _lF_cross, _lC_h_grads);
            // compute the hydrostatic alpha
            const double alpha_h = 1/(_material.lambda() * _vols[i]);

            const double dlam_h = (-C_h - alpha_h * lambdas(i,9) / (dt*dt)) / 
                ((
                    inv_m1*_lC_h_grads.col(0).squaredNorm() + 
                    inv_m2*_lC_h_grads.col(1).squaredNorm() + 
                    inv_m3*_lC_h_grads.col(2).squaredNorm() + 
                    inv_m4*_lC_h_grads.col(3).squaredNorm()
                 ) + alpha_h/(dt*dt));

            _vertices.row(elem(0)) += _lC_h_grads.col(0) * dlam_h * inv_m1;
            _vertices.row(elem(1)) += _lC_h_grads.col(1) * dlam_h * inv_m2;
            _vertices.row(elem(2)) += _lC_h_grads.col(2) * dlam_h * inv_m3;
            _vertices.row(elem(3)) += _lC_h_grads.col(3) * dlam_h * inv_m4;

            // update Lagrange multipliers
            // lambda_hs(i) += dlam_h;
            lambdas(i,9) += dlam_h;

        }

        // if the residual policy is to calculate every iteration, do that
        if (_residual_policy == XPBDResidualPolicy::EVERY_ITERATION)
        {
            if (const OutputSimulation* output_sim = dynamic_cast<const OutputSimulation*>(_sim))
            {
                _calculateResidualsSplitDeviatoric(dt, inertial_positions, lambdas);
                // if we are calculating the residuals every Gauss Seidel iteration, it's likely we want to create
                // a residual vs. iteration number plot, so write info to file
                output_sim->printInfo();
            }
        }
    }

    // if the residual policy is to calculate every substep, do that
    if (_residual_policy == XPBDResidualPolicy::EVERY_SUBSTEP)
    {
        _calculateResidualsSplitDeviatoric(dt, inertial_positions, lambdas);
    }
}

void XPBDMeshObject::_projectConstraintsSplitDeviatoricSimultaneous9G(const double dt)
{
    Eigen::MatrixXd lambdas = Eigen::MatrixXd::Zero(_elements.rows(), 10);

    VerticesMat inertial_positions = _vertices;
    VerticesMat x_minus_x_tilde = _vertices - inertial_positions;

    Eigen::VectorXd b(9);
    Eigen::VectorXd dlam(9);
    for (unsigned gi = 0; gi < _num_iters; gi++)
    {
        x_minus_x_tilde = _vertices - inertial_positions;
        for (int i = 0; i < _elements.rows(); i++)
        {
            const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);
            // extract masses of each vertex in the current element

            const double inv_m1 = 1.0/_m[elem(0)];
            const double inv_m2 = 1.0/_m[elem(1)];
            const double inv_m3 = 1.0/_m[elem(2)];
            const double inv_m4 = 1.0/_m[elem(3)];

            // compute the deviatoric alpha
            const double alpha_d = 1/(_material.mu() * _vols[i]);
            const double alpha_d_tilde = alpha_d / (dt*dt);

            // compute the deformation gradient
            _computeF(i, _lX, _lF);

            const double delC_x_1 = _Q[i](0,0)*x_minus_x_tilde(elem(0),0) +
                                    _Q[i](1,0)*x_minus_x_tilde(elem(1),0) +
                                    _Q[i](2,0)*x_minus_x_tilde(elem(2),0) +
                                    (-_Q[i](0,0)-_Q[i](1,0)-_Q[i](2,0))*x_minus_x_tilde(elem(3),0);
            const double delC_x_2 = _Q[i](0,1)*x_minus_x_tilde(elem(0),0) +
                                    _Q[i](1,1)*x_minus_x_tilde(elem(1),0) +
                                    _Q[i](2,1)*x_minus_x_tilde(elem(2),0) +
                                    (-_Q[i](0,1)-_Q[i](1,1)-_Q[i](2,1))*x_minus_x_tilde(elem(3),0);
            const double delC_x_3 = _Q[i](0,2)*x_minus_x_tilde(elem(0),0) +
                                    _Q[i](1,2)*x_minus_x_tilde(elem(1),0) +
                                    _Q[i](2,2)*x_minus_x_tilde(elem(2),0) +
                                    (-_Q[i](0,2)-_Q[i](1,2)-_Q[i](2,2))*x_minus_x_tilde(elem(3),0);
            const double delC_x_4 = _Q[i](0,0)*x_minus_x_tilde(elem(0),1) +
                                    _Q[i](1,0)*x_minus_x_tilde(elem(1),1) +
                                    _Q[i](2,0)*x_minus_x_tilde(elem(2),1) +
                                    (-_Q[i](0,0)-_Q[i](1,0)-_Q[i](2,0))*x_minus_x_tilde(elem(3),1);
            const double delC_x_5 = _Q[i](0,1)*x_minus_x_tilde(elem(0),1) +
                                    _Q[i](1,1)*x_minus_x_tilde(elem(1),1) +
                                    _Q[i](2,1)*x_minus_x_tilde(elem(2),1) +
                                    (-_Q[i](0,1)-_Q[i](1,1)-_Q[i](2,1))*x_minus_x_tilde(elem(3),1);
            const double delC_x_6 = _Q[i](0,2)*x_minus_x_tilde(elem(0),1) +
                                    _Q[i](1,2)*x_minus_x_tilde(elem(1),1) +
                                    _Q[i](2,2)*x_minus_x_tilde(elem(2),1) +
                                    (-_Q[i](0,2)-_Q[i](1,2)-_Q[i](2,2))*x_minus_x_tilde(elem(3),1);
            const double delC_x_7 = _Q[i](0,0)*x_minus_x_tilde(elem(0),2) +
                                    _Q[i](1,0)*x_minus_x_tilde(elem(1),2) +
                                    _Q[i](2,0)*x_minus_x_tilde(elem(2),2) +
                                    (-_Q[i](0,0)-_Q[i](1,0)-_Q[i](2,0))*x_minus_x_tilde(elem(3),2);
            const double delC_x_8 = _Q[i](0,1)*x_minus_x_tilde(elem(0),2) +
                                    _Q[i](1,1)*x_minus_x_tilde(elem(1),2) +
                                    _Q[i](2,1)*x_minus_x_tilde(elem(2),2) +
                                    (-_Q[i](0,1)-_Q[i](1,1)-_Q[i](2,1))*x_minus_x_tilde(elem(3),2);
            const double delC_x_9 = _Q[i](0,2)*x_minus_x_tilde(elem(0),2) +
                                    _Q[i](1,2)*x_minus_x_tilde(elem(1),2) +
                                    _Q[i](2,2)*x_minus_x_tilde(elem(2),2) +
                                    (-_Q[i](0,2)-_Q[i](1,2)-_Q[i](2,2))*x_minus_x_tilde(elem(3),2);

            // calculate the RHS of the matrix eq (i.e. -C - alpha_tilde * lambda)
            b(0) = -_lF(0,0) + _g_scaling*delC_x_1;
            b(1) = -_lF(0,1) + _g_scaling*delC_x_2;
            b(2) = -_lF(0,2) + _g_scaling*delC_x_3;
            b(3) = -_lF(1,0) + _g_scaling*delC_x_4;
            b(4) = -_lF(1,1) + _g_scaling*delC_x_5;
            b(5) = -_lF(1,2) + _g_scaling*delC_x_6;
            b(6) = -_lF(2,0) + _g_scaling*delC_x_7;
            b(7) = -_lF(2,1) + _g_scaling*delC_x_8;
            b(8) = -_lF(2,2) + _g_scaling*delC_x_9;

            b(Eigen::seq(0,2)) += -_g_scaling*_A[i]*lambdas(i,Eigen::seq(0,2)).transpose() + (_g_scaling-1)*alpha_d_tilde*lambdas(i,Eigen::seq(0,2)).transpose();
            b(Eigen::seq(3,5)) += -_g_scaling*_A[i]*lambdas(i,Eigen::seq(3,5)).transpose() + (_g_scaling-1)*alpha_d_tilde*lambdas(i,Eigen::seq(3,5)).transpose();
            b(Eigen::seq(6,8)) += -_g_scaling*_A[i]*lambdas(i,Eigen::seq(6,8)).transpose() + (_g_scaling-1)*alpha_d_tilde*lambdas(i,Eigen::seq(6,8)).transpose();

            // solve for dlams
            dlam(Eigen::seq(0,2)) = _A_inv[i] * b(Eigen::seq(0,2));
            dlam(Eigen::seq(3,5)) = _A_inv[i] * b(Eigen::seq(3,5));
            dlam(Eigen::seq(6,8)) = _A_inv[i] * b(Eigen::seq(6,8));

            // apply position updates
            Eigen::VectorXd dlam_plus_lam = dlam + _g_scaling*lambdas(i, Eigen::seq(0,8)).transpose(); 
            _vertices(elem(0), 0) += inv_m1 * (dlam_plus_lam(0)*_Q[i](0,0) + dlam_plus_lam(1)*_Q[i](0,1) + dlam_plus_lam(2)*_Q[i](0,2));
            _vertices(elem(0), 1) += inv_m1 * (dlam_plus_lam(3)*_Q[i](0,0) + dlam_plus_lam(4)*_Q[i](0,1) + dlam_plus_lam(5)*_Q[i](0,2));
            _vertices(elem(0), 2) += inv_m1 * (dlam_plus_lam(6)*_Q[i](0,0) + dlam_plus_lam(7)*_Q[i](0,1) + dlam_plus_lam(8)*_Q[i](0,2));
            _vertices.row(elem(0)) -= _g_scaling*x_minus_x_tilde.row(elem(0));

            _vertices(elem(1), 0) += inv_m2 * (dlam_plus_lam(0)*_Q[i](1,0) + dlam_plus_lam(1)*_Q[i](1,1) + dlam_plus_lam(2)*_Q[i](1,2));
            _vertices(elem(1), 1) += inv_m2 * (dlam_plus_lam(3)*_Q[i](1,0) + dlam_plus_lam(4)*_Q[i](1,1) + dlam_plus_lam(5)*_Q[i](1,2));
            _vertices(elem(1), 2) += inv_m2 * (dlam_plus_lam(6)*_Q[i](1,0) + dlam_plus_lam(7)*_Q[i](1,1) + dlam_plus_lam(8)*_Q[i](1,2));
            _vertices.row(elem(1)) -= _g_scaling*x_minus_x_tilde.row(elem(1));

            _vertices(elem(2), 0) += inv_m3 * (dlam_plus_lam(0)*_Q[i](2,0) + dlam_plus_lam(1)*_Q[i](2,1) + dlam_plus_lam(2)*_Q[i](2,2));
            _vertices(elem(2), 1) += inv_m3 * (dlam_plus_lam(3)*_Q[i](2,0) + dlam_plus_lam(4)*_Q[i](2,1) + dlam_plus_lam(5)*_Q[i](2,2));
            _vertices(elem(2), 2) += inv_m3 * (dlam_plus_lam(6)*_Q[i](2,0) + dlam_plus_lam(7)*_Q[i](2,1) + dlam_plus_lam(8)*_Q[i](2,2));
            _vertices.row(elem(2)) -= _g_scaling*x_minus_x_tilde.row(elem(2));

            _vertices(elem(3), 0) += inv_m4 * (dlam_plus_lam(0)*(-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0)) + dlam_plus_lam(1)*(-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1)) + dlam_plus_lam(2)*(-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2)));
            _vertices(elem(3), 1) += inv_m4 * (dlam_plus_lam(3)*(-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0)) + dlam_plus_lam(4)*(-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1)) + dlam_plus_lam(5)*(-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2))); 
            _vertices(elem(3), 2) += inv_m4 * (dlam_plus_lam(6)*(-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0)) + dlam_plus_lam(7)*(-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1)) + dlam_plus_lam(8)*(-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2)));
            _vertices.row(elem(3)) -= _g_scaling*x_minus_x_tilde.row(elem(3));

            lambdas(i, Eigen::seq(0,8)) += dlam;

            

            /** HYDROSTATIC CONSTRAINT */

            _computeF(i, _lX, _lF);
            const double C_h = _computeHydrostaticConstraint(_lF, _Q[i], _lF_cross, _lC_h_grads);
            // compute the hydrostatic alpha
            const double alpha_h = 1/(_material.lambda() * _vols[i]);
            
            const double delC_Minv_delC_h = (inv_m1)*_lC_h_grads.col(0).squaredNorm() + 
                    (inv_m2)*_lC_h_grads.col(1).squaredNorm() + 
                    (inv_m3)*_lC_h_grads.col(2).squaredNorm() + 
                    (inv_m4)*_lC_h_grads.col(3).squaredNorm();
            const double delC_Minv_delC_alpha_h = delC_Minv_delC_h + alpha_h/(dt*dt);

            const double delC_x_h = _lC_h_grads.col(0).dot(x_minus_x_tilde.row(elem(0))) +
                                    _lC_h_grads.col(1).dot(x_minus_x_tilde.row(elem(1))) +
                                    _lC_h_grads.col(2).dot(x_minus_x_tilde.row(elem(2))) +
                                    _lC_h_grads.col(3).dot(x_minus_x_tilde.row(elem(3)));

            const double dlam_h = (-C_h - (_g_scaling*delC_Minv_delC_h + alpha_h/(dt*dt)) * lambdas(i,9) + _g_scaling * delC_x_h) / delC_Minv_delC_alpha_h;
                    

            _vertices.row(elem(0)) += _lC_h_grads.col(0) * (dlam_h + _g_scaling * lambdas(i,9)) * inv_m1 - _g_scaling * x_minus_x_tilde.row(elem(0)).transpose();
            _vertices.row(elem(1)) += _lC_h_grads.col(1) * (dlam_h + _g_scaling * lambdas(i,9)) * inv_m2 - _g_scaling * x_minus_x_tilde.row(elem(1)).transpose();
            _vertices.row(elem(2)) += _lC_h_grads.col(2) * (dlam_h + _g_scaling * lambdas(i,9)) * inv_m3 - _g_scaling * x_minus_x_tilde.row(elem(2)).transpose();
            _vertices.row(elem(3)) += _lC_h_grads.col(3) * (dlam_h + _g_scaling * lambdas(i,9)) * inv_m4 - _g_scaling * x_minus_x_tilde.row(elem(3)).transpose();

            // update Lagrange multipliers
            // lambda_hs(i) += dlam_h;
            lambdas(i,9) += dlam_h;

        }

        // if the residual policy is to calculate every iteration, do that
        if (_residual_policy == XPBDResidualPolicy::EVERY_ITERATION)
        {
            if (const OutputSimulation* output_sim = dynamic_cast<const OutputSimulation*>(_sim))
            {
                _calculateResidualsSplitDeviatoric(dt, inertial_positions, lambdas);
                // if we are calculating the residuals every Gauss Seidel iteration, it's likely we want to create
                // a residual vs. iteration number plot, so write info to file
                output_sim->printInfo();
            }
        }
    }

    // if the residual policy is to calculate every substep, do that
    if (_residual_policy == XPBDResidualPolicy::EVERY_SUBSTEP)
    {
        _calculateResidualsSplitDeviatoric(dt, inertial_positions, lambdas);
    }
}

void XPBDMeshObject::_projectConstraintsSplitDeviatoricSimultaneous10(const double dt)
{
    Eigen::MatrixXd lambdas = Eigen::MatrixXd::Zero(_elements.rows(), 10);

    // remember the inertial positions for residual calculation
    VerticesMat inertial_positions = _vertices;

    for (unsigned gi = 0; gi < _num_iters; gi++)
    {
        for (int i = 0; i < _elements.rows(); i++)
        {
            const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);

            // extract the inverse masses of each vertex in the current element
            const double inv_m1 = 1.0/_m[elem(0)];
            const double inv_m2 = 1.0/_m[elem(1)];
            const double inv_m3 = 1.0/_m[elem(2)];
            const double inv_m4 = 1.0/_m[elem(3)];

        
            // compute the deformation gradient
            _computeF(i, _lX, _lF);

            // compute the hydrostatic constraint and its gradients
            const double C_h = _computeHydrostaticConstraint(_lF, _Q[i], _lF_cross, _lC_h_grads);
            // compute the hydrostatic alpha
            const double alpha_h = 1/(_material.lambda() * _vols[i]);


            // compute the deviatoric alpha
            const double alpha_d = 1/(_material.mu() * _vols[i]);
            const double alpha_d_tilde = alpha_d / (dt*dt);

            // calculate the RHS of the matrix eq (i.e. -C - alpha_tilde * lambda)
            // and use lowercase b to represent this vector
            _lb(0) = -_lF(0,0) - alpha_d_tilde * lambdas(i,0);
            _lb(1) = -_lF(0,1) - alpha_d_tilde * lambdas(i,1);
            _lb(2) = -_lF(0,2) - alpha_d_tilde * lambdas(i,2);
            _lb(3) = -_lF(1,0) - alpha_d_tilde * lambdas(i,3);
            _lb(4) = -_lF(1,1) - alpha_d_tilde * lambdas(i,4);
            _lb(5) = -_lF(1,2) - alpha_d_tilde * lambdas(i,5);
            _lb(6) = -_lF(2,0) - alpha_d_tilde * lambdas(i,6);
            _lb(7) = -_lF(2,1) - alpha_d_tilde * lambdas(i,7);
            _lb(8) = -_lF(2,2) - alpha_d_tilde * lambdas(i,8);
            _lb(9) = -C_h - alpha_h / (dt*dt) * lambdas(i,9);

            // form 10x10 LHS matrix inverse (i.e. the M in Mx=b)
            // when solving for dlam, this is (delC M^-1 delC^T + alpha_tilde)
            // we break the 10x10 into 4 matrices:
            //   [ A B ]
            //   [ C D ]
            // A is 9x9 and constant (from the 9 deformation gradient constraints)
            // B = C^T and is 9x1 and encodes the interaction between each deformation gradient constraint and the hydrostatic constraint gradient
            // D is 1x1 and only depends on the hydrostatic constraint gradient
            // now we can compute the inverse according to
            //   [ A^-1 + A^-1B(D-CA^-1B)^-1CA^-1  -A^-1B(D-CA^-1B)^-1 ]
            //   [        -(D-CA^-1B)^-1CA^-1           (D-CA^-1B)^-1  ]
            //
            // things to note:
            // - (D-CA^-1B) is a scalar (i.e. 1x1), so taking its matrix inverse is trivial
            // - A^-1 shows up a lot - however this can be precomputed since A is constant and block diagonal
            // - the products CA^-1 and A^-1B show up a lot as well, and are 9-vectors
            //
            // A has special structure, which is that it is 9x9 block diagonal, with 3 3x3 constant matrices along its diagonal
            // it can be shown that these 3x3 matrices along the diagonal are actually the same for a given A!
            // That is what '_A_inv[i]' is - it is the 3x3 matrix inverse of these block diagonal matrices of A for a given element i
            //
            // Start by forming the B 9x1 matrix
            _lB(0) = inv_m1*_Q[i](0,0)*_lC_h_grads(0,0) + inv_m2*_Q[i](1,0)*_lC_h_grads(0,1) + inv_m3*_Q[i](2,0)*_lC_h_grads(0,2) + inv_m4*(-_Q[i](0,0)-_Q[i](1,0)-_Q[i](2,0))*_lC_h_grads(0,3);
            _lB(1) = inv_m1*_Q[i](0,1)*_lC_h_grads(0,0) + inv_m2*_Q[i](1,1)*_lC_h_grads(0,1) + inv_m3*_Q[i](2,1)*_lC_h_grads(0,2) + inv_m4*(-_Q[i](0,1)-_Q[i](1,1)-_Q[i](2,1))*_lC_h_grads(0,3);
            _lB(2) = inv_m1*_Q[i](0,2)*_lC_h_grads(0,0) + inv_m2*_Q[i](1,2)*_lC_h_grads(0,1) + inv_m3*_Q[i](2,2)*_lC_h_grads(0,2) + inv_m4*(-_Q[i](0,2)-_Q[i](1,2)-_Q[i](2,2))*_lC_h_grads(0,3);
            _lB(3) = inv_m1*_Q[i](0,0)*_lC_h_grads(1,0) + inv_m2*_Q[i](1,0)*_lC_h_grads(1,1) + inv_m3*_Q[i](2,0)*_lC_h_grads(1,2) + inv_m4*(-_Q[i](0,0)-_Q[i](1,0)-_Q[i](2,0))*_lC_h_grads(1,3);
            _lB(4) = inv_m1*_Q[i](0,1)*_lC_h_grads(1,0) + inv_m2*_Q[i](1,1)*_lC_h_grads(1,1) + inv_m3*_Q[i](2,1)*_lC_h_grads(1,2) + inv_m4*(-_Q[i](0,1)-_Q[i](1,1)-_Q[i](2,1))*_lC_h_grads(1,3);
            _lB(5) = inv_m1*_Q[i](0,2)*_lC_h_grads(1,0) + inv_m2*_Q[i](1,2)*_lC_h_grads(1,1) + inv_m3*_Q[i](2,2)*_lC_h_grads(1,2) + inv_m4*(-_Q[i](0,2)-_Q[i](1,2)-_Q[i](2,2))*_lC_h_grads(1,3);
            _lB(6) = inv_m1*_Q[i](0,0)*_lC_h_grads(2,0) + inv_m2*_Q[i](1,0)*_lC_h_grads(2,1) + inv_m3*_Q[i](2,0)*_lC_h_grads(2,2) + inv_m4*(-_Q[i](0,0)-_Q[i](1,0)-_Q[i](2,0))*_lC_h_grads(2,3);
            _lB(7) = inv_m1*_Q[i](0,1)*_lC_h_grads(2,0) + inv_m2*_Q[i](1,1)*_lC_h_grads(2,1) + inv_m3*_Q[i](2,1)*_lC_h_grads(2,2) + inv_m4*(-_Q[i](0,1)-_Q[i](1,1)-_Q[i](2,1))*_lC_h_grads(2,3);
            _lB(8) = inv_m1*_Q[i](0,2)*_lC_h_grads(2,0) + inv_m2*_Q[i](1,2)*_lC_h_grads(2,1) + inv_m3*_Q[i](2,2)*_lC_h_grads(2,2) + inv_m4*(-_Q[i](0,2)-_Q[i](1,2)-_Q[i](2,2))*_lC_h_grads(2,3);

            // compute the D 1x1 matrix - only has to do with the hydrostatic constraint gradient
            const double D =    inv_m1*_lC_h_grads.col(0).squaredNorm() + 
                                inv_m2*_lC_h_grads.col(1).squaredNorm() + 
                                inv_m3*_lC_h_grads.col(2).squaredNorm() + 
                                inv_m4*_lC_h_grads.col(3).squaredNorm() +
                                alpha_h/(dt*dt);
            
            // compute the product CA^-1 - result is a 9-vector
            // we take advantage of the block diagonal structure of A here so we don't have to do full 9x9 matrix multiplication
            _lCA_inv(0) = _lB(Eigen::seq(0,2)).dot(_A_inv[i].col(0));
            _lCA_inv(1) = _lB(Eigen::seq(0,2)).dot(_A_inv[i].col(1));
            _lCA_inv(2) = _lB(Eigen::seq(0,2)).dot(_A_inv[i].col(2));
            _lCA_inv(3) = _lB(Eigen::seq(3,5)).dot(_A_inv[i].col(0));
            _lCA_inv(4) = _lB(Eigen::seq(3,5)).dot(_A_inv[i].col(1));
            _lCA_inv(5) = _lB(Eigen::seq(3,5)).dot(_A_inv[i].col(2));
            _lCA_inv(6) = _lB(Eigen::seq(6,8)).dot(_A_inv[i].col(0));
            _lCA_inv(7) = _lB(Eigen::seq(6,8)).dot(_A_inv[i].col(1));
            _lCA_inv(8) = _lB(Eigen::seq(6,8)).dot(_A_inv[i].col(2));

            // compute the product A^-1B - result is a 9-vector
            // we take advantage of the block diagonal structure of A here so we don't have to do full 9x9 matrix multiplication
            _lA_invB(0) = _lB(Eigen::seq(0,2)).dot(_A_inv[i].row(0));
            _lA_invB(1) = _lB(Eigen::seq(0,2)).dot(_A_inv[i].row(1));
            _lA_invB(2) = _lB(Eigen::seq(0,2)).dot(_A_inv[i].row(2));
            _lA_invB(3) = _lB(Eigen::seq(3,5)).dot(_A_inv[i].row(0));
            _lA_invB(4) = _lB(Eigen::seq(3,5)).dot(_A_inv[i].row(1));
            _lA_invB(5) = _lB(Eigen::seq(3,5)).dot(_A_inv[i].row(2));
            _lA_invB(6) = _lB(Eigen::seq(6,8)).dot(_A_inv[i].row(0));
            _lA_invB(7) = _lB(Eigen::seq(6,8)).dot(_A_inv[i].row(1));
            _lA_invB(8) = _lB(Eigen::seq(6,8)).dot(_A_inv[i].row(2));

            // compute (D-CA^-1B)^-1 - this product shows up a lot in the block matrix inversion formula
            const double inv_D_minus_CA_invB = 1/(D - _lCA_inv.dot(_lB));

            // assemble the 'M' 10x10 system matrix
            // the 9x9 block is A^-1 + A^-1B(D-CA^-1B)^-1CA^-1
            _lM.block<9,9>(0,0) = inv_D_minus_CA_invB * _lA_invB * _lCA_inv.transpose();
            _lM.block<3,3>(0,0) += _A_inv[i];
            _lM.block<3,3>(3,3) += _A_inv[i];
            _lM.block<3,3>(6,6) += _A_inv[i];

            // the 1x9 block in the bottommost row is -(D-CA^-1B)^-1CA^-1
            _lM(9, Eigen::seq(0,8)) = -inv_D_minus_CA_invB * _lCA_inv;

            // the 9x1 block in the rightmost column is -A^-1B(D-CA^-1B)^-1
            _lM(Eigen::seq(0,8), 9) = -inv_D_minus_CA_invB * _lA_invB;
            
            // the 1x1 block in the bottom right corner is (D-CA^-1B)^-1
            _lM(9,9) = inv_D_minus_CA_invB;
 
            // solve for dlams through simple matrix multiplication
            _ldlam = _lM*_lb;

            // apply position updates
            // for the deformation gradient constraints, the constraint gradients only affect one component of the position
            //   i.e. the gradients of F11, F12, F13 only affect the x-component of position, F21, F22, F23 only affect the y-component of position, and F31, F32, F33 only affect the z-component
            _vertices(elem(0), 0) += inv_m1 * (_ldlam(0)*_Q[i](0,0) + _ldlam(1)*_Q[i](0,1) + _ldlam(2)*_Q[i](0,2));
            _vertices(elem(0), 1) += inv_m1 * (_ldlam(3)*_Q[i](0,0) + _ldlam(4)*_Q[i](0,1) + _ldlam(5)*_Q[i](0,2));
            _vertices(elem(0), 2) += inv_m1 * (_ldlam(6)*_Q[i](0,0) + _ldlam(7)*_Q[i](0,1) + _ldlam(8)*_Q[i](0,2));

            _vertices(elem(1), 0) += inv_m2 * (_ldlam(0)*_Q[i](1,0) + _ldlam(1)*_Q[i](1,1) + _ldlam(2)*_Q[i](1,2));
            _vertices(elem(1), 1) += inv_m2 * (_ldlam(3)*_Q[i](1,0) + _ldlam(4)*_Q[i](1,1) + _ldlam(5)*_Q[i](1,2));
            _vertices(elem(1), 2) += inv_m2 * (_ldlam(6)*_Q[i](1,0) + _ldlam(7)*_Q[i](1,1) + _ldlam(8)*_Q[i](1,2));

            _vertices(elem(2), 0) += inv_m3 * (_ldlam(0)*_Q[i](2,0) + _ldlam(1)*_Q[i](2,1) + _ldlam(2)*_Q[i](2,2));
            _vertices(elem(2), 1) += inv_m3 * (_ldlam(3)*_Q[i](2,0) + _ldlam(4)*_Q[i](2,1) + _ldlam(5)*_Q[i](2,2));
            _vertices(elem(2), 2) += inv_m3 * (_ldlam(6)*_Q[i](2,0) + _ldlam(7)*_Q[i](2,1) + _ldlam(8)*_Q[i](2,2));

            _vertices(elem(3), 0) += inv_m4 * (_ldlam(0)*(-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0)) + _ldlam(1)*(-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1)) + _ldlam(2)*(-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2)));
            _vertices(elem(3), 1) += inv_m4 * (_ldlam(3)*(-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0)) + _ldlam(4)*(-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1)) + _ldlam(5)*(-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2))); 
            _vertices(elem(3), 2) += inv_m4 * (_ldlam(6)*(-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0)) + _ldlam(7)*(-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1)) + _ldlam(8)*(-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2)));

            // apply position update from hydrostatic constraint
            _vertices.row(elem(0)) += _lC_h_grads.col(0) * _ldlam(9) * inv_m1;
            _vertices.row(elem(1)) += _lC_h_grads.col(1) * _ldlam(9) * inv_m2;
            _vertices.row(elem(2)) += _lC_h_grads.col(2) * _ldlam(9) * inv_m3;
            _vertices.row(elem(3)) += _lC_h_grads.col(3) * _ldlam(9) * inv_m4;

            // update the overall lambdas using dlam
            lambdas.row(i) += _ldlam;



            
        }

        // if the residual policy is to calculate every iteration, do that
        if (_residual_policy == XPBDResidualPolicy::EVERY_ITERATION)
        {
            if (const OutputSimulation* output_sim = dynamic_cast<const OutputSimulation*>(_sim))
            {
                _calculateResidualsSplitDeviatoric(dt, inertial_positions, lambdas);
                // if we are calculating the residuals every Gauss Seidel iteration, it's likely we want to create
                // a residual vs. iteration number plot, so write info to file
                output_sim->printInfo();
            }
        }
    }

    // if the residual policy is to calculate every substep, do that
    if (_residual_policy == XPBDResidualPolicy::EVERY_SUBSTEP)
    {
        _calculateResidualsSplitDeviatoric(dt, inertial_positions, lambdas);
    }

 
}

void XPBDMeshObject::_projectConstraintsSplitDeviatoricSimultaneous10LLT(const double dt)
{
    Eigen::MatrixXd lambdas = Eigen::MatrixXd::Zero(_elements.rows(), 10);

    // remember the inertial positions for residual calculation
    VerticesMat inertial_positions = _vertices;

    for (unsigned gi = 0; gi < _num_iters; gi++)
    {
        for (int i = 0; i < _elements.rows(); i++)
        {
            const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);

            // extract the inverse masses of each vertex in the current element
            const double inv_m1 = 1.0/_m[elem(0)];
            const double inv_m2 = 1.0/_m[elem(1)];
            const double inv_m3 = 1.0/_m[elem(2)];
            const double inv_m4 = 1.0/_m[elem(3)];

        
            // compute the deformation gradient
            _computeF(i, _lX, _lF);

            // compute the hydrostatic constraint and its gradients
            const double C_h = _computeHydrostaticConstraint(_lF, _Q[i], _lF_cross, _lC_h_grads);
            // compute the hydrostatic alpha
            const double alpha_h = 1/(_material.lambda() * _vols[i]);


            // compute the deviatoric alpha
            const double alpha_d = 1/(_material.mu() * _vols[i]);
            const double alpha_d_tilde = alpha_d / (dt*dt);

            // calculate the RHS of the matrix eq (i.e. -C - alpha_tilde * lambda)
            // and use lowercase b to represent this vector
            _lb(0) = -_lF(0,0) - alpha_d_tilde * lambdas(i,0);
            _lb(1) = -_lF(0,1) - alpha_d_tilde * lambdas(i,1);
            _lb(2) = -_lF(0,2) - alpha_d_tilde * lambdas(i,2);
            _lb(3) = -_lF(1,0) - alpha_d_tilde * lambdas(i,3);
            _lb(4) = -_lF(1,1) - alpha_d_tilde * lambdas(i,4);
            _lb(5) = -_lF(1,2) - alpha_d_tilde * lambdas(i,5);
            _lb(6) = -_lF(2,0) - alpha_d_tilde * lambdas(i,6);
            _lb(7) = -_lF(2,1) - alpha_d_tilde * lambdas(i,7);
            _lb(8) = -_lF(2,2) - alpha_d_tilde * lambdas(i,8);
            _lb(9) = -C_h - alpha_h / (dt*dt) * lambdas(i,9);

            // form 10x10 LHS matrix inverse (i.e. the M in Mx=b)
            // when solving for dlam, this is (delC M^-1 delC^T + alpha_tilde)
            // we break the 10x10 into 4 matrices:
            //   [ A B ]
            //   [ C D ]
            // A is 9x9 and constant (from the 9 deformation gradient constraints)
            // B = C^T and is 9x1 and encodes the interaction between each deformation gradient constraint and the hydrostatic constraint gradient
            // D is 1x1 and only depends on the hydrostatic constraint gradient
            // now we can compute the inverse according to
            //   [ A^-1 + A^-1B(D-CA^-1B)^-1CA^-1  -A^-1B(D-CA^-1B)^-1 ]
            //   [        -(D-CA^-1B)^-1CA^-1           (D-CA^-1B)^-1  ]
            //
            // things to note:
            // - (D-CA^-1B) is a scalar (i.e. 1x1), so taking its matrix inverse is trivial
            // - A^-1 shows up a lot - however this can be precomputed since A is constant and block diagonal
            // - the products CA^-1 and A^-1B show up a lot as well, and are 9-vectors
            //
            // A has special structure, which is that it is 9x9 block diagonal, with 3 3x3 constant matrices along its diagonal
            // it can be shown that these 3x3 matrices along the diagonal are actually the same for a given A!
            // That is what '_A_inv[i]' is - it is the 3x3 matrix inverse of these block diagonal matrices of A for a given element i
            //
            // Start by forming the B 9x1 matrix
            _lB(0) = inv_m1*_Q[i](0,0)*_lC_h_grads(0,0) + inv_m2*_Q[i](1,0)*_lC_h_grads(0,1) + inv_m3*_Q[i](2,0)*_lC_h_grads(0,2) + inv_m4*(-_Q[i](0,0)-_Q[i](1,0)-_Q[i](2,0))*_lC_h_grads(0,3);
            _lB(1) = inv_m1*_Q[i](0,1)*_lC_h_grads(0,0) + inv_m2*_Q[i](1,1)*_lC_h_grads(0,1) + inv_m3*_Q[i](2,1)*_lC_h_grads(0,2) + inv_m4*(-_Q[i](0,1)-_Q[i](1,1)-_Q[i](2,1))*_lC_h_grads(0,3);
            _lB(2) = inv_m1*_Q[i](0,2)*_lC_h_grads(0,0) + inv_m2*_Q[i](1,2)*_lC_h_grads(0,1) + inv_m3*_Q[i](2,2)*_lC_h_grads(0,2) + inv_m4*(-_Q[i](0,2)-_Q[i](1,2)-_Q[i](2,2))*_lC_h_grads(0,3);
            _lB(3) = inv_m1*_Q[i](0,0)*_lC_h_grads(1,0) + inv_m2*_Q[i](1,0)*_lC_h_grads(1,1) + inv_m3*_Q[i](2,0)*_lC_h_grads(1,2) + inv_m4*(-_Q[i](0,0)-_Q[i](1,0)-_Q[i](2,0))*_lC_h_grads(1,3);
            _lB(4) = inv_m1*_Q[i](0,1)*_lC_h_grads(1,0) + inv_m2*_Q[i](1,1)*_lC_h_grads(1,1) + inv_m3*_Q[i](2,1)*_lC_h_grads(1,2) + inv_m4*(-_Q[i](0,1)-_Q[i](1,1)-_Q[i](2,1))*_lC_h_grads(1,3);
            _lB(5) = inv_m1*_Q[i](0,2)*_lC_h_grads(1,0) + inv_m2*_Q[i](1,2)*_lC_h_grads(1,1) + inv_m3*_Q[i](2,2)*_lC_h_grads(1,2) + inv_m4*(-_Q[i](0,2)-_Q[i](1,2)-_Q[i](2,2))*_lC_h_grads(1,3);
            _lB(6) = inv_m1*_Q[i](0,0)*_lC_h_grads(2,0) + inv_m2*_Q[i](1,0)*_lC_h_grads(2,1) + inv_m3*_Q[i](2,0)*_lC_h_grads(2,2) + inv_m4*(-_Q[i](0,0)-_Q[i](1,0)-_Q[i](2,0))*_lC_h_grads(2,3);
            _lB(7) = inv_m1*_Q[i](0,1)*_lC_h_grads(2,0) + inv_m2*_Q[i](1,1)*_lC_h_grads(2,1) + inv_m3*_Q[i](2,1)*_lC_h_grads(2,2) + inv_m4*(-_Q[i](0,1)-_Q[i](1,1)-_Q[i](2,1))*_lC_h_grads(2,3);
            _lB(8) = inv_m1*_Q[i](0,2)*_lC_h_grads(2,0) + inv_m2*_Q[i](1,2)*_lC_h_grads(2,1) + inv_m3*_Q[i](2,2)*_lC_h_grads(2,2) + inv_m4*(-_Q[i](0,2)-_Q[i](1,2)-_Q[i](2,2))*_lC_h_grads(2,3);

            // compute the D 1x1 matrix - only has to do with the hydrostatic constraint gradient
            const double D =    inv_m1*_lC_h_grads.col(0).squaredNorm() + 
                                inv_m2*_lC_h_grads.col(1).squaredNorm() + 
                                inv_m3*_lC_h_grads.col(2).squaredNorm() + 
                                inv_m4*_lC_h_grads.col(3).squaredNorm() +
                                alpha_h/(dt*dt);
            
            // form the 10x10 system matrix
            _lM.block<3,3>(0,0) = _A[i];
            _lM.block<3,3>(3,3) = _A[i];
            _lM.block<3,3>(6,6) = _A[i];

            // the 1x9 block in the bottommost row is B^T
            _lM(9, Eigen::seq(0,8)) = _lB;

            // the 9x1 block in the rightmost column is B
            _lM(Eigen::seq(0,8), 9) = _lB;
            
            // the 1x1 block in the bottom right corner is D
            _lM(9,9) = D;
 

            // use Eigen's Cholesky solver to solve the 10x10 system
            _ldlam = _lM.llt().solve(_lb);

            // solve for dlams through simple matrix multiplication
            // _ldlam = _lM*_lb;

            // apply position updates
            // for the deformation gradient constraints, the constraint gradients only affect one component of the position
            //   i.e. the gradients of F11, F12, F13 only affect the x-component of position, F21, F22, F23 only affect the y-component of position, and F31, F32, F33 only affect the z-component
            _vertices(elem(0), 0) += inv_m1 * (_ldlam(0)*_Q[i](0,0) + _ldlam(1)*_Q[i](0,1) + _ldlam(2)*_Q[i](0,2));
            _vertices(elem(0), 1) += inv_m1 * (_ldlam(3)*_Q[i](0,0) + _ldlam(4)*_Q[i](0,1) + _ldlam(5)*_Q[i](0,2));
            _vertices(elem(0), 2) += inv_m1 * (_ldlam(6)*_Q[i](0,0) + _ldlam(7)*_Q[i](0,1) + _ldlam(8)*_Q[i](0,2));

            _vertices(elem(1), 0) += inv_m2 * (_ldlam(0)*_Q[i](1,0) + _ldlam(1)*_Q[i](1,1) + _ldlam(2)*_Q[i](1,2));
            _vertices(elem(1), 1) += inv_m2 * (_ldlam(3)*_Q[i](1,0) + _ldlam(4)*_Q[i](1,1) + _ldlam(5)*_Q[i](1,2));
            _vertices(elem(1), 2) += inv_m2 * (_ldlam(6)*_Q[i](1,0) + _ldlam(7)*_Q[i](1,1) + _ldlam(8)*_Q[i](1,2));

            _vertices(elem(2), 0) += inv_m3 * (_ldlam(0)*_Q[i](2,0) + _ldlam(1)*_Q[i](2,1) + _ldlam(2)*_Q[i](2,2));
            _vertices(elem(2), 1) += inv_m3 * (_ldlam(3)*_Q[i](2,0) + _ldlam(4)*_Q[i](2,1) + _ldlam(5)*_Q[i](2,2));
            _vertices(elem(2), 2) += inv_m3 * (_ldlam(6)*_Q[i](2,0) + _ldlam(7)*_Q[i](2,1) + _ldlam(8)*_Q[i](2,2));

            _vertices(elem(3), 0) += inv_m4 * (_ldlam(0)*(-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0)) + _ldlam(1)*(-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1)) + _ldlam(2)*(-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2)));
            _vertices(elem(3), 1) += inv_m4 * (_ldlam(3)*(-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0)) + _ldlam(4)*(-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1)) + _ldlam(5)*(-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2))); 
            _vertices(elem(3), 2) += inv_m4 * (_ldlam(6)*(-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0)) + _ldlam(7)*(-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1)) + _ldlam(8)*(-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2)));

            // apply position update from hydrostatic constraint
            _vertices.row(elem(0)) += _lC_h_grads.col(0) * _ldlam(9) * inv_m1;
            _vertices.row(elem(1)) += _lC_h_grads.col(1) * _ldlam(9) * inv_m2;
            _vertices.row(elem(2)) += _lC_h_grads.col(2) * _ldlam(9) * inv_m3;
            _vertices.row(elem(3)) += _lC_h_grads.col(3) * _ldlam(9) * inv_m4;

            // update the overall lambdas using dlam
            lambdas.row(i) += _ldlam;



            
        }

        // if the residual policy is to calculate every iteration, do that
        if (_residual_policy == XPBDResidualPolicy::EVERY_ITERATION)
        {
            if (const OutputSimulation* output_sim = dynamic_cast<const OutputSimulation*>(_sim))
            {
                _calculateResidualsSplitDeviatoric(dt, inertial_positions, lambdas);
                // if we are calculating the residuals every Gauss Seidel iteration, it's likely we want to create
                // a residual vs. iteration number plot, so write info to file
                output_sim->printInfo();
            }
        }
    }

    // if the residual policy is to calculate every substep, do that
    if (_residual_policy == XPBDResidualPolicy::EVERY_SUBSTEP)
    {
        _calculateResidualsSplitDeviatoric(dt, inertial_positions, lambdas);
    }
}

void XPBDMeshObject::_projectConstraintsSplitDeviatoricSimultaneous10G(const double dt)
{
    Eigen::MatrixXd lambdas = Eigen::MatrixXd::Zero(_elements.rows(), 10);

    // remember the inertial positions for residual calculation
    VerticesMat inertial_positions = _vertices;
    VerticesMat x_minus_x_tilde = _vertices - inertial_positions;

    for (unsigned gi = 0; gi < _num_iters; gi++)
    {
        x_minus_x_tilde = _vertices - inertial_positions;
        for (int i = 0; i < _elements.rows(); i++)
        {
            const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);

            // extract the inverse masses of each vertex in the current element
            const double inv_m1 = 1.0/_m[elem(0)];
            const double inv_m2 = 1.0/_m[elem(1)];
            const double inv_m3 = 1.0/_m[elem(2)];
            const double inv_m4 = 1.0/_m[elem(3)];

        
            // compute the deformation gradient
            _computeF(i, _lX, _lF);

            // compute the hydrostatic constraint and its gradients
            const double C_h = _computeHydrostaticConstraint(_lF, _Q[i], _lF_cross, _lC_h_grads);
            // compute the hydrostatic alpha
            const double alpha_h = 1/(_material.lambda() * _vols[i]);
            const double alpha_h_tilde = alpha_h / (dt*dt);


            // compute the deviatoric alpha
            const double alpha_d = 1/(_material.mu() * _vols[i]);
            const double alpha_d_tilde = alpha_d / (dt*dt);


            // form 10x10 LHS matrix inverse (i.e. the M in Mx=b)
            // when solving for dlam, this is (delC M^-1 delC^T + alpha_tilde)
            // we break the 10x10 into 4 matrices:
            //   [ A B ]
            //   [ C D ]
            // A is 9x9 and constant (from the 9 deformation gradient constraints)
            // B = C^T and is 9x1 and encodes the interaction between each deformation gradient constraint and the hydrostatic constraint gradient
            // D is 1x1 and only depends on the hydrostatic constraint gradient
            // now we can compute the inverse according to
            //   [ A^-1 + A^-1B(D-CA^-1B)^-1CA^-1  -A^-1B(D-CA^-1B)^-1 ]
            //   [        -(D-CA^-1B)^-1CA^-1           (D-CA^-1B)^-1  ]
            //
            // things to note:
            // - (D-CA^-1B) is a scalar (i.e. 1x1), so taking its matrix inverse is trivial
            // - A^-1 shows up a lot - however this can be precomputed since A is constant and block diagonal
            // - the products CA^-1 and A^-1B show up a lot as well, and are 9-vectors
            //
            // A has special structure, which is that it is 9x9 block diagonal, with 3 3x3 constant matrices along its diagonal
            // it can be shown that these 3x3 matrices along the diagonal are actually the same for a given A!
            // That is what '_A_inv[i]' is - it is the 3x3 matrix inverse of these block diagonal matrices of A for a given element i
            //
            // Start by forming the B 9x1 matrix
            _lB(0) = inv_m1*_Q[i](0,0)*_lC_h_grads(0,0) + inv_m2*_Q[i](1,0)*_lC_h_grads(0,1) + inv_m3*_Q[i](2,0)*_lC_h_grads(0,2) + inv_m4*(-_Q[i](0,0)-_Q[i](1,0)-_Q[i](2,0))*_lC_h_grads(0,3);
            _lB(1) = inv_m1*_Q[i](0,1)*_lC_h_grads(0,0) + inv_m2*_Q[i](1,1)*_lC_h_grads(0,1) + inv_m3*_Q[i](2,1)*_lC_h_grads(0,2) + inv_m4*(-_Q[i](0,1)-_Q[i](1,1)-_Q[i](2,1))*_lC_h_grads(0,3);
            _lB(2) = inv_m1*_Q[i](0,2)*_lC_h_grads(0,0) + inv_m2*_Q[i](1,2)*_lC_h_grads(0,1) + inv_m3*_Q[i](2,2)*_lC_h_grads(0,2) + inv_m4*(-_Q[i](0,2)-_Q[i](1,2)-_Q[i](2,2))*_lC_h_grads(0,3);
            _lB(3) = inv_m1*_Q[i](0,0)*_lC_h_grads(1,0) + inv_m2*_Q[i](1,0)*_lC_h_grads(1,1) + inv_m3*_Q[i](2,0)*_lC_h_grads(1,2) + inv_m4*(-_Q[i](0,0)-_Q[i](1,0)-_Q[i](2,0))*_lC_h_grads(1,3);
            _lB(4) = inv_m1*_Q[i](0,1)*_lC_h_grads(1,0) + inv_m2*_Q[i](1,1)*_lC_h_grads(1,1) + inv_m3*_Q[i](2,1)*_lC_h_grads(1,2) + inv_m4*(-_Q[i](0,1)-_Q[i](1,1)-_Q[i](2,1))*_lC_h_grads(1,3);
            _lB(5) = inv_m1*_Q[i](0,2)*_lC_h_grads(1,0) + inv_m2*_Q[i](1,2)*_lC_h_grads(1,1) + inv_m3*_Q[i](2,2)*_lC_h_grads(1,2) + inv_m4*(-_Q[i](0,2)-_Q[i](1,2)-_Q[i](2,2))*_lC_h_grads(1,3);
            _lB(6) = inv_m1*_Q[i](0,0)*_lC_h_grads(2,0) + inv_m2*_Q[i](1,0)*_lC_h_grads(2,1) + inv_m3*_Q[i](2,0)*_lC_h_grads(2,2) + inv_m4*(-_Q[i](0,0)-_Q[i](1,0)-_Q[i](2,0))*_lC_h_grads(2,3);
            _lB(7) = inv_m1*_Q[i](0,1)*_lC_h_grads(2,0) + inv_m2*_Q[i](1,1)*_lC_h_grads(2,1) + inv_m3*_Q[i](2,1)*_lC_h_grads(2,2) + inv_m4*(-_Q[i](0,1)-_Q[i](1,1)-_Q[i](2,1))*_lC_h_grads(2,3);
            _lB(8) = inv_m1*_Q[i](0,2)*_lC_h_grads(2,0) + inv_m2*_Q[i](1,2)*_lC_h_grads(2,1) + inv_m3*_Q[i](2,2)*_lC_h_grads(2,2) + inv_m4*(-_Q[i](0,2)-_Q[i](1,2)-_Q[i](2,2))*_lC_h_grads(2,3);

            // compute the D 1x1 matrix - only has to do with the hydrostatic constraint gradient
            const double D =    inv_m1*_lC_h_grads.col(0).squaredNorm() + 
                                inv_m2*_lC_h_grads.col(1).squaredNorm() + 
                                inv_m3*_lC_h_grads.col(2).squaredNorm() + 
                                inv_m4*_lC_h_grads.col(3).squaredNorm() +
                                alpha_h_tilde;
            
            // compute the product CA^-1 - result is a 9-vector
            // we take advantage of the block diagonal structure of A here so we don't have to do full 9x9 matrix multiplication
            _lCA_inv(0) = _lB(Eigen::seq(0,2)).dot(_A_inv[i].col(0));
            _lCA_inv(1) = _lB(Eigen::seq(0,2)).dot(_A_inv[i].col(1));
            _lCA_inv(2) = _lB(Eigen::seq(0,2)).dot(_A_inv[i].col(2));
            _lCA_inv(3) = _lB(Eigen::seq(3,5)).dot(_A_inv[i].col(0));
            _lCA_inv(4) = _lB(Eigen::seq(3,5)).dot(_A_inv[i].col(1));
            _lCA_inv(5) = _lB(Eigen::seq(3,5)).dot(_A_inv[i].col(2));
            _lCA_inv(6) = _lB(Eigen::seq(6,8)).dot(_A_inv[i].col(0));
            _lCA_inv(7) = _lB(Eigen::seq(6,8)).dot(_A_inv[i].col(1));
            _lCA_inv(8) = _lB(Eigen::seq(6,8)).dot(_A_inv[i].col(2));

            // compute the product A^-1B - result is a 9-vector
            // we take advantage of the block diagonal structure of A here so we don't have to do full 9x9 matrix multiplication
            _lA_invB(0) = _lB(Eigen::seq(0,2)).dot(_A_inv[i].row(0));
            _lA_invB(1) = _lB(Eigen::seq(0,2)).dot(_A_inv[i].row(1));
            _lA_invB(2) = _lB(Eigen::seq(0,2)).dot(_A_inv[i].row(2));
            _lA_invB(3) = _lB(Eigen::seq(3,5)).dot(_A_inv[i].row(0));
            _lA_invB(4) = _lB(Eigen::seq(3,5)).dot(_A_inv[i].row(1));
            _lA_invB(5) = _lB(Eigen::seq(3,5)).dot(_A_inv[i].row(2));
            _lA_invB(6) = _lB(Eigen::seq(6,8)).dot(_A_inv[i].row(0));
            _lA_invB(7) = _lB(Eigen::seq(6,8)).dot(_A_inv[i].row(1));
            _lA_invB(8) = _lB(Eigen::seq(6,8)).dot(_A_inv[i].row(2));

            // compute (D-CA^-1B)^-1 - this product shows up a lot in the block matrix inversion formula
            const double inv_D_minus_CA_invB = 1/(D - _lCA_inv.dot(_lB));

            // assemble the 'M' 10x10 system matrix
            // the 9x9 block is A^-1 + A^-1B(D-CA^-1B)^-1CA^-1
            _lM.block<9,9>(0,0) = inv_D_minus_CA_invB * _lA_invB * _lCA_inv.transpose();
            _lM.block<3,3>(0,0) += _A_inv[i];
            _lM.block<3,3>(3,3) += _A_inv[i];
            _lM.block<3,3>(6,6) += _A_inv[i];

            // the 1x9 block in the bottommost row is -(D-CA^-1B)^-1CA^-1
            _lM(9, Eigen::seq(0,8)) = -inv_D_minus_CA_invB * _lCA_inv;

            // the 9x1 block in the rightmost column is -A^-1B(D-CA^-1B)^-1
            _lM(Eigen::seq(0,8), 9) = -inv_D_minus_CA_invB * _lA_invB;
            
            // the 1x1 block in the bottom right corner is (D-CA^-1B)^-1
            _lM(9,9) = inv_D_minus_CA_invB;

            // calculate the RHS of the matrix eq (i.e. -C - alpha_tilde * lambda)
            // and use lowercase b to represent this vector
            const double delC_x_1 = _Q[i](0,0)*x_minus_x_tilde(elem(0),0) +
                                    _Q[i](1,0)*x_minus_x_tilde(elem(1),0) +
                                    _Q[i](2,0)*x_minus_x_tilde(elem(2),0) +
                                    (-_Q[i](0,0)-_Q[i](1,0)-_Q[i](2,0))*x_minus_x_tilde(elem(3),0);
            const double delC_x_2 = _Q[i](0,1)*x_minus_x_tilde(elem(0),0) +
                                    _Q[i](1,1)*x_minus_x_tilde(elem(1),0) +
                                    _Q[i](2,1)*x_minus_x_tilde(elem(2),0) +
                                    (-_Q[i](0,1)-_Q[i](1,1)-_Q[i](2,1))*x_minus_x_tilde(elem(3),0);
            const double delC_x_3 = _Q[i](0,2)*x_minus_x_tilde(elem(0),0) +
                                    _Q[i](1,2)*x_minus_x_tilde(elem(1),0) +
                                    _Q[i](2,2)*x_minus_x_tilde(elem(2),0) +
                                    (-_Q[i](0,2)-_Q[i](1,2)-_Q[i](2,2))*x_minus_x_tilde(elem(3),0);
            const double delC_x_4 = _Q[i](0,0)*x_minus_x_tilde(elem(0),1) +
                                    _Q[i](1,0)*x_minus_x_tilde(elem(1),1) +
                                    _Q[i](2,0)*x_minus_x_tilde(elem(2),1) +
                                    (-_Q[i](0,0)-_Q[i](1,0)-_Q[i](2,0))*x_minus_x_tilde(elem(3),1);
            const double delC_x_5 = _Q[i](0,1)*x_minus_x_tilde(elem(0),1) +
                                    _Q[i](1,1)*x_minus_x_tilde(elem(1),1) +
                                    _Q[i](2,1)*x_minus_x_tilde(elem(2),1) +
                                    (-_Q[i](0,1)-_Q[i](1,1)-_Q[i](2,1))*x_minus_x_tilde(elem(3),1);
            const double delC_x_6 = _Q[i](0,2)*x_minus_x_tilde(elem(0),1) +
                                    _Q[i](1,2)*x_minus_x_tilde(elem(1),1) +
                                    _Q[i](2,2)*x_minus_x_tilde(elem(2),1) +
                                    (-_Q[i](0,2)-_Q[i](1,2)-_Q[i](2,2))*x_minus_x_tilde(elem(3),1);
            const double delC_x_7 = _Q[i](0,0)*x_minus_x_tilde(elem(0),2) +
                                    _Q[i](1,0)*x_minus_x_tilde(elem(1),2) +
                                    _Q[i](2,0)*x_minus_x_tilde(elem(2),2) +
                                    (-_Q[i](0,0)-_Q[i](1,0)-_Q[i](2,0))*x_minus_x_tilde(elem(3),2);
            const double delC_x_8 = _Q[i](0,1)*x_minus_x_tilde(elem(0),2) +
                                    _Q[i](1,1)*x_minus_x_tilde(elem(1),2) +
                                    _Q[i](2,1)*x_minus_x_tilde(elem(2),2) +
                                    (-_Q[i](0,1)-_Q[i](1,1)-_Q[i](2,1))*x_minus_x_tilde(elem(3),2);
            const double delC_x_9 = _Q[i](0,2)*x_minus_x_tilde(elem(0),2) +
                                    _Q[i](1,2)*x_minus_x_tilde(elem(1),2) +
                                    _Q[i](2,2)*x_minus_x_tilde(elem(2),2) +
                                    (-_Q[i](0,2)-_Q[i](1,2)-_Q[i](2,2))*x_minus_x_tilde(elem(3),2);
            const double delC_x_10 = _lC_h_grads.col(0).dot(x_minus_x_tilde.row(elem(0))) +
                                     _lC_h_grads.col(1).dot(x_minus_x_tilde.row(elem(1))) +
                                     _lC_h_grads.col(2).dot(x_minus_x_tilde.row(elem(2))) +
                                     _lC_h_grads.col(3).dot(x_minus_x_tilde.row(elem(3)));

            // calculate the RHS of the matrix eq (i.e. -C - alpha_tilde * lambda)
            _lb(0) = -_lF(0,0) + _g_scaling*delC_x_1;
            _lb(1) = -_lF(0,1) + _g_scaling*delC_x_2;
            _lb(2) = -_lF(0,2) + _g_scaling*delC_x_3;
            _lb(3) = -_lF(1,0) + _g_scaling*delC_x_4;
            _lb(4) = -_lF(1,1) + _g_scaling*delC_x_5;
            _lb(5) = -_lF(1,2) + _g_scaling*delC_x_6;
            _lb(6) = -_lF(2,0) + _g_scaling*delC_x_7;
            _lb(7) = -_lF(2,1) + _g_scaling*delC_x_8;
            _lb(8) = -_lF(2,2) + _g_scaling*delC_x_9;
            _lb(9) = -C_h + _g_scaling*delC_x_10;
            _lb(Eigen::seq(0,2)) += -_g_scaling*_A[i]*lambdas(i,Eigen::seq(0,2)).transpose() + (_g_scaling-1)*alpha_d_tilde*lambdas(i,Eigen::seq(0,2)).transpose();
            _lb(Eigen::seq(3,5)) += -_g_scaling*_A[i]*lambdas(i,Eigen::seq(3,5)).transpose() + (_g_scaling-1)*alpha_d_tilde*lambdas(i,Eigen::seq(3,5)).transpose();
            _lb(Eigen::seq(6,8)) += -_g_scaling*_A[i]*lambdas(i,Eigen::seq(6,8)).transpose() + (_g_scaling-1)*alpha_d_tilde*lambdas(i,Eigen::seq(6,8)).transpose();
            _lb(Eigen::seq(0,8)) += -_g_scaling*_lB*lambdas(i,9);
            _lb(9) += -_g_scaling*_lB.dot(lambdas(i,Eigen::seq(0,8))) - _g_scaling*D*lambdas(i,9) + (_g_scaling-1)*alpha_h_tilde*lambdas(i,9);
 
            // solve for dlams through simple matrix multiplication
            _ldlam = _lM*_lb;

            // apply position updates
            // for the deformation gradient constraints, the constraint gradients only affect one component of the position
            //   i.e. the gradients of F11, F12, F13 only affect the x-component of position, F21, F22, F23 only affect the y-component of position, and F31, F32, F33 only affect the z-component
            Eigen::VectorXd dlam_plus_lam = _ldlam + _g_scaling*lambdas.row(i).transpose(); 
            
            _vertices(elem(0), 0) += inv_m1 * (dlam_plus_lam(0)*_Q[i](0,0) + dlam_plus_lam(1)*_Q[i](0,1) + dlam_plus_lam(2)*_Q[i](0,2));
            _vertices(elem(0), 1) += inv_m1 * (dlam_plus_lam(3)*_Q[i](0,0) + dlam_plus_lam(4)*_Q[i](0,1) + dlam_plus_lam(5)*_Q[i](0,2));
            _vertices(elem(0), 2) += inv_m1 * (dlam_plus_lam(6)*_Q[i](0,0) + dlam_plus_lam(7)*_Q[i](0,1) + dlam_plus_lam(8)*_Q[i](0,2));
            // _vertices.row(elem(0)) -= x_minus_x_tilde.row(elem(0));

            _vertices(elem(1), 0) += inv_m2 * (dlam_plus_lam(0)*_Q[i](1,0) + dlam_plus_lam(1)*_Q[i](1,1) + dlam_plus_lam(2)*_Q[i](1,2));
            _vertices(elem(1), 1) += inv_m2 * (dlam_plus_lam(3)*_Q[i](1,0) + dlam_plus_lam(4)*_Q[i](1,1) + dlam_plus_lam(5)*_Q[i](1,2));
            _vertices(elem(1), 2) += inv_m2 * (dlam_plus_lam(6)*_Q[i](1,0) + dlam_plus_lam(7)*_Q[i](1,1) + dlam_plus_lam(8)*_Q[i](1,2));
            // _vertices.row(elem(1)) -= x_minus_x_tilde.row(elem(1));

            _vertices(elem(2), 0) += inv_m3 * (dlam_plus_lam(0)*_Q[i](2,0) + dlam_plus_lam(1)*_Q[i](2,1) + dlam_plus_lam(2)*_Q[i](2,2));
            _vertices(elem(2), 1) += inv_m3 * (dlam_plus_lam(3)*_Q[i](2,0) + dlam_plus_lam(4)*_Q[i](2,1) + dlam_plus_lam(5)*_Q[i](2,2));
            _vertices(elem(2), 2) += inv_m3 * (dlam_plus_lam(6)*_Q[i](2,0) + dlam_plus_lam(7)*_Q[i](2,1) + dlam_plus_lam(8)*_Q[i](2,2));
            // _vertices.row(elem(2)) -= x_minus_x_tilde.row(elem(2));

            _vertices(elem(3), 0) += inv_m4 * (dlam_plus_lam(0)*(-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0)) + dlam_plus_lam(1)*(-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1)) + dlam_plus_lam(2)*(-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2)));
            _vertices(elem(3), 1) += inv_m4 * (dlam_plus_lam(3)*(-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0)) + dlam_plus_lam(4)*(-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1)) + dlam_plus_lam(5)*(-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2))); 
            _vertices(elem(3), 2) += inv_m4 * (dlam_plus_lam(6)*(-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0)) + dlam_plus_lam(7)*(-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1)) + dlam_plus_lam(8)*(-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2)));
            // _vertices.row(elem(3)) -= x_minus_x_tilde.row(elem(3));

            // apply position update from hydrostatic constraint
            _vertices.row(elem(0)) += _lC_h_grads.col(0) * dlam_plus_lam(9) * inv_m1 - _g_scaling*x_minus_x_tilde.row(elem(0)).transpose();
            _vertices.row(elem(1)) += _lC_h_grads.col(1) * dlam_plus_lam(9) * inv_m2 - _g_scaling*x_minus_x_tilde.row(elem(1)).transpose();
            _vertices.row(elem(2)) += _lC_h_grads.col(2) * dlam_plus_lam(9) * inv_m3 - _g_scaling*x_minus_x_tilde.row(elem(2)).transpose();
            _vertices.row(elem(3)) += _lC_h_grads.col(3) * dlam_plus_lam(9) * inv_m4 - _g_scaling*x_minus_x_tilde.row(elem(3)).transpose();

            // update the overall lambdas using dlam
            lambdas.row(i) += _ldlam;
        }

        // if the residual policy is to calculate every iteration, do that
        if (_residual_policy == XPBDResidualPolicy::EVERY_ITERATION)
        {
            if (const OutputSimulation* output_sim = dynamic_cast<const OutputSimulation*>(_sim))
            {
                _calculateResidualsSplitDeviatoric(dt, inertial_positions, lambdas);
                // if we are calculating the residuals every Gauss Seidel iteration, it's likely we want to create
                // a residual vs. iteration number plot, so write info to file
                output_sim->printInfo();
            }
        }
    }

    // if the residual policy is to calculate every substep, do that
    if (_residual_policy == XPBDResidualPolicy::EVERY_SUBSTEP)
    {
        _calculateResidualsSplitDeviatoric(dt, inertial_positions, lambdas);
    }

}

void XPBDMeshObject::_projectConstraintsFirstOrderSimultaneous(const double dt)
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
            const double b1 = _m[elem(0)] * _mass_to_damping_multiplier;
            const double b2 = _m[elem(1)] * _mass_to_damping_multiplier;
            const double b3 = _m[elem(2)] * _mass_to_damping_multiplier;
            const double b4 = _m[elem(3)] * _mass_to_damping_multiplier;

            _computeF(i, _lX, _lF);

            /** DEVIATORIC CONSTRAINT */
            const double C_d = _computeDeviatoricConstraint(_lF, _Q[i], _lC_d_grads);

            // compute the deviatoric alpha
            const double alpha_d = 1/(_material.mu() * _vols[i]);

            /** HYDROSTATIC CONSTRAINT */

            const double C_h = _computeHydrostaticConstraint(_lF, _Q[i], _lF_cross, _lC_h_grads);

            // compute the hydrostatic alpha
            const double alpha_h = 1/(_material.lambda() * _vols[i]);


            const double inv_b1 = 1/b1;
            const double inv_b2 = 1/b2;
            const double inv_b3 = 1/b3;
            const double inv_b4 = 1/b4;

            // solve the 2x2 system
            const double a11 = inv_b1*_lC_h_grads.col(0).squaredNorm() + 
                               inv_b2*_lC_h_grads.col(1).squaredNorm() + 
                               inv_b3*_lC_h_grads.col(2).squaredNorm() + 
                               inv_b4*_lC_h_grads.col(3).squaredNorm() + 
                               alpha_h/dt;
            const double a12 = inv_b1*_lC_h_grads.col(0).dot(_lC_d_grads.col(0)) +
                               inv_b2*_lC_h_grads.col(1).dot(_lC_d_grads.col(1)) +
                               inv_b3*_lC_h_grads.col(2).dot(_lC_d_grads.col(2)) +
                               inv_b4*_lC_h_grads.col(3).dot(_lC_d_grads.col(3));
            const double a21 = a12;
            const double a22 = inv_b1*_lC_d_grads.col(0).squaredNorm() + 
                               inv_b2*_lC_d_grads.col(1).squaredNorm() + 
                               inv_b3*_lC_d_grads.col(2).squaredNorm() + 
                               inv_b4*_lC_d_grads.col(3).squaredNorm() + 
                               alpha_d/dt;
            const double k1 = -C_h - alpha_h / dt * lambda_hs(i);
            const double k2 = -C_d - alpha_d / dt * lambda_ds(i);

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
        if (_residual_policy == XPBDResidualPolicy::EVERY_ITERATION)
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
    if (_residual_policy == XPBDResidualPolicy::EVERY_SUBSTEP)
    {
        // _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
    }
}

void XPBDMeshObject::_projectConstraintsTrueGaussSeidel(const double dt)
{
    assert(0);
}

void XPBDMeshObject::_projectConstraintsSequentialDistributedG(const double dt)
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
            const double inv_m1 = 1.0/_m[elem(0)];
            const double inv_m2 = 1.0/_m[elem(1)];
            const double inv_m3 = 1.0/_m[elem(2)];
            const double inv_m4 = 1.0/_m[elem(3)];

            const Eigen::Vector3d scaled_g0 = _primary_residual.row(elem(0)) / (2*_num_elements_with_position(elem(0)));
            const Eigen::Vector3d scaled_g1 = _primary_residual.row(elem(1)) / (2*_num_elements_with_position(elem(1)));
            const Eigen::Vector3d scaled_g2 = _primary_residual.row(elem(2)) / (2*_num_elements_with_position(elem(2)));
            const Eigen::Vector3d scaled_g3 = _primary_residual.row(elem(3)) / (2*_num_elements_with_position(elem(3)));


            /** DEVIATORIC CONSTRAINT */

            _computeF(i, _lX, _lF);
            const double C_d = _computeDeviatoricConstraint(_lF, _Q[i], _lC_d_grads);

            // compute the deviatoric alpha
            const double alpha_d = 1/(_material.mu() * _vols[i]);

            const double delC_Minv_delC_d = (inv_m1)*_lC_d_grads.col(0).squaredNorm() + 
                    (inv_m2)*_lC_d_grads.col(1).squaredNorm() + 
                    (inv_m3)*_lC_d_grads.col(2).squaredNorm() + 
                    (inv_m4)*_lC_d_grads.col(3).squaredNorm(); 
            const double delC_Minv_delC_alpha_d = delC_Minv_delC_d + alpha_d/(dt*dt);

            // const double dlam_d = (-C_d - (_g_scaling * delC_Minv_delC_d + alpha_d/(dt*dt)) * lambda_ds(i) + _g_scaling * delC_x_d) / delC_Minv_delC_alpha_d;

            const double dlam_d = (-C_d - alpha_d/(dt*dt)*lambda_ds(i) +
                                inv_m1*(_lC_d_grads.col(0).dot(scaled_g0)) +
                                inv_m2*(_lC_d_grads.col(1).dot(scaled_g1)) + 
                                inv_m3*(_lC_d_grads.col(2).dot(scaled_g2)) +
                                inv_m4*(_lC_d_grads.col(3).dot(scaled_g3))) / delC_Minv_delC_alpha_d;

            _vertices.row(elem(0)) += _lC_d_grads.col(0) * dlam_d * inv_m1 - inv_m1 * scaled_g0;
            _vertices.row(elem(1)) += _lC_d_grads.col(1) * dlam_d * inv_m2 - inv_m2 * scaled_g1;
            _vertices.row(elem(2)) += _lC_d_grads.col(2) * dlam_d * inv_m3 - inv_m3 * scaled_g2;
            _vertices.row(elem(3)) += _lC_d_grads.col(3) * dlam_d * inv_m4 - inv_m4 * scaled_g3;


            /** HYDROSTATIC CONSTRAINT */

            _computeF(i, _lX, _lF);
            const double C_h = _computeHydrostaticConstraint(_lF, _Q[i], _lF_cross, _lC_h_grads);
            // compute the hydrostatic alpha
            const double alpha_h = 1/(_material.lambda() * _vols[i]);
            
            const double delC_Minv_delC_h = (inv_m1)*_lC_h_grads.col(0).squaredNorm() + 
                    (inv_m2)*_lC_h_grads.col(1).squaredNorm() + 
                    (inv_m3)*_lC_h_grads.col(2).squaredNorm() + 
                    (inv_m4)*_lC_h_grads.col(3).squaredNorm();
            const double delC_Minv_delC_alpha_h = delC_Minv_delC_h + alpha_h/(dt*dt);

            // const double dlam_h = (-C_h - (_g_scaling*delC_Minv_delC_h + alpha_h/(dt*dt)) * lambda_hs(i) + _g_scaling * delC_x_h) / delC_Minv_delC_alpha_h;
            const double dlam_h = (-C_h - alpha_h/(dt*dt)*lambda_hs(i) + 
                                inv_m1*(_lC_h_grads.col(0).dot(scaled_g0)) +
                                inv_m2*(_lC_h_grads.col(1).dot(scaled_g1)) +
                                inv_m3*(_lC_h_grads.col(2).dot(scaled_g2)) +
                                inv_m4*(_lC_h_grads.col(3).dot(scaled_g3))) / delC_Minv_delC_alpha_h;

            _vertices.row(elem(0)) += _lC_h_grads.col(0) * dlam_h * inv_m1 - inv_m1 * scaled_g0;
            _vertices.row(elem(1)) += _lC_h_grads.col(1) * dlam_h * inv_m2 - inv_m2 * scaled_g1;
            _vertices.row(elem(2)) += _lC_h_grads.col(2) * dlam_h * inv_m3 - inv_m3 * scaled_g2;
            _vertices.row(elem(3)) += _lC_h_grads.col(3) * dlam_h * inv_m4 - inv_m4 * scaled_g3;

            // update Lagrange multipliers
            lambda_hs(i) += dlam_h;
            lambda_ds(i) += dlam_d;
        }

        _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);

        // if the residual policy is to calculate every iteration, do that
        if (_residual_policy == XPBDResidualPolicy::EVERY_ITERATION)
        {
            if (const OutputSimulation* output_sim = dynamic_cast<const OutputSimulation*>(_sim))
            {
                _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
                // if we are calculating the residuals every Gauss Seidel iteration, it's likely we want to create
                // a residual vs. iteration number plot, so write info to file
                output_sim->printInfo();
            }
        }
    }

    // if the residual policy is to calculate every substep, do that
    if (_residual_policy == XPBDResidualPolicy::EVERY_SUBSTEP)
    {
        _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
    }
}

void XPBDMeshObject::_projectConstraintsSimultaneousDistributedG(const double dt)
{
    // accumulated hydrostatic Lagrange multipliers
    Eigen::VectorXd lambda_hs = Eigen::VectorXd::Zero(_elements.rows());
    // accumulated deviatoric Lagrange multipliers
    Eigen::VectorXd lambda_ds = Eigen::VectorXd::Zero(_elements.rows());

    _primary_residual = VerticesMat::Zero(_vertices.rows(), 3);

    // store positions before constraints are projected
    // this is x-tilde, seen in XPBD eqn 8 - used for computing primary residual
    VerticesMat inertial_positions = _vertices;

    // create vector of element indices to shuffle
    std::vector<int> element_indices(_elements.rows());
    std::iota(std::begin(element_indices), std::end(element_indices), 0);
    auto rd = std::random_device {};
    auto rng = std::default_random_engine {rd()};

    for (unsigned gi = 0; gi < _num_iters; gi++)
    {
        // shuffle the element indices
        std::shuffle(std::begin(element_indices), std::end(element_indices), rng);
        //for (int i = 0; i < _elements.rows(); i++)
        for (const auto& i : element_indices)
        {
            const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);

            // extract masses of each vertex in the current element
            const double m1 = _m[elem(0)];
            const double m2 = _m[elem(1)];
            const double m3 = _m[elem(2)];
            const double m4 = _m[elem(3)];

            _computeF(i, _lX, _lF);

            /** DEVIATORIC CONSTRAINT */
            const double C_d = _computeDeviatoricConstraint(_lF, _Q[i], _lC_d_grads);

            // compute the deviatoric alpha
            const double alpha_d = 1/(_material.mu() * _vols[i]);

            /** HYDROSTATIC CONSTRAINT */

            const double C_h = _computeHydrostaticConstraint(_lF, _Q[i], _lF_cross, _lC_h_grads);

            // compute the hydrostatic alpha
            const double alpha_h = 1/(_material.lambda() * _vols[i]);


            const double inv_m1 = 1/m1;
            const double inv_m2 = 1/m2;
            const double inv_m3 = 1/m3;
            const double inv_m4 = 1/m4;

            // solve the 2x2 system
            const double delC_Minv_delC_h = inv_m1*_lC_h_grads.col(0).squaredNorm() + 
                               inv_m2*_lC_h_grads.col(1).squaredNorm() + 
                               inv_m3*_lC_h_grads.col(2).squaredNorm() + 
                               inv_m4*_lC_h_grads.col(3).squaredNorm();

            const double delC_Minv_delC_d = inv_m1*_lC_d_grads.col(0).squaredNorm() + 
                               inv_m2*_lC_d_grads.col(1).squaredNorm() + 
                               inv_m3*_lC_d_grads.col(2).squaredNorm() + 
                               inv_m4*_lC_d_grads.col(3).squaredNorm();

            const double a11 = delC_Minv_delC_h + 
                               alpha_h/(dt*dt);
            const double a12 = inv_m1*_lC_h_grads.col(0).dot(_lC_d_grads.col(0)) +
                               inv_m2*_lC_h_grads.col(1).dot(_lC_d_grads.col(1)) +
                               inv_m3*_lC_h_grads.col(2).dot(_lC_d_grads.col(2)) +
                               inv_m4*_lC_h_grads.col(3).dot(_lC_d_grads.col(3));
            const double a21 = a12;
            const double a22 = delC_Minv_delC_d + 
                               alpha_d/(dt*dt);

            const Eigen::Vector3d scaled_g0 = _primary_residual.row(elem(0)) / (_num_elements_with_position(elem(0)));
            const Eigen::Vector3d scaled_g1 = _primary_residual.row(elem(1)) / (_num_elements_with_position(elem(1)));
            const Eigen::Vector3d scaled_g2 = _primary_residual.row(elem(2)) / (_num_elements_with_position(elem(2)));
            const Eigen::Vector3d scaled_g3 = _primary_residual.row(elem(3)) / (_num_elements_with_position(elem(3)));

            const double k1 = -C_h - alpha_h/(dt*dt)*lambda_hs(i) + 
                                inv_m1*(_lC_h_grads.col(0).dot(scaled_g0)) +
                                inv_m2*(_lC_h_grads.col(1).dot(scaled_g1)) +
                                inv_m3*(_lC_h_grads.col(2).dot(scaled_g2)) +
                                inv_m4*(_lC_h_grads.col(3).dot(scaled_g3));

            const double k2 = -C_d - alpha_d/(dt*dt)*lambda_ds(i) +
                                inv_m1*(_lC_d_grads.col(0).dot(scaled_g0)) +
                                inv_m2*(_lC_d_grads.col(1).dot(scaled_g1)) + 
                                inv_m3*(_lC_d_grads.col(2).dot(scaled_g2)) +
                                inv_m4*(_lC_d_grads.col(3).dot(scaled_g3));

            const double detA = a11*a22 - a21*a12;

            const double dlam_h = (k1*a22 - k2*a12) / detA;
            const double dlam_d = (a11*k2 - a21*k1) / detA;


            // update vertex positions
            _vertices.row(elem(0)) += _lC_h_grads.col(0) * dlam_h * inv_m1 + _lC_d_grads.col(0) * dlam_d * inv_m1 - inv_m1 * scaled_g0;
            _vertices.row(elem(1)) += _lC_h_grads.col(1) * dlam_h * inv_m2 + _lC_d_grads.col(1) * dlam_d * inv_m2 - inv_m2 * scaled_g1;
            _vertices.row(elem(2)) += _lC_h_grads.col(2) * dlam_h * inv_m3 + _lC_d_grads.col(2) * dlam_d * inv_m3 - inv_m3 * scaled_g2;
            _vertices.row(elem(3)) += _lC_h_grads.col(3) * dlam_h * inv_m4 + _lC_d_grads.col(3) * dlam_d * inv_m4 - inv_m4 * scaled_g3;
            

            // update Lagrange multipliers
            lambda_hs(i) += dlam_h;
            lambda_ds(i) += dlam_d;
            // _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);

        }

        // don't calculate the residual on the last Gauss-Seidel iteration
        if (gi < _num_iters-1)
        {
            _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);

        }
        
        // if the residual policy is to calculate every iteration, do that
        if (_residual_policy == XPBDResidualPolicy::EVERY_ITERATION)
        {
            if (const OutputSimulation* output_sim = dynamic_cast<const OutputSimulation*>(_sim))
            {
                // if we are calculating the residuals every Gauss Seidel iteration, it's likely we want to create
                // a residual vs. iteration number plot, so write info to file
                output_sim->printInfo();
            }
        }
    }

    // if the residual policy is to calculate every substep, do that
    if (_residual_policy == XPBDResidualPolicy::EVERY_SUBSTEP)
    {
        _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
    }
}

void XPBDMeshObject::_projectConstraintsSimultaneousDistributedGInitLambda(const double dt)
{
    // accumulated hydrostatic Lagrange multipliers
    Eigen::VectorXd lambda_hs = _initial_lambda_hs;
    // accumulated deviatoric Lagrange multipliers
    Eigen::VectorXd lambda_ds = _initial_lambda_ds;
    
    // store positions before constraints are projected
    // this is x-tilde, seen in XPBD eqn 8 - used for computing primary residual
    VerticesMat inertial_positions = _vertices;

    // calculate the residual initially since g is nonzero (since we initialized the lambdas)
    _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);

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

            _computeF(i, _lX, _lF);

            /** DEVIATORIC CONSTRAINT */
            const double C_d = _computeDeviatoricConstraint(_lF, _Q[i], _lC_d_grads);

            // compute the deviatoric alpha
            const double alpha_d = 1/(_material.mu() * _vols[i]);

            /** HYDROSTATIC CONSTRAINT */

            const double C_h = _computeHydrostaticConstraint(_lF, _Q[i], _lF_cross, _lC_h_grads);

            // compute the hydrostatic alpha
            const double alpha_h = 1/(_material.lambda() * _vols[i]);


            const double inv_m1 = 1/m1;
            const double inv_m2 = 1/m2;
            const double inv_m3 = 1/m3;
            const double inv_m4 = 1/m4;

            // const Eigen::Vector3d x_minus_x_tilde1 = _vertices.row(elem(0)) - inertial_positions.row(elem(0));
            // const Eigen::Vector3d x_minus_x_tilde2 = _vertices.row(elem(1)) - inertial_positions.row(elem(1));
            // const Eigen::Vector3d x_minus_x_tilde3 = _vertices.row(elem(2)) - inertial_positions.row(elem(2));
            // const Eigen::Vector3d x_minus_x_tilde4 = _vertices.row(elem(3)) - inertial_positions.row(elem(3));

            // solve the 2x2 system
            const double delC_Minv_delC_h = inv_m1*_lC_h_grads.col(0).squaredNorm() + 
                               inv_m2*_lC_h_grads.col(1).squaredNorm() + 
                               inv_m3*_lC_h_grads.col(2).squaredNorm() + 
                               inv_m4*_lC_h_grads.col(3).squaredNorm();

            const double delC_Minv_delC_d = inv_m1*_lC_d_grads.col(0).squaredNorm() + 
                               inv_m2*_lC_d_grads.col(1).squaredNorm() + 
                               inv_m3*_lC_d_grads.col(2).squaredNorm() + 
                               inv_m4*_lC_d_grads.col(3).squaredNorm();

            const double a11 = delC_Minv_delC_h + 
                               alpha_h/(dt*dt);
            const double a12 = inv_m1*_lC_h_grads.col(0).dot(_lC_d_grads.col(0)) +
                               inv_m2*_lC_h_grads.col(1).dot(_lC_d_grads.col(1)) +
                               inv_m3*_lC_h_grads.col(2).dot(_lC_d_grads.col(2)) +
                               inv_m4*_lC_h_grads.col(3).dot(_lC_d_grads.col(3));
            const double a21 = a12;
            const double a22 = delC_Minv_delC_d + 
                               alpha_d/(dt*dt);

            const Eigen::Vector3d scaled_g0 = _primary_residual.row(elem(0)) / (2*_num_elements_with_position(elem(0)));
            const Eigen::Vector3d scaled_g1 = _primary_residual.row(elem(1)) / (2*_num_elements_with_position(elem(1)));
            const Eigen::Vector3d scaled_g2 = _primary_residual.row(elem(2)) / (2*_num_elements_with_position(elem(2)));
            const Eigen::Vector3d scaled_g3 = _primary_residual.row(elem(3)) / (2*_num_elements_with_position(elem(3)));

            const double k1 = -C_h - alpha_h/(dt*dt)*lambda_hs(i) + 
                                inv_m1*(_lC_h_grads.col(0).dot(scaled_g0)) +
                                inv_m2*(_lC_h_grads.col(1).dot(scaled_g1)) +
                                inv_m3*(_lC_h_grads.col(2).dot(scaled_g2)) +
                                inv_m4*(_lC_h_grads.col(3).dot(scaled_g3));

            const double k2 = -C_d - alpha_d/(dt*dt)*lambda_ds(i) +
                                inv_m1*(_lC_d_grads.col(0).dot(scaled_g0)) +
                                inv_m2*(_lC_d_grads.col(1).dot(scaled_g1)) + 
                                inv_m3*(_lC_d_grads.col(2).dot(scaled_g2)) +
                                inv_m4*(_lC_d_grads.col(3).dot(scaled_g3));

            const double detA = a11*a22 - a21*a12;

            const double dlam_h = (k1*a22 - k2*a12) / detA;
            const double dlam_d = (a11*k2 - a21*k1) / detA;


            // update vertex positions
            _vertices.row(elem(0)) += _lC_h_grads.col(0) * dlam_h * inv_m1 + _lC_d_grads.col(0) * dlam_d * inv_m1 - inv_m1 * scaled_g0;
            _vertices.row(elem(1)) += _lC_h_grads.col(1) * dlam_h * inv_m2 + _lC_d_grads.col(1) * dlam_d * inv_m2 - inv_m2 * scaled_g1;
            _vertices.row(elem(2)) += _lC_h_grads.col(2) * dlam_h * inv_m3 + _lC_d_grads.col(2) * dlam_d * inv_m3 - inv_m3 * scaled_g2;
            _vertices.row(elem(3)) += _lC_h_grads.col(3) * dlam_h * inv_m4 + _lC_d_grads.col(3) * dlam_d * inv_m4 - inv_m4 * scaled_g3;
            

            // update Lagrange multipliers
            lambda_hs(i) += dlam_h;
            lambda_ds(i) += dlam_d;
            // _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);

        }

        // don't calculate the residual on the last Gauss-Seidel iteration
        if (gi < _num_iters-1)
        {
            _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);

        }
        
        // if the residual policy is to calculate every iteration, do that
        if (_residual_policy == XPBDResidualPolicy::EVERY_ITERATION)
        {
            if (const OutputSimulation* output_sim = dynamic_cast<const OutputSimulation*>(_sim))
            {
                // if we are calculating the residuals every Gauss Seidel iteration, it's likely we want to create
                // a residual vs. iteration number plot, so write info to file
                output_sim->printInfo();
            }
        }
    }

    _initial_lambda_hs = lambda_hs;
    _initial_lambda_ds = lambda_ds;

    // if the residual policy is to calculate every substep, do that
    if (_residual_policy == XPBDResidualPolicy::EVERY_SUBSTEP)
    {
        _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
    }
}

void XPBDMeshObject::_projectConstraintsSimultaneousDampedNewton(const double dt)
{
    // accumulated hydrostatic Lagrange multipliers
    Eigen::VectorXd lambda_hs = Eigen::VectorXd::Zero(_elements.rows());
    // accumulated deviatoric Lagrange multipliers
    Eigen::VectorXd lambda_ds = Eigen::VectorXd::Zero(_elements.rows());

    _primary_residual = VerticesMat::Zero(_vertices.rows(), 3);

    // store positions before constraints are projected
    // this is x-tilde, seen in XPBD eqn 8 - used for computing primary residual
    VerticesMat inertial_positions = _vertices;

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

            _computeF(i, _lX, _lF);

            /** DEVIATORIC CONSTRAINT */
            const double C_d = _computeDeviatoricConstraint(_lF, _Q[i], _lC_d_grads);

            // compute the deviatoric alpha
            const double alpha_d = 1/(_material.mu() * _vols[i]);

            /** HYDROSTATIC CONSTRAINT */

            const double C_h = _computeHydrostaticConstraint(_lF, _Q[i], _lF_cross, _lC_h_grads);

            // compute the hydrostatic alpha
            const double alpha_h = 1/(_material.lambda() * _vols[i]);


            const double inv_m1 = 1/m1;
            const double inv_m2 = 1/m2;
            const double inv_m3 = 1/m3;
            const double inv_m4 = 1/m4;

            // solve the 2x2 system
            const double delC_Minv_delC_h = inv_m1*_lC_h_grads.col(0).squaredNorm() + 
                               inv_m2*_lC_h_grads.col(1).squaredNorm() + 
                               inv_m3*_lC_h_grads.col(2).squaredNorm() + 
                               inv_m4*_lC_h_grads.col(3).squaredNorm();

            const double delC_Minv_delC_d = inv_m1*_lC_d_grads.col(0).squaredNorm() + 
                               inv_m2*_lC_d_grads.col(1).squaredNorm() + 
                               inv_m3*_lC_d_grads.col(2).squaredNorm() + 
                               inv_m4*_lC_d_grads.col(3).squaredNorm();

            const double a11 = delC_Minv_delC_h + 
                               alpha_h/(dt*dt);
            const double a12 = inv_m1*_lC_h_grads.col(0).dot(_lC_d_grads.col(0)) +
                               inv_m2*_lC_h_grads.col(1).dot(_lC_d_grads.col(1)) +
                               inv_m3*_lC_h_grads.col(2).dot(_lC_d_grads.col(2)) +
                               inv_m4*_lC_h_grads.col(3).dot(_lC_d_grads.col(3));
            const double a21 = a12;
            const double a22 = delC_Minv_delC_d + 
                               alpha_d/(dt*dt);

            const Eigen::Vector3d scaled_g0 = _primary_residual.row(elem(0)) * _g_scaling; // / (2*_num_elements_with_position(elem(0)));
            const Eigen::Vector3d scaled_g1 = _primary_residual.row(elem(1)) * _g_scaling; // / (2*_num_elements_with_position(elem(1)));
            const Eigen::Vector3d scaled_g2 = _primary_residual.row(elem(2)) * _g_scaling; // / (2*_num_elements_with_position(elem(2)));
            const Eigen::Vector3d scaled_g3 = _primary_residual.row(elem(3)) * _g_scaling; // / (2*_num_elements_with_position(elem(3)));

            const double k1 = _g_scaling*(-C_h - alpha_h/(dt*dt)*lambda_hs(i)) + 
                                inv_m1*(_lC_h_grads.col(0).dot(scaled_g0)) +
                                inv_m2*(_lC_h_grads.col(1).dot(scaled_g1)) +
                                inv_m3*(_lC_h_grads.col(2).dot(scaled_g2)) +
                                inv_m4*(_lC_h_grads.col(3).dot(scaled_g3));

            const double k2 = _g_scaling*(-C_d - alpha_d/(dt*dt)*lambda_ds(i)) +
                                inv_m1*(_lC_d_grads.col(0).dot(scaled_g0)) +
                                inv_m2*(_lC_d_grads.col(1).dot(scaled_g1)) + 
                                inv_m3*(_lC_d_grads.col(2).dot(scaled_g2)) +
                                inv_m4*(_lC_d_grads.col(3).dot(scaled_g3));

            const double detA = a11*a22 - a21*a12;

            const double dlam_h = (k1*a22 - k2*a12) / detA;
            const double dlam_d = (a11*k2 - a21*k1) / detA;


            // update vertex positions
            _vertices.row(elem(0)) += _lC_h_grads.col(0) * dlam_h * inv_m1 + _lC_d_grads.col(0) * dlam_d * inv_m1 - inv_m1 * scaled_g0;
            _vertices.row(elem(1)) += _lC_h_grads.col(1) * dlam_h * inv_m2 + _lC_d_grads.col(1) * dlam_d * inv_m2 - inv_m2 * scaled_g1;
            _vertices.row(elem(2)) += _lC_h_grads.col(2) * dlam_h * inv_m3 + _lC_d_grads.col(2) * dlam_d * inv_m3 - inv_m3 * scaled_g2;
            _vertices.row(elem(3)) += _lC_h_grads.col(3) * dlam_h * inv_m4 + _lC_d_grads.col(3) * dlam_d * inv_m4 - inv_m4 * scaled_g3;
            

            // update Lagrange multipliers
            lambda_hs(i) += dlam_h;
            lambda_ds(i) += dlam_d;
            // _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);

        }

        // don't calculate the residual on the last Gauss-Seidel iteration
        if (gi < _num_iters-1)
        {
            _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);

        }
        
        // if the residual policy is to calculate every iteration, do that
        if (_residual_policy == XPBDResidualPolicy::EVERY_ITERATION)
        {
            if (const OutputSimulation* output_sim = dynamic_cast<const OutputSimulation*>(_sim))
            {
                // if we are calculating the residuals every Gauss Seidel iteration, it's likely we want to create
                // a residual vs. iteration number plot, so write info to file
                output_sim->printInfo();
            }
        }
    }

    // if the residual policy is to calculate every substep, do that
    if (_residual_policy == XPBDResidualPolicy::EVERY_SUBSTEP)
    {
        _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
    }
}

void XPBDMeshObject::_projectConstraintsSimultaneousDistributedGOrdered(const double dt)
{
    // accumulated hydrostatic Lagrange multipliers
    Eigen::VectorXd lambda_hs = Eigen::VectorXd::Zero(_elements.rows());
    // accumulated deviatoric Lagrange multipliers
    Eigen::VectorXd lambda_ds = Eigen::VectorXd::Zero(_elements.rows());

    _primary_residual = VerticesMat::Zero(_vertices.rows(), 3);

    // store positions before constraints are projected
    // this is x-tilde, seen in XPBD eqn 8 - used for computing primary residual
    VerticesMat inertial_positions = _vertices;

    for (unsigned gi = 0; gi < _num_iters; gi++)
    {
        //for (int i = 0; i < _elements.rows(); i++)
        for (const auto& i : _element_order)
        {
            const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);

            // extract masses of each vertex in the current element
            const double m1 = _m[elem(0)];
            const double m2 = _m[elem(1)];
            const double m3 = _m[elem(2)];
            const double m4 = _m[elem(3)];

            _computeF(i, _lX, _lF);

            /** DEVIATORIC CONSTRAINT */
            const double C_d = _computeDeviatoricConstraint(_lF, _Q[i], _lC_d_grads);

            // compute the deviatoric alpha
            const double alpha_d = 1/(_material.mu() * _vols[i]);

            /** HYDROSTATIC CONSTRAINT */

            const double C_h = _computeHydrostaticConstraint(_lF, _Q[i], _lF_cross, _lC_h_grads);

            // compute the hydrostatic alpha
            const double alpha_h = 1/(_material.lambda() * _vols[i]);


            const double inv_m1 = 1/m1;
            const double inv_m2 = 1/m2;
            const double inv_m3 = 1/m3;
            const double inv_m4 = 1/m4;

            // solve the 2x2 system
            const double delC_Minv_delC_h = inv_m1*_lC_h_grads.col(0).squaredNorm() + 
                               inv_m2*_lC_h_grads.col(1).squaredNorm() + 
                               inv_m3*_lC_h_grads.col(2).squaredNorm() + 
                               inv_m4*_lC_h_grads.col(3).squaredNorm();

            const double delC_Minv_delC_d = inv_m1*_lC_d_grads.col(0).squaredNorm() + 
                               inv_m2*_lC_d_grads.col(1).squaredNorm() + 
                               inv_m3*_lC_d_grads.col(2).squaredNorm() + 
                               inv_m4*_lC_d_grads.col(3).squaredNorm();

            const double a11 = delC_Minv_delC_h + 
                               alpha_h/(dt*dt);
            const double a12 = inv_m1*_lC_h_grads.col(0).dot(_lC_d_grads.col(0)) +
                               inv_m2*_lC_h_grads.col(1).dot(_lC_d_grads.col(1)) +
                               inv_m3*_lC_h_grads.col(2).dot(_lC_d_grads.col(2)) +
                               inv_m4*_lC_h_grads.col(3).dot(_lC_d_grads.col(3));
            const double a21 = a12;
            const double a22 = delC_Minv_delC_d + 
                               alpha_d/(dt*dt);

            const Eigen::Vector3d scaled_g0 = _primary_residual.row(elem(0)) / (2*_num_elements_with_position(elem(0)));
            const Eigen::Vector3d scaled_g1 = _primary_residual.row(elem(1)) / (2*_num_elements_with_position(elem(1)));
            const Eigen::Vector3d scaled_g2 = _primary_residual.row(elem(2)) / (2*_num_elements_with_position(elem(2)));
            const Eigen::Vector3d scaled_g3 = _primary_residual.row(elem(3)) / (2*_num_elements_with_position(elem(3)));

            const double k1 = -C_h - alpha_h/(dt*dt)*lambda_hs(i) + 
                                inv_m1*(_lC_h_grads.col(0).dot(scaled_g0)) +
                                inv_m2*(_lC_h_grads.col(1).dot(scaled_g1)) +
                                inv_m3*(_lC_h_grads.col(2).dot(scaled_g2)) +
                                inv_m4*(_lC_h_grads.col(3).dot(scaled_g3));

            const double k2 = -C_d - alpha_d/(dt*dt)*lambda_ds(i) +
                                inv_m1*(_lC_d_grads.col(0).dot(scaled_g0)) +
                                inv_m2*(_lC_d_grads.col(1).dot(scaled_g1)) + 
                                inv_m3*(_lC_d_grads.col(2).dot(scaled_g2)) +
                                inv_m4*(_lC_d_grads.col(3).dot(scaled_g3));

            const double detA = a11*a22 - a21*a12;

            const double dlam_h = (k1*a22 - k2*a12) / detA;
            const double dlam_d = (a11*k2 - a21*k1) / detA;


            // update vertex positions
            _vertices.row(elem(0)) += _lC_h_grads.col(0) * dlam_h * inv_m1 + _lC_d_grads.col(0) * dlam_d * inv_m1 - inv_m1 * scaled_g0;
            _vertices.row(elem(1)) += _lC_h_grads.col(1) * dlam_h * inv_m2 + _lC_d_grads.col(1) * dlam_d * inv_m2 - inv_m2 * scaled_g1;
            _vertices.row(elem(2)) += _lC_h_grads.col(2) * dlam_h * inv_m3 + _lC_d_grads.col(2) * dlam_d * inv_m3 - inv_m3 * scaled_g2;
            _vertices.row(elem(3)) += _lC_h_grads.col(3) * dlam_h * inv_m4 + _lC_d_grads.col(3) * dlam_d * inv_m4 - inv_m4 * scaled_g3;
            

            // update Lagrange multipliers
            lambda_hs(i) += dlam_h;
            lambda_ds(i) += dlam_d;
            // _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);

        }

        // don't calculate the residual on the last Gauss-Seidel iteration
        if (gi < _num_iters-1)
        {
            _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);

        }
        
        // if the residual policy is to calculate every iteration, do that
        if (_residual_policy == XPBDResidualPolicy::EVERY_ITERATION)
        {
            if (const OutputSimulation* output_sim = dynamic_cast<const OutputSimulation*>(_sim))
            {
                // if we are calculating the residuals every Gauss Seidel iteration, it's likely we want to create
                // a residual vs. iteration number plot, so write info to file
                output_sim->printInfo();
            }
        }
    }

    // if the residual policy is to calculate every substep, do that
    if (_residual_policy == XPBDResidualPolicy::EVERY_SUBSTEP)
    {
        _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
    }
}


void XPBDMeshObject::_projectConstraintsSimultaneousJacobi(const double dt)
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
            const double m1 = _m[elem(0)];
            const double m2 = _m[elem(1)];
            const double m3 = _m[elem(2)];
            const double m4 = _m[elem(3)];

            _computeF(i, _lX, _lF);

            /** DEVIATORIC CONSTRAINT */
            const double C_d = _computeDeviatoricConstraint(_lF, _Q[i], _lC_d_grads);

            // compute the deviatoric alpha
            const double alpha_d = 1/(_material.mu() * _vols[i]);

            /** HYDROSTATIC CONSTRAINT */

            const double C_h = _computeHydrostaticConstraint(_lF, _Q[i], _lF_cross, _lC_h_grads);

            // compute the hydrostatic alpha
            const double alpha_h = 1/(_material.lambda() * _vols[i]);


            const double inv_m1 = 1/m1;
            const double inv_m2 = 1/m2;
            const double inv_m3 = 1/m3;
            const double inv_m4 = 1/m4;

            // solve the 2x2 system
            const double a11 = inv_m1*_lC_h_grads.col(0).squaredNorm() + 
                               inv_m2*_lC_h_grads.col(1).squaredNorm() + 
                               inv_m3*_lC_h_grads.col(2).squaredNorm() + 
                               inv_m4*_lC_h_grads.col(3).squaredNorm() + 
                               alpha_h/(dt*dt);
            const double a12 = inv_m1*_lC_h_grads.col(0).dot(_lC_d_grads.col(0)) +
                               inv_m2*_lC_h_grads.col(1).dot(_lC_d_grads.col(1)) +
                               inv_m3*_lC_h_grads.col(2).dot(_lC_d_grads.col(2)) +
                               inv_m4*_lC_h_grads.col(3).dot(_lC_d_grads.col(3));
            const double a21 = a12;
            const double a22 = inv_m1*_lC_d_grads.col(0).squaredNorm() + 
                               inv_m2*_lC_d_grads.col(1).squaredNorm() + 
                               inv_m3*_lC_d_grads.col(2).squaredNorm() + 
                               inv_m4*_lC_d_grads.col(3).squaredNorm() + 
                               alpha_d/(dt*dt);
            const double k1 = (-C_h - alpha_h / (dt*dt) * lambda_hs(i));
            const double k2 = (-C_d - alpha_d / (dt*dt) * lambda_ds(i));

            const double detA = a11*a22 - a21*a12;

            const double dlam_h = (k1*a22 - k2*a12) / detA;
            const double dlam_d = (a11*k2 - a21*k1) / detA;

            // update Lagrange multipliers
            lambda_hs(i) += dlam_h;
            lambda_ds(i) += dlam_d;

            const double scaling1 = 1.0/_num_elements_with_position(elem(0));
            const double scaling2 = 1.0/_num_elements_with_position(elem(1));
            const double scaling3 = 1.0/_num_elements_with_position(elem(2));
            const double scaling4 = 1.0/_num_elements_with_position(elem(3));

            // update nodal forces (delC*lambda)
            dx.row(elem(0)) += scaling1 * inv_m1 * (_lC_h_grads.col(0) * dlam_h + _lC_d_grads.col(0) * dlam_d);
            dx.row(elem(1)) += scaling2 * inv_m2 * (_lC_h_grads.col(1) * dlam_h + _lC_d_grads.col(1) * dlam_d);
            dx.row(elem(2)) += scaling3 * inv_m3 * (_lC_h_grads.col(2) * dlam_h + _lC_d_grads.col(2) * dlam_d);
            dx.row(elem(3)) += scaling4 * inv_m4 * (_lC_h_grads.col(3) * dlam_h + _lC_d_grads.col(3) * dlam_d);
        }

        _vertices += dx;
        
        // if the residual policy is to calculate every iteration, do that
        if (_residual_policy == XPBDResidualPolicy::EVERY_ITERATION)
        {
            if (const OutputSimulation* output_sim = dynamic_cast<const OutputSimulation*>(_sim))
            {
                _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
                // if we are calculating the residuals every Gauss Seidel iteration, it's likely we want to create
                // a residual vs. iteration number plot, so write info to file
                output_sim->printInfo();
            }
        }
    }

    // if the residual policy is to calculate every substep, do that
    if (_residual_policy == XPBDResidualPolicy::EVERY_SUBSTEP)
    {
        _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
    }
}

void XPBDMeshObject::_projectConstraintsSimultaneousConvergentJacobi(const double dt)
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
            const double m1 = _m[elem(0)];
            const double m2 = _m[elem(1)];
            const double m3 = _m[elem(2)];
            const double m4 = _m[elem(3)];

            _computeF(i, _lX, _lF);

            /** DEVIATORIC CONSTRAINT */
            const double C_d = _computeDeviatoricConstraint(_lF, _Q[i], _lC_d_grads);

            // compute the deviatoric alpha
            const double alpha_d = 1/(_material.mu() * _vols[i]);

            /** HYDROSTATIC CONSTRAINT */

            const double C_h = _computeHydrostaticConstraint(_lF, _Q[i], _lF_cross, _lC_h_grads);

            // compute the hydrostatic alpha
            const double alpha_h = 1/(_material.lambda() * _vols[i]);


            const double inv_m1 = 1/m1;
            const double inv_m2 = 1/m2;
            const double inv_m3 = 1/m3;
            const double inv_m4 = 1/m4;

            // solve the 2x2 system
            const double a11 = inv_m1*_lC_h_grads.col(0).squaredNorm() + 
                               inv_m2*_lC_h_grads.col(1).squaredNorm() + 
                               inv_m3*_lC_h_grads.col(2).squaredNorm() + 
                               inv_m4*_lC_h_grads.col(3).squaredNorm() + 
                               alpha_h/(dt*dt);
            const double a12 = inv_m1*_lC_h_grads.col(0).dot(_lC_d_grads.col(0)) +
                               inv_m2*_lC_h_grads.col(1).dot(_lC_d_grads.col(1)) +
                               inv_m3*_lC_h_grads.col(2).dot(_lC_d_grads.col(2)) +
                               inv_m4*_lC_h_grads.col(3).dot(_lC_d_grads.col(3));
            const double a21 = a12;
            const double a22 = inv_m1*_lC_d_grads.col(0).squaredNorm() + 
                               inv_m2*_lC_d_grads.col(1).squaredNorm() + 
                               inv_m3*_lC_d_grads.col(2).squaredNorm() + 
                               inv_m4*_lC_d_grads.col(3).squaredNorm() + 
                               alpha_d/(dt*dt);
            const double k1 = (-C_h - alpha_h / (dt*dt) * lambda_hs(i));
            const double k2 = (-C_d - alpha_d / (dt*dt) * lambda_ds(i));

            const double detA = a11*a22 - a21*a12;

            const double dlam_h = (k1*a22 - k2*a12) / detA;
            const double dlam_d = (a11*k2 - a21*k1) / detA;

            // update Lagrange multipliers
            lambda_hs(i) += dlam_h;
            lambda_ds(i) += dlam_d;


            // update nodal forces (delC*lambda)
            nodal_forces.row(elem(0)) += inv_m1 * (_lC_h_grads.col(0) * lambda_hs(i) + _lC_d_grads.col(0) * lambda_ds(i));
            nodal_forces.row(elem(1)) += inv_m2 * (_lC_h_grads.col(1) * lambda_hs(i) + _lC_d_grads.col(1) * lambda_ds(i));
            nodal_forces.row(elem(2)) += inv_m3 * (_lC_h_grads.col(2) * lambda_hs(i) + _lC_d_grads.col(2) * lambda_ds(i));
            nodal_forces.row(elem(3)) += inv_m4 * (_lC_h_grads.col(3) * lambda_hs(i) + _lC_d_grads.col(3) * lambda_ds(i));
        }

        _vertices = inertial_positions + nodal_forces;
        
        // if the residual policy is to calculate every iteration, do that
        if (_residual_policy == XPBDResidualPolicy::EVERY_ITERATION)
        {
            if (const OutputSimulation* output_sim = dynamic_cast<const OutputSimulation*>(_sim))
            {
                _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
                // if we are calculating the residuals every Gauss Seidel iteration, it's likely we want to create
                // a residual vs. iteration number plot, so write info to file
                output_sim->printInfo();
            }
        }
    }

    // if the residual policy is to calculate every substep, do that
    if (_residual_policy == XPBDResidualPolicy::EVERY_SUBSTEP)
    {
        _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
    }
}

void XPBDMeshObject::_projectConstraintsSimultaneousConvergentJacobiLineSearch(const double dt)
{
    // accumulated hydrostatic Lagrange multipliers
    Eigen::VectorXd lambda_hs = Eigen::VectorXd::Zero(_elements.rows());
    // accumulated deviatoric Lagrange multipliers
    Eigen::VectorXd lambda_ds = Eigen::VectorXd::Zero(_elements.rows());

    // accumulated hydrostatic Lagrange multipliers
    Eigen::VectorXd dlam_ds = Eigen::VectorXd::Zero(_elements.rows());
    // accumulated deviatoric Lagrange multipliers
    Eigen::VectorXd dlam_hs = Eigen::VectorXd::Zero(_elements.rows());

    // store positions before constraints are projected
    // this is x-tilde, seen in XPBD eqn 8 - used for computing primary residual
    VerticesMat inertial_positions = _vertices;
    VerticesMat latest_vertices = _vertices;

    _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
    double last_c_res_rms = _constraint_residual_rms;

    const int max_line_search_iters = 10;
    VerticesMat nodal_forces = VerticesMat::Zero(_vertices.rows(), 3);

    for (unsigned gi = 0; gi < _num_iters; gi++)
    {
        double s = 1;
        double c_res_rms = 0;

        for (int li = 0; li < max_line_search_iters; li++)
        {
            nodal_forces = VerticesMat::Zero(_vertices.rows(), 3);

            for (int i = 0; i < _elements.rows(); i++)
            {
                const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);

                // extract masses of each vertex in the current element
                const double m1 = _m[elem(0)];
                const double m2 = _m[elem(1)];
                const double m3 = _m[elem(2)];
                const double m4 = _m[elem(3)];

                _computeF(i, _lX, _lF);

                /** DEVIATORIC CONSTRAINT */
                const double C_d = _computeDeviatoricConstraint(_lF, _Q[i], _lC_d_grads);

                // compute the deviatoric alpha
                const double alpha_d = 1/(_material.mu() * _vols[i]);

                /** HYDROSTATIC CONSTRAINT */

                const double C_h = _computeHydrostaticConstraint(_lF, _Q[i], _lF_cross, _lC_h_grads);

                // compute the hydrostatic alpha
                const double alpha_h = 1/(_material.lambda() * _vols[i]);


                const double inv_m1 = 1/m1;
                const double inv_m2 = 1/m2;
                const double inv_m3 = 1/m3;
                const double inv_m4 = 1/m4;

                // solve the 2x2 system
                const double a11 = inv_m1*_lC_h_grads.col(0).squaredNorm() + 
                                inv_m2*_lC_h_grads.col(1).squaredNorm() + 
                                inv_m3*_lC_h_grads.col(2).squaredNorm() + 
                                inv_m4*_lC_h_grads.col(3).squaredNorm() + 
                                alpha_h/(dt*dt);
                const double a12 = inv_m1*_lC_h_grads.col(0).dot(_lC_d_grads.col(0)) +
                                inv_m2*_lC_h_grads.col(1).dot(_lC_d_grads.col(1)) +
                                inv_m3*_lC_h_grads.col(2).dot(_lC_d_grads.col(2)) +
                                inv_m4*_lC_h_grads.col(3).dot(_lC_d_grads.col(3));
                const double a21 = a12;
                const double a22 = inv_m1*_lC_d_grads.col(0).squaredNorm() + 
                                inv_m2*_lC_d_grads.col(1).squaredNorm() + 
                                inv_m3*_lC_d_grads.col(2).squaredNorm() + 
                                inv_m4*_lC_d_grads.col(3).squaredNorm() + 
                                alpha_d/(dt*dt);
                const double k1 = s*(-C_h - alpha_h / (dt*dt) * lambda_hs(i));
                const double k2 = s*(-C_d - alpha_d / (dt*dt) * lambda_ds(i));

                const double detA = a11*a22 - a21*a12;

                dlam_hs(i) = (k1*a22 - k2*a12) / detA;
                dlam_ds(i) = (a11*k2 - a21*k1) / detA;

                // add contributions to constraint residual
                const double new_lam_d = lambda_ds(i) + dlam_ds(i);
                const double new_lam_h = lambda_hs(i) + dlam_hs(i);


                // update nodal forces (delC*lambda)
                nodal_forces.row(elem(0)) += inv_m1 * (_lC_h_grads.col(0) * new_lam_h + _lC_d_grads.col(0) * new_lam_d);
                nodal_forces.row(elem(1)) += inv_m2 * (_lC_h_grads.col(1) * new_lam_h + _lC_d_grads.col(1) * new_lam_d);
                nodal_forces.row(elem(2)) += inv_m3 * (_lC_h_grads.col(2) * new_lam_h + _lC_d_grads.col(2) * new_lam_d);
                nodal_forces.row(elem(3)) += inv_m4 * (_lC_h_grads.col(3) * new_lam_h + _lC_d_grads.col(3) * new_lam_d);
            
            }

            _vertices = inertial_positions + nodal_forces;
            _calculateResiduals(dt, inertial_positions, lambda_hs+dlam_hs, lambda_ds+dlam_ds);

            std::cout << "Last res: " << last_c_res_rms << "\tNew res: " << _constraint_residual_rms <<std::endl;
            if (_constraint_residual_rms < last_c_res_rms)
            {
                // update Lagrange multipliers
                lambda_hs += dlam_hs;
                lambda_ds += dlam_ds;
                last_c_res_rms = _constraint_residual_rms;
                latest_vertices = _vertices;
                break;
            }

            _vertices = latest_vertices;
            s /= 5.0;
            std::cout << "Constraint residual increased!\tNew h scaling: " << s << std::endl;
        }

        
        
        // if the residual policy is to calculate every iteration, do that
        if (_residual_policy == XPBDResidualPolicy::EVERY_ITERATION)
        {
            if (const OutputSimulation* output_sim = dynamic_cast<const OutputSimulation*>(_sim))
            {
                _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
                // if we are calculating the residuals every Gauss Seidel iteration, it's likely we want to create
                // a residual vs. iteration number plot, so write info to file
                output_sim->printInfo();
            }
        }
    }

    // if the residual policy is to calculate every substep, do that
    if (_residual_policy == XPBDResidualPolicy::EVERY_SUBSTEP)
    {
        _calculateResiduals(dt, inertial_positions, lambda_hs, lambda_ds);
    }
}

inline void XPBDMeshObject::_computeF(const unsigned elem_index, Eigen::Matrix3d& X, Eigen::Matrix3d& F)
{
    // create the deformed shape matrix from current deformed vertex positions
    X.col(0) = _vertices.row(_elements(elem_index,0)) - _vertices.row(_elements(elem_index,3));
    X.col(1) = _vertices.row(_elements(elem_index,1)) - _vertices.row(_elements(elem_index,3));
    X.col(2) = _vertices.row(_elements(elem_index,2)) - _vertices.row(_elements(elem_index,3));

    // compute F
    F = X * _Q[elem_index];
}

inline double XPBDMeshObject::_computeDeviatoricConstraint(const Eigen::Matrix3d& F, const Eigen::Matrix3d& Q, Eigen::Matrix<double, 3, 4>& C_d_grads)
{
    // compute constraint itself    
    const double C_d = std::sqrt(F.col(0).squaredNorm() + F.col(1).squaredNorm() + F.col(2).squaredNorm());

    // compute constraint gradient
    C_d_grads.block<3,3>(0,0) = 1/C_d * (F * Q.transpose());
    C_d_grads.col(3) = -C_d_grads.col(0) - C_d_grads.col(1) - C_d_grads.col(2);

    return C_d;
}

inline double XPBDMeshObject::_computeHydrostaticConstraint(const Eigen::Matrix3d& F, const Eigen::Matrix3d& Q, Eigen::Matrix3d& F_cross, Eigen::Matrix<double, 3, 4>& C_h_grads)
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

void XPBDMeshObject::_calculateForces()
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

void XPBDMeshObject::_calculateForcesSplitDeviatoric()
{
    Eigen::Matrix<double, -1, 3> del_C_alpha_C = Eigen::MatrixXd::Zero(_vertices.rows(), 3);

    for (int i = 0; i < _elements.rows(); i++)
    {
        const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);

        _computeF(i, _lX, _lF);

        /** HYDROSTATIC CONSTRAINT */
        // compute constraint itself
        const double C_h = _computeHydrostaticConstraint(_lF, _Q[i], _lF_cross, _lC_h_grads);
        // compute the hydrostatic alpha
        const double alpha_h = 1/(_material.lambda() * _vols[i]);

        const double alpha_d = 1/(_material.mu() * _vols[i]);

        // compute forces
        del_C_alpha_C.row(elem(0)) += _lC_h_grads.col(0) / alpha_h * C_h;
        del_C_alpha_C(elem(0), 0) += _lF(0,0) / alpha_d * _Q[i](0,0) + _lF(0,1) / alpha_d * _Q[i](0,1) + _lF(0,2) / alpha_d * _Q[i](0,2);
        del_C_alpha_C(elem(0), 1) += _lF(1,0) / alpha_d * _Q[i](0,0) + _lF(1,1) / alpha_d * _Q[i](0,1) + _lF(1,2) / alpha_d * _Q[i](0,2);
        del_C_alpha_C(elem(0), 2) += _lF(2,0) / alpha_d * _Q[i](0,0) + _lF(2,1) / alpha_d * _Q[i](0,1) + _lF(2,2) / alpha_d * _Q[i](0,2);

        del_C_alpha_C.row(elem(1)) += _lC_h_grads.col(1) / alpha_h * C_h;
        del_C_alpha_C(elem(1), 0) += _lF(0,0) / alpha_d * _Q[i](1,0) + _lF(0,1) / alpha_d * _Q[i](1,1) + _lF(0,2) / alpha_d * _Q[i](1,2);
        del_C_alpha_C(elem(1), 1) += _lF(1,0) / alpha_d * _Q[i](1,0) + _lF(1,1) / alpha_d * _Q[i](1,1) + _lF(1,2) / alpha_d * _Q[i](1,2);
        del_C_alpha_C(elem(1), 2) += _lF(2,0) / alpha_d * _Q[i](1,0) + _lF(2,1) / alpha_d * _Q[i](1,1) + _lF(2,2) / alpha_d * _Q[i](1,2);

        del_C_alpha_C.row(elem(2)) += _lC_h_grads.col(2) / alpha_h * C_h;
        del_C_alpha_C(elem(2), 0) += _lF(0,0) / alpha_d * _Q[i](2,0) + _lF(0,1) / alpha_d * _Q[i](2,1) + _lF(0,2) / alpha_d * _Q[i](2,2);
        del_C_alpha_C(elem(2), 1) += _lF(1,0) / alpha_d * _Q[i](2,0) + _lF(1,1) / alpha_d * _Q[i](2,1) + _lF(1,2) / alpha_d * _Q[i](2,2);
        del_C_alpha_C(elem(2), 2) += _lF(2,0) / alpha_d * _Q[i](2,0) + _lF(2,1) / alpha_d * _Q[i](2,1) + _lF(2,2) / alpha_d * _Q[i](2,2);

        del_C_alpha_C.row(elem(3)) += _lC_h_grads.col(3) / alpha_h * C_h;
        del_C_alpha_C(elem(3), 0) += _lF(0,0) / alpha_d * (-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0)) + _lF(0,1) / alpha_d * (-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1)) + _lF(0,2) / alpha_d * (-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2));
        del_C_alpha_C(elem(3), 1) += _lF(1,0) / alpha_d * (-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0)) + _lF(1,1) / alpha_d * (-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1)) + _lF(1,2) / alpha_d * (-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2));
        del_C_alpha_C(elem(3), 2) += _lF(2,0) / alpha_d * (-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0)) + _lF(2,1) / alpha_d * (-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1)) + _lF(2,2) / alpha_d * (-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2));
    }

    const double forces_abs_mean = del_C_alpha_C.cwiseAbs().mean();
    const double forces_rms = std::sqrt(del_C_alpha_C.squaredNorm()/del_C_alpha_C.size());

    std::cout << name() << " Forces:" << std::endl;
    std::cout << "\tRMS:\t" << forces_rms << "\t\tAverage of abs coeffs:\t" << forces_abs_mean << std::endl;

    
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
    Eigen::Matrix3d X, F, F_cross, _lC_h_grads, _lC_d_grads;
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

        // primary residual
        del_C_lam.row(elem(0)) += _lC_h_grads.col(0) * lambda_hs(i) + _lC_d_grads.col(0) * lambda_ds(i);
        del_C_lam.row(elem(1)) += _lC_h_grads.col(1) * lambda_hs(i) + _lC_d_grads.col(1) * lambda_ds(i);
        del_C_lam.row(elem(2)) += _lC_h_grads.col(2) * lambda_hs(i) + _lC_d_grads.col(2) * lambda_ds(i);
        del_C_lam.row(elem(3)) += C_h_grad_4 * lambda_hs(i) + C_d_grad_4 * lambda_ds(i); 

        // constraint residual
        const double c_res_h = C_h + alpha_h / (dt*dt) * lambda_hs(i);
        const double c_res_d = C_d + alpha_d / (dt*dt) * lambda_ds(i);
        // std::cout << "C_h: " << C_h << "\talpha_h: " << alpha_h << "\tlambda_h: " << lambda_hs(i) << std::endl;
        // std::cout << "C_d: " << C_d << "\talpha_d: " << alpha_d << "\tlambda_d: " << lambda_ds(i) << std::endl;
        constraint_residual(2*i) = c_res_h;
        constraint_residual(2*i+1) = c_res_d;

        // compute forces
        del_C_alpha_C.row(elem(0)) += _lC_h_grads.col(0) / alpha_h * C_h + _lC_d_grads.col(0) / alpha_d * C_d;
        del_C_alpha_C.row(elem(1)) += _lC_h_grads.col(1) / alpha_h * C_h + _lC_d_grads.col(1) / alpha_d * C_d;
        del_C_alpha_C.row(elem(2)) += _lC_h_grads.col(2) / alpha_h * C_h + _lC_d_grads.col(2) / alpha_d * C_d;
        del_C_alpha_C.row(elem(3)) += C_h_grad_4 / alpha_h * C_h + C_d_grad_4 / alpha_d * C_d;
    }

     

   
    _primary_residual = Mx - del_C_lam;
    // const double primary_residual_abs_mean = primary_residual.cwiseAbs().mean();
    const double primary_residual_rms = std::sqrt(_primary_residual.squaredNorm()/_primary_residual.size());
    
    VerticesMat Mxdt2 = Mx.array() / (dt*dt);
    // std::cout << "mxdt2: " << Mxdt2(0,2) << "\tforce: " << del_C_alpha_C(0,2) << std::endl;
    // std::cout << Mx << std::endl;
    VerticesMat dynamics_residual = Mx - del_C_alpha_C*(dt*dt);
    // std::cout << dynamics_residual << std::endl;
    // const double dynamics_residual_abs_mean = dynamics_residual.cwiseAbs().mean();
    const double dynamics_residual_rms = std::sqrt(dynamics_residual.squaredNorm()/dynamics_residual.size());

    const double constraint_residual_rms = std::sqrt(constraint_residual.squaredNorm()/constraint_residual.size());
    // const double constraint_residual_abs_mean = constraint_residual.cwiseAbs().mean();

    _primary_residual_rms = primary_residual_rms;
    _constraint_residual_rms = constraint_residual_rms;
    _dynamics_residual_rms = dynamics_residual_rms;
    _vol_ratio = cur_vols.sum() / _vols.sum();

    // std::cout << primary_residual << std::endl;

    // std::cout << name() << " Residuals:" << std::endl;
    // std::cout << "\tDynamics residual\t--\tRMS:\t" << dynamics_residual_rms << "\t\tAverage:\t" << dynamics_residual_abs_mean << std::endl;
    // std::cout << "\tPrimary residual\t--\tRMS:\t" << primary_residual_rms << "\t\tAverage:\t" << primary_residual_abs_mean << std::endl;
    // std::cout << "\tConstraint residual\t--\tRMS:\t" << constraint_residual_rms << "\t\tAverage:\t" << constraint_residual_abs_mean << std::endl;
    // // std::cout << "\tLambda_h(0):\t" << lambda_hs(0) << "\tLambda_d(0):\t" << lambda_ds(0) << std::endl;
    // std::cout << "\tCurrent volume / initial volume:\t" << cur_vols.sum() / _vols.sum() << std::endl;
    // std::cout << "\tConstraint residuals for first element: " << constraint_residual(0) << "\t, " << constraint_residual(1) << std::endl;
    // // std::cout << "\tForces:\n" << forces << std::endl;
}

void XPBDMeshObject::_calculateResidualsSplitDeviatoric(const double dt, const VerticesMat& inertial_positions, const Eigen::MatrixXd& lambdas)
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

    // Eigen::VectorXd lambdas(_elements.rows() * 2);
    Eigen::VectorXd constraint_residual(_elements.rows() * 10);

    Eigen::VectorXd cur_vols(_elements.rows());

    for (int i = 0; i < _elements.rows(); i++)
    {
        const Eigen::Matrix<unsigned, 1, 4>& elem = _elements.row(i);

        _computeF(i, _lX, _lF);

        cur_vols(i) = std::abs(_lX.determinant()/6);

        

        /** HYDROSTATIC CONSTRAINT */
        // compute constraint itself
        const double C_h = _computeHydrostaticConstraint(_lF, _Q[i], _lF_cross, _lC_h_grads);
        // compute the hydrostatic alpha
        const double alpha_h = 1/(_material.lambda() * _vols[i]);

        const double alpha_d = 1/(_material.mu() * _vols[i]);

        // primary residual
        del_C_lam.row(elem(0)) += _lC_h_grads.col(0) * lambdas(i,9);
        del_C_lam(elem(0), 0) += lambdas(i,0) * _Q[i](0,0) + lambdas(i,1) * _Q[i](0,1) + lambdas(i,2) * _Q[i](0,2);
        del_C_lam(elem(0), 1) += lambdas(i,3) * _Q[i](0,0) + lambdas(i,4) * _Q[i](0,1) + lambdas(i,5) * _Q[i](0,2);
        del_C_lam(elem(0), 2) += lambdas(i,6) * _Q[i](0,0) + lambdas(i,7) * _Q[i](0,1) + lambdas(i,8) * _Q[i](0,2);

        del_C_lam.row(elem(1)) += _lC_h_grads.col(1) * lambdas(i,9);
        del_C_lam(elem(1), 0) += lambdas(i,0) * _Q[i](1,0) + lambdas(i,1) * _Q[i](1,1) + lambdas(i,2) * _Q[i](1,2);
        del_C_lam(elem(1), 1) += lambdas(i,3) * _Q[i](1,0) + lambdas(i,4) * _Q[i](1,1) + lambdas(i,5) * _Q[i](1,2);
        del_C_lam(elem(1), 2) += lambdas(i,6) * _Q[i](1,0) + lambdas(i,7) * _Q[i](1,1) + lambdas(i,8) * _Q[i](1,2);

        del_C_lam.row(elem(2)) += _lC_h_grads.col(2) * lambdas(i,9);
        del_C_lam(elem(2), 0) += lambdas(i,0) * _Q[i](2,0) + lambdas(i,1) * _Q[i](2,1) + lambdas(i,2) * _Q[i](2,2);
        del_C_lam(elem(2), 1) += lambdas(i,3) * _Q[i](2,0) + lambdas(i,4) * _Q[i](2,1) + lambdas(i,5) * _Q[i](2,2);
        del_C_lam(elem(2), 2) += lambdas(i,6) * _Q[i](2,0) + lambdas(i,7) * _Q[i](2,1) + lambdas(i,8) * _Q[i](2,2);

        del_C_lam.row(elem(3)) += _lC_h_grads.col(3) * lambdas(i,9);
        del_C_lam(elem(3), 0) += lambdas(i,0) * (-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0)) + lambdas(i,1) * (-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1)) + lambdas(i,2) * (-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2));
        del_C_lam(elem(3), 1) += lambdas(i,3) * (-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0)) + lambdas(i,4) * (-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1)) + lambdas(i,5) * (-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2));
        del_C_lam(elem(3), 2) += lambdas(i,6) * (-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0)) + lambdas(i,7) * (-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1)) + lambdas(i,8) * (-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2));

        // constraint residual
        constraint_residual(10*i) = _lF(0,0) + alpha_d / (dt*dt) * lambdas(i,0);
        constraint_residual(10*i+1) = _lF(0,1) + alpha_d / (dt*dt) * lambdas(i,1);
        constraint_residual(10*i+2) = _lF(0,2) + alpha_d / (dt*dt) * lambdas(i,2);
        constraint_residual(10*i+3) = _lF(1,0) + alpha_d / (dt*dt) * lambdas(i,3);
        constraint_residual(10*i+4) = _lF(1,1) + alpha_d / (dt*dt) * lambdas(i,4);
        constraint_residual(10*i+5) = _lF(1,2) + alpha_d / (dt*dt) * lambdas(i,5);
        constraint_residual(10*i+6) = _lF(2,0) + alpha_d / (dt*dt) * lambdas(i,6);
        constraint_residual(10*i+7) = _lF(2,1) + alpha_d / (dt*dt) * lambdas(i,7);
        constraint_residual(10*i+8) = _lF(2,2) + alpha_d / (dt*dt) * lambdas(i,8);
        constraint_residual(10*i+9) = C_h + alpha_h / (dt*dt) * lambdas(i,9);

        // compute forces
        del_C_alpha_C.row(elem(0)) += _lC_h_grads.col(0) / alpha_h * C_h;
        del_C_alpha_C(elem(0), 0) += _lF(0,0) / alpha_d * _Q[i](0,0) + _lF(0,1) / alpha_d * _Q[i](0,1) + _lF(0,2) / alpha_d * _Q[i](0,2);
        del_C_alpha_C(elem(0), 1) += _lF(1,0) / alpha_d * _Q[i](0,0) + _lF(1,1) / alpha_d * _Q[i](0,1) + _lF(1,2) / alpha_d * _Q[i](0,2);
        del_C_alpha_C(elem(0), 2) += _lF(2,0) / alpha_d * _Q[i](0,0) + _lF(2,1) / alpha_d * _Q[i](0,1) + _lF(2,2) / alpha_d * _Q[i](0,2);

        del_C_alpha_C.row(elem(1)) += _lC_h_grads.col(1) / alpha_h * C_h;
        del_C_alpha_C(elem(1), 0) += _lF(0,0) / alpha_d * _Q[i](1,0) + _lF(0,1) / alpha_d * _Q[i](1,1) + _lF(0,2) / alpha_d * _Q[i](1,2);
        del_C_alpha_C(elem(1), 1) += _lF(1,0) / alpha_d * _Q[i](1,0) + _lF(1,1) / alpha_d * _Q[i](1,1) + _lF(1,2) / alpha_d * _Q[i](1,2);
        del_C_alpha_C(elem(1), 2) += _lF(2,0) / alpha_d * _Q[i](1,0) + _lF(2,1) / alpha_d * _Q[i](1,1) + _lF(2,2) / alpha_d * _Q[i](1,2);

        del_C_alpha_C.row(elem(2)) += _lC_h_grads.col(2) / alpha_h * C_h;
        del_C_alpha_C(elem(2), 0) += _lF(0,0) / alpha_d * _Q[i](2,0) + _lF(0,1) / alpha_d * _Q[i](2,1) + _lF(0,2) / alpha_d * _Q[i](2,2);
        del_C_alpha_C(elem(2), 1) += _lF(1,0) / alpha_d * _Q[i](2,0) + _lF(1,1) / alpha_d * _Q[i](2,1) + _lF(1,2) / alpha_d * _Q[i](2,2);
        del_C_alpha_C(elem(2), 2) += _lF(2,0) / alpha_d * _Q[i](2,0) + _lF(2,1) / alpha_d * _Q[i](2,1) + _lF(2,2) / alpha_d * _Q[i](2,2);

        del_C_alpha_C.row(elem(3)) += _lC_h_grads.col(3) / alpha_h * C_h;
        del_C_alpha_C(elem(3), 0) += _lF(0,0) / alpha_d * (-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0)) + _lF(0,1) / alpha_d * (-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1)) + _lF(0,2) / alpha_d * (-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2));
        del_C_alpha_C(elem(3), 1) += _lF(1,0) / alpha_d * (-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0)) + _lF(1,1) / alpha_d * (-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1)) + _lF(1,2) / alpha_d * (-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2));
        del_C_alpha_C(elem(3), 2) += _lF(2,0) / alpha_d * (-_Q[i](0,0) - _Q[i](1,0) - _Q[i](2,0)) + _lF(2,1) / alpha_d * (-_Q[i](0,1) - _Q[i](1,1) - _Q[i](2,1)) + _lF(2,2) / alpha_d * (-_Q[i](0,2) - _Q[i](1,2) - _Q[i](2,2));
    }

     

   
    _primary_residual = Mx - del_C_lam;
    // const double primary_residual_abs_mean = primary_residual.cwiseAbs().mean();
    const double primary_residual_rms = std::sqrt(_primary_residual.squaredNorm()/_primary_residual.size());
    
    VerticesMat Mxdt2 = Mx.array() / (dt*dt);
    // std::cout << "mxdt2: " << Mxdt2(0,2) << "\tforce: " << del_C_alpha_C(0,2) << std::endl;
    // std::cout << Mx << std::endl;
    VerticesMat dynamics_residual = Mx - del_C_alpha_C*(dt*dt);
    // std::cout << dynamics_residual << std::endl;
    // const double dynamics_residual_abs_mean = dynamics_residual.cwiseAbs().mean();
    const double dynamics_residual_rms = std::sqrt(dynamics_residual.squaredNorm()/dynamics_residual.size());

    const double constraint_residual_rms = std::sqrt(constraint_residual.squaredNorm()/constraint_residual.size());
    // const double constraint_residual_abs_mean = constraint_residual.cwiseAbs().mean();

    _primary_residual_rms = primary_residual_rms;
    _constraint_residual_rms = constraint_residual_rms;
    _dynamics_residual_rms = dynamics_residual_rms;
    _vol_ratio = cur_vols.sum() / _vols.sum();

    // std::cout << name() << " Residuals:" << std::endl;
    // std::cout << "\tDynamics residual\t--\tRMS:\t" << dynamics_residual_rms << "\t\tAverage:\t" << dynamics_residual_abs_mean << std::endl;
    // std::cout << "\tPrimary residual\t--\tRMS:\t" << primary_residual_rms << "\t\tAverage:\t" << primary_residual_abs_mean << std::endl;
    // std::cout << "\tConstraint residual\t--\tRMS:\t" << constraint_residual_rms << "\t\tAverage:\t" << constraint_residual_abs_mean << std::endl;
    // // std::cout << "\tLambda_h(0):\t" << lambda_hs(0) << "\tLambda_d(0):\t" << lambda_ds(0) << std::endl;
    // std::cout << "\tCurrent volume / initial volume:\t" << cur_vols.sum() / _vols.sum() << std::endl;
    // std::cout << "\tConstraint residuals for first element: " << constraint_residual(0) << "\t, " << constraint_residual(1) << std::endl;
    // std::cout << "\tForces:\n" << forces << std::endl;
}

void XPBDMeshObject::_projectCollisionConstraints(const double dt)
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

void XPBDMeshObject::_updateVelocities(const double dt)
{
    // velocities are simply (cur_pos - last_pos) / deltaT
    _v = (_vertices - _x_prev) / dt;
    // set _x_prev to be ready for the next substep
    _x_prev = _vertices;
}
