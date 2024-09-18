#include "FastFEMMeshObject.hpp"

FastFEMMeshObject::FastFEMMeshObject(const FastFEMMeshObjectConfig* config)
    : ElasticMeshObject(config)
{
    std::cout << "FastFEMMEshOBject constructor!" << std::endl;
    _init();
    _precomputeQuantities();
    std::cout << "Init and precompute done!" << std::endl;
}

void FastFEMMeshObject::_init()
{
    _RHS = Eigen::VectorXd::Zero(_vertices.rows()*3);
    _RHS_perm = Eigen::VectorXd::Zero(_vertices.rows()*3);
    _K_vec = Eigen::VectorXd::Zero(_elements.rows());

    _rest_volumes = Eigen::VectorXd::Zero(_elements.rows());
    _masses = Eigen::VectorXd::Zero(_vertices.rows());

    _Q.resize(_elements.rows());
    _Dt.resize(_elements.rows());

    _quaternions.resize(_elements.rows());
    for (int i = 0; i < _elements.rows(); i++)
    {
        _quaternions.at(i) = Eigen::Vector4d({0,0,0,1});
    }
}

void FastFEMMeshObject::_precomputeQuantities()
{
    std::vector<Eigen::Triplet<double> > triplets_D;
    std::vector<Eigen::Triplet<double> > triplets_K;
    std::vector<Eigen::Triplet<double> > triplets_M;


    for (int e = 0; e < _elements.rows(); e++)
    {
        Eigen::Matrix<double, 3, 4> Dt;

        // extract the positions of each vertex in this element
        const Eigen::Vector3d& X1 = _vertices.row(_elements(e,0));
        const Eigen::Vector3d& X2 = _vertices.row(_elements(e,1));
        const Eigen::Vector3d& X3 = _vertices.row(_elements(e,2));
        const Eigen::Vector3d& X4 = _vertices.row(_elements(e,3));

        // fill out the X matrix
        // Note: we use the Sifakis 2012 definition, which is to subtract X4 from X1, X2, X3
        //       rather than the Kugelstadt definition, which is to subtract X1 from X2, X3, X4
        Eigen::Matrix3d X;
        X.col(0) = (X1 - X4);
        X.col(1) = (X2 - X4);
        X.col(2) = (X3 - X4);

        // calculate the rest volume = 1/6 the determinant of X
        _rest_volumes(e) = std::abs(X.determinant()) / 6;

        Eigen::Matrix3d B = X.inverse();

        // store rest state matrix for use during volume constraint solve
        _Q[e] = B;

        // fill out the Dt matrix
        // Note: because of how we defined X, the columns are a little different than how they were defined in the Kugelstadt paper
        //       Namely, the first column in the paper is now the last column so that the matrix product Dt*x still produces vec(F).
        Dt.col(0) = B.row(0);
        Dt.col(1) = B.row(1);
        Dt.col(2) = B.row(2);
        Dt.col(3) = -B.row(0) - B.row(1) - B.row(2);

        // store Dt for use during optimization solve
        _Dt[e] = Dt;

        // add contributions to the D sparse matrix
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                triplets_D.push_back(Eigen::Triplet<double>(9*e + 3*j,      3*_elements(e,i),       Dt(j,i)));
                triplets_D.push_back(Eigen::Triplet<double>(9*e + 3*j + 1,  3*_elements(e,i) + 1,   Dt(j,i)));
                triplets_D.push_back(Eigen::Triplet<double>(9*e + 3*j + 2,  3*_elements(e,i) + 2,   Dt(j,i)));
            }
        }

        // create K sparse matrix - bake 2*dt*dt into it
        for (int i = 0; i < 9; i++)
        {
            triplets_K.push_back(Eigen::Triplet<double>(9*e + i, 9*e + i, 2*_dt*_dt*_material.mu()*_rest_volumes(e)));
        }

        _K_vec(e) = 2*_dt*_dt*_material.mu()*_rest_volumes(e);

        // calculate mass of tetrahedron and add mass contribution to its nodes
        const double m = _material.density()*_rest_volumes(e);

        _masses(_elements(e,0)) += 0.25*m;
        _masses(_elements(e,1)) += 0.25*m;
        _masses(_elements(e,2)) += 0.25*m;
        _masses(_elements(e,3)) += 0.25*m;
    }

    // assemble the sparse M matrix
    for (int v = 0; v < _vertices.rows(); v++)
    {
        for (int i = 0; i < 3; i++)
        {
            triplets_M.push_back(Eigen::Triplet<double>(3*v + i, 3*v + i, _masses(v)));
        }
    }

    // form matrices from triplets
    Eigen::SparseMatrix<double> K(9*_elements.rows(), 9*_elements.rows());
    Eigen::SparseMatrix<double> D(9*_elements.rows(), 3*_vertices.rows());
    Eigen::SparseMatrix<double> M(3*_vertices.rows(), 3*_vertices.rows());

    K.setFromTriplets(triplets_K.begin(), triplets_K.end());
    D.setFromTriplets(triplets_D.begin(), triplets_D.end());
    M.setFromTriplets(triplets_M.begin(), triplets_M.end());

    // compute system matrix and Cholesky factorization
    Eigen::SparseMatrix<double> M_plus_DT_K_D = (M + D.transpose() * K * D);

    Eigen::SimplicialLLT<Eigen::SparseMatrix<double>, Eigen::Lower, Eigen::NaturalOrdering<int>> LLT;
    LLT.compute(M_plus_DT_K_D);
    _perm = LLT.permutationP();
    _perm_inv = LLT.permutationPinv();
    _matL = LLT.matrixL();
    _matLT = LLT.matrixU();

}

std::string FastFEMMeshObject::toString() const
{
    return ElasticMeshObject::toString();
}

void FastFEMMeshObject::update(const double dt, const double g_accel)
{
    _movePositionsInertially(g_accel);
    _solveOptimizationProblem();
    _solveVolumeConstraints();
    _solveCollisionConstraints();
    _updateVelocities();
}

void FastFEMMeshObject::_movePositionsInertially(const double g_accel)
{
    // move vertices according to their velocity
    _vertices += _dt*_v;
    // external forces (right now just gravity, which acts in -z direction)
    for (int i = 0; i < _vertices.rows(); i++)
    {
        _vertices(i,2) += -g_accel * _dt * _dt;
    }
}

void FastFEMMeshObject::_solveOptimizationProblem()
{
    _RHS = Eigen::VectorXd::Zero(_vertices.rows()*3);

    for (int e = 0; e < _elements.rows(); e++)
    {
        // compute deformation gradient
        _computeF(e, _lX, _lF);

        // APD
        // _computeAPD(_lF, _quaternions[e]);

        // convert quaternion to rotation matrix
        _quaternionToRotationMatrix(_quaternions[e], _lR);

        // R <- R-F
        _lR -= _lF;

        _RHS(Eigen::seqN(3*_elements(e,0),3)) += _K_vec(e)*(_lR.col(0)*_Dt[e](0,0) + _lR.col(1)*_Dt[e](1,0) + _lR.col(2)*_Dt[e](2,0));
        _RHS(Eigen::seqN(3*_elements(e,1),3)) += _K_vec(e)*(_lR.col(0)*_Dt[e](0,1) + _lR.col(1)*_Dt[e](1,1) + _lR.col(2)*_Dt[e](2,1));
        _RHS(Eigen::seqN(3*_elements(e,2),3)) += _K_vec(e)*(_lR.col(0)*_Dt[e](0,2) + _lR.col(1)*_Dt[e](1,2) + _lR.col(2)*_Dt[e](2,2));
        _RHS(Eigen::seqN(3*_elements(e,3),3)) += _K_vec(e)*(_lR.col(0)*_Dt[e](0,3) + _lR.col(1)*_Dt[e](1,3) + _lR.col(2)*_Dt[e](2,3));
    }

    std::cout << _RHS << std::endl;

    // solve the linear system (taken exactly from Kugelstadt code)
    //permutation of the RHS because of Eigen's fill-in reduction
	// for (size_t i = 0; i < _RHS.size(); i++)
	// 	_RHS_perm[_perm.indices()[i]] = _RHS[i];

	// //foreward substitution
	// for (int k = 0; k<_matL.outerSize(); ++k)
	// 	for (Eigen::SparseMatrix<double, Eigen::ColMajor>::InnerIterator it(_matL, k); it; ++it)
	// 		if (it.row() == it.col())
	// 			_RHS_perm[it.row()] = _RHS_perm[it.row()] / it.value();
	// 		else
	// 			_RHS_perm[it.row()] -= it.value() * _RHS_perm[it.col()];
		
	// //backward substitution
	// for (int k = _matLT.outerSize() - 1; k >= 0 ; --k)
	// 	for (Eigen::SparseMatrix<double, Eigen::ColMajor>::ReverseInnerIterator it(_matLT, k); it; --it)
	// 		if (it.row() == it.col())
	// 			_RHS_perm[it.row()] = _RHS_perm[it.row()] / it.value();
	// 		else
	// 			_RHS_perm[it.row()] -= it.value() * _RHS_perm[it.col()];
	
	// //invert permutation
	// for (size_t i = 0; i < _RHS.size(); i++)
	// 	_RHS[_perm_inv.indices()[i]] = _RHS_perm[i];

    Eigen::SimplicialLLT<Eigen::SparseMatrix<double, Eigen::ColMajor> > solver; 
    Eigen::SparseMatrix<double, Eigen::ColMajor> A = _matL * _matLT;
    solver.compute(A);
    Eigen::VectorXd x = solver.solve(_RHS);

    for (size_t i = 0; i < _vertices.rows(); i++)
    {
        _vertices(i,0) += x(3*i);
        _vertices(i,1) += x(3*i+1);
        _vertices(i,2) += x(3*i+2);
    }

}

void FastFEMMeshObject::_solveVolumeConstraints()
{
    // accumulated hydrostatic Lagrange multipliers
    Eigen::VectorXd lambda_hs = Eigen::VectorXd::Zero(_elements.rows());

    for (unsigned gi = 0; gi < 1; gi++)
    {
        for (int e = 0; e < _elements.rows(); e++)
        {
            const double inv_m1 = 1.0/_masses[_elements(e,0)];
            const double inv_m2 = 1.0/_masses[_elements(e,1)];
            const double inv_m3 = 1.0/_masses[_elements(e,2)];
            const double inv_m4 = 1.0/_masses[_elements(e,3)];


            const double C_h = _computeHydrostaticConstraint(_lF, _Q[e], _lF_cross, _lC_h_grads);

            // compute the hydrostatic alpha
            const double alpha_h = 1/(_material.lambda() * _rest_volumes[e]);

            const double dlam_h = (-C_h - alpha_h * lambda_hs(e) / (_dt*_dt)) / ((
                    (inv_m1)*_lC_h_grads.col(0).squaredNorm() + 
                    (inv_m2)*_lC_h_grads.col(1).squaredNorm() + 
                    (inv_m3)*_lC_h_grads.col(2).squaredNorm() + 
                    (inv_m4)*_lC_h_grads.col(3).squaredNorm()
                 ) + alpha_h/(_dt*_dt));

            _vertices.row(_elements(e,0)) += _lC_h_grads.col(0) * dlam_h * inv_m1;
            _vertices.row(_elements(e,1)) += _lC_h_grads.col(1) * dlam_h * inv_m2;
            _vertices.row(_elements(e,2)) += _lC_h_grads.col(2) * dlam_h * inv_m3;
            _vertices.row(_elements(e,3)) += _lC_h_grads.col(3) * dlam_h * inv_m4;

            // update Lagrange multipliers
            lambda_hs(e) += dlam_h;
        }
    }
}

void FastFEMMeshObject::_solveCollisionConstraints()
{
    // for now, forbid vertex z-positions going below 0
    for (unsigned i = 0; i < _vertices.rows(); i++)
    {
        if (_vertices(i,2) < 0)
        {
            _vertices(i,2) = 0;
        }
    }
}

void FastFEMMeshObject::_updateVelocities()
{
    // velocities are simply (cur_pos - last_pos) / deltaT
    _v = (_vertices - _x_prev) / _dt;
    // set _x_prev to be ready for the next substep
    _x_prev = _vertices;
}

inline void FastFEMMeshObject::_computeF(const unsigned elem_index, Eigen::Matrix3d& X, Eigen::Matrix3d& F)
{
    // create the deformed shape matrix from current deformed vertex positions
    X.col(0) = _vertices.row(_elements(elem_index,0)) - _vertices.row(_elements(elem_index,3));
    X.col(1) = _vertices.row(_elements(elem_index,1)) - _vertices.row(_elements(elem_index,3));
    X.col(2) = _vertices.row(_elements(elem_index,2)) - _vertices.row(_elements(elem_index,3));

    // compute F
    F = X * _Q[elem_index];
}

inline void FastFEMMeshObject::_computeAPD(const Eigen::Matrix3d& F, Eigen::Vector4d& q)
{
    // 1 iteration enough for plausible results
    for (int it = 0; it < 1; it++)
    {
        Eigen::Matrix3d R;
        _quaternionToRotationMatrix(q, R);

        Eigen::Matrix3d B = R.transpose() * F;
        Eigen::Vector3d gradient({B(2,1) - B(1,2), B(0,2) - B(2,0), B(1,0) - B(0,1)});

        // compute Hessian
        const double h00 = B(1,1) + B(2,2);
        const double h11 = B(0,0) + B(2,2);
        const double h22 = B(0,0) + B(1,1);
        const double h01 = -0.5*(B(1,0) + B(0,1));
        const double h02 = -0.5*(B(2,0) + B(0,2));
        const double h12 = -0.5*(B(2,1) + B(1,2));

        const double detH = -h02*h02*h11 + 2*h01*h02*h12 - h00*h12*h12 - h01*h01*h22 + h00*h11*h22;

        const double factor = -0.25 / detH;
        Eigen::Vector3d omega;
        omega[0] = (h11 * h22 - h12 * h12) * gradient[0]
			+ (h02 * h12 - h01 * h22) * gradient[1]
			+ (h01 * h12 - h02 * h11) * gradient[2];
		omega[0] *= factor;

        omega[1] = (h02 * h12 - h01 * h22) * gradient[0]
			+ (h00 * h22 - h02 * h02) * gradient[1]
			+ (h01 * h02 - h00 * h12) * gradient[2];
		omega[1] *= factor;

		omega[2] = (h01 * h12 - h02 * h11) * gradient[0]
			+ (h01 * h02 - h00 * h12) * gradient[1]
			+ (h00 * h11 - h01 * h01) * gradient[2];
		omega[2] *= factor;

        // instead of clamping use gradient descent (I have no idea how this works, taken from the Kugelstadt code)
        if (omega.dot(gradient) > 0.0)
            omega = gradient * -0.125;
        
        const double l_omega2 = omega.squaredNorm();
        const double w = (1.0 - l_omega2) / (1.0 + l_omega2);
        const Eigen::Vector3d vec = omega * (2.0 / (1.0 + l_omega2));
        const Eigen::Vector4d cayley_quat({vec(0), vec(1), vec(2), w});
        q = _quaternionMultiply(q, cayley_quat);

    }
}

inline Eigen::Vector4d FastFEMMeshObject::_quaternionMultiply(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2) 
{
		return
			Eigen::Vector4d({q1[3] * q2[0] + q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1],
				q1[3] * q2[1] - q1[0] * q2[2] + q1[1] * q2[3] + q1[2] * q2[0],
				q1[3] * q2[2] + q1[0] * q2[1] - q1[1] * q2[0] + q1[2] * q2[3],
				q1[3] * q2[3] - q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2]});
}

inline void FastFEMMeshObject::_quaternionToRotationMatrix(const Eigen::Vector4d& q, Eigen::Matrix3d& R)
{
    const double tx = 2.0 * q[0];
    const double ty = 2.0 * q[1];
    const double tz = 2.0 * q[2];
    const double twx = tx*q[3];
    const double twy = ty*q[3];
    const double twz = tz*q[3];
    const double txx = tx*q[0];
    const double txy = ty*q[0];
    const double txz = tz*q[0];
    const double tyy = ty*q[1];
    const double tyz = tz*q[1];
    const double tzz = tz*q[2];

    R(0,0) = 1.0 - (tyy + tzz);
    R(0,1) = txy - twz;
    R(0,2) = txz + twy;
    R(1,0) = txy + twz;
    R(1,1) = 1.0 - (txx + tzz);
    R(1,2) = tyz - twx;
    R(2,0) = txz - twy;
    R(2,1) = tyz + twx;
    R(2,2) = 1.0 - (txx + tyy);
}

inline double FastFEMMeshObject::_computeHydrostaticConstraint(const Eigen::Matrix3d& F, const Eigen::Matrix3d& Q, Eigen::Matrix3d& F_cross, Eigen::Matrix<double, 3, 4>& C_h_grads)
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