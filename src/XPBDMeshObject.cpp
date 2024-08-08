#include "XPBDMeshObject.hpp"

#include <easy3d/renderer/renderer.h>
#include <easy3d/core/types.h>

#include <chrono>

XPBDMeshObject::XPBDMeshObject(const std::string& name, const YAML::Node& config)
    : ElasticMeshObject(name, config)
{
    _init();
    _precomputeQuantities();
}

XPBDMeshObject::XPBDMeshObject(const std::string& name, const std::string& filename, const ElasticMaterial& material)
    : ElasticMeshObject(name, filename, material)
{
    _init();
    _precomputeQuantities();
}

XPBDMeshObject::XPBDMeshObject(const std::string& name, const VerticesMat& verts, const ElementsMat& elems, const ElasticMaterial& material)
    : ElasticMeshObject(name, verts, elems, material)
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
    _vols.resize(_elements.rows());
    // masses are per-vertex
    _m.resize(_vertices.rows(), 0);

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
        _vols.at(i) = vol;

        // compute mass of element
        double m_element = vol * _material.density();
        // add mass contribution of element to each of its vertices
        _m.at(_elements(i,0)) += m_element/4;
        _m.at(_elements(i,1)) += m_element/4;
        _m.at(_elements(i,2)) += m_element/4;
        _m.at(_elements(i,3)) += m_element/4;
    }

    std::cout << "Precomputed quantities" << std::endl;
}

void XPBDMeshObject::update(const double dt)
{
    auto t1 = std::chrono::steady_clock::now();
    _movePositionsIntertially(dt);
    auto t2 = std::chrono::steady_clock::now();
    _projectConstraints(dt);
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

void XPBDMeshObject::_movePositionsIntertially(const double dt)
{
    // move vertices according to their velocity
    _vertices += dt*_v;
    // external forces (right now just gravity, which acts in -z direction)
    _vertices.col(2).array() += -9.81 * dt * dt;

}

void XPBDMeshObject::_projectConstraints(const double dt)
{
    const double lambda_h = 0;
    const double lambda_d = 0;

    // define Eigen loop variables
    Eigen::Matrix3d X, F, F_cross, C_h_grads, C_d_grads;
    Eigen::Vector3d C_h_grad_4, C_d_grad_4;
    for (unsigned i = 0; i < _elements.rows(); i++)
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

        Eigen::Vector2d b {  -C_h - alpha_h * lambda_h / (dt*dt),
                                    -C_d - alpha_d * lambda_d / (dt*dt)    
                                };

        // solve the system
        // TODO: can we use a faster system solve?
        Eigen::ColPivHouseholderQR<Eigen::Matrix2d> dec(A);
        const Eigen::Vector2d& dlambda = dec.solve(b);

        const double dlam_h = dlambda(0);
        const double dlam_d = dlambda(1);
        

        /** SOLVE FOR DLAM SEQUENTIALLY 
        const double dlam_h = (-C_h - alpha_h * lambda_h / (dt*dt)) / 
         ((1/m1)*C_h_grads.col(0).squaredNorm() + (1/m2)*C_h_grads.col(1).squaredNorm() + (1/m3)*C_h_grads.col(2).squaredNorm() + (1/m4)*C_h_grad_4.squaredNorm() + alpha_h/(dt*dt));

        const double dlam_d = (-C_d - alpha_d * lambda_d / (dt*dt)) /
        ((1/m1)*C_d_grads.col(0).squaredNorm() + (1/m2)*C_d_grads.col(1).squaredNorm() + (1/m3)*C_d_grads.col(2).squaredNorm() + (1/m4)*C_d_grad_4.squaredNorm() + alpha_d/(dt*dt)); 
        */

        // update vertex positions
        _vertices.row(elem(0)) += C_h_grads.col(0) * dlam_h/m1;
        _vertices.row(elem(1)) += C_h_grads.col(1) * dlam_h/m2;
        _vertices.row(elem(2)) += C_h_grads.col(2) * dlam_h/m3;
        _vertices.row(elem(3)) += C_h_grad_4 * dlam_h/m4;

        _vertices.row(elem(0)) += C_d_grads.col(0) * dlam_d/m1;
        _vertices.row(elem(1)) += C_d_grads.col(1) * dlam_d/m2;
        _vertices.row(elem(2)) += C_d_grads.col(2) * dlam_d/m3;
        _vertices.row(elem(3)) += C_d_grad_4 * dlam_d/m4;
    }

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
}

void XPBDMeshObject::_updateVelocities(const double dt)
{
    // velocities are simply (cur_pos - last_pos) / deltaT
    _v = (_vertices - _x_prev) / dt;
    // set _x_prev to be ready for the next substep
    _x_prev = _vertices;
}
