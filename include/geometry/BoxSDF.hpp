#ifndef __BOX_SDF_HPP
#define __BOX_SDF_HPP

#include "geometry/SDF.hpp"
#include "simobject/RigidPrimitives.hpp"
#include "utils/GeometryUtils.hpp"

namespace Geometry
{

class BoxSDF : public SDF
{
    public:
    BoxSDF(const Sim::RigidBox* box)
        : SDF(), _box(box)
    {}

    virtual double evaluate(const Eigen::Vector3d& x) const override
    {
        const Eigen::Vector3d x_body = GeometryUtils::rotateVectorByQuat(x, GeometryUtils::inverseQuat(_box->orientation())) - _box->position();
        const Eigen::Vector3d q = x_body.cwiseAbs() - _box->size()/2.0;
        // std::cout << "x: " << x[0] << ", " << x[1] << ", " << x[2] << std::endl;
        // std::cout << "x_body: " << x_body[0] << ", " << x_body[1] << ", " << x_body[2] << std::endl;
        const double max_q = q.maxCoeff();
        const double q0 = std::max(q[0], 0.0);
        const double q1 = std::max(q[1], 0.0);
        const double q2 = std::max(q[2], 0.0);
        // std::cout << "BoxSDF distance: " << std::sqrt(q0*q0 + q1*q1 + q2*q2) + std::min(max_q, 0.0) << std::endl;
        return std::sqrt(q0*q0 + q1*q1 + q2*q2) + std::min(max_q, 0.0);
    }

    virtual Eigen::Vector3d gradient(const Eigen::Vector3d& x) const override
    {
        // const Eigen::Vector4d quat_inv = GeometryUtils::inverseQuat(_box->orientation());
        const Eigen::Vector3d x_body = GeometryUtils::rotateVectorByQuat(x, GeometryUtils::inverseQuat(_box->orientation())) - _box->position();
        // std::cout << "quat: " << _box->orientation()[0] << ", " << _box->orientation()[1] << ", " << _box->orientation()[2] << ", " << _box->orientation()[3] << std::endl;
        // std::cout << "quat_inv: " << quat_inv[0] << ", " << quat_inv[1] << ", " << quat_inv[2] << ", " << quat_inv[3] << std::endl;
        // std::cout << "x_body: " << x_body[0] << ", " << x_body[1] << ", " << x_body[2] << std::endl;
        const Eigen::Vector3d q = x_body.cwiseAbs() - _box->size()/2.0;
        const double q0 = std::max(q[0], 0.0);
        const double q1 = std::max(q[1], 0.0);
        const double q2 = std::max(q[2], 0.0);
        const double len = std::sqrt(q0*q0 + q1*q1 + q2*q2) + 1e-10;

        const int sign0 = x_body[0] > 0 ? 1 : -1;
        const int sign1 = x_body[1] > 0 ? 1 : -1;
        const int sign2 = x_body[2] > 0 ? 1 : -1;

        const double max_q = q.maxCoeff();
        const double min_dist = std::min(max_q, 0.0);
        const Eigen::Vector3d grad = 1/len * Eigen::Vector3d({sign0*q0, sign1*q1, sign2*q2}) + 
                                    Eigen::Vector3d({sign0*static_cast<double>(q[0]==min_dist), sign1*static_cast<double>(q[1]==min_dist), sign2*static_cast<double>(q[2]==min_dist)});
        
        // std::cout << "grad: " << grad[0] << ", " << grad[1] << ", " << grad[2] << std::endl;
        return GeometryUtils::rotateVectorByQuat(grad, _box->orientation());
    }

    protected:
    const Sim::RigidBox* _box;

};

} // namespace Geometry

#endif // __BOX_SDF_HPP