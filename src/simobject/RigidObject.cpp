#include "simobject/RigidObject.hpp"

namespace Sim
{

RigidObject::RigidObject(const Simulation* sim, const RigidObjectConfig* config)
    : Object(sim, config)
{
    _p = config->initialPosition();
    _q = _eulXYZ2Quat(config->initialRotation()[0], config->initialRotation()[1], config->initialRotation()[2]);

    _v = config->initialVelocity();
    _w = config->initialAngularVelocity();
}

RigidObject::RigidObject(const Simulation* sim, const std::string& name)
    : Object(sim, name), _p({0,0,0}), _q({0,0,0,1}), _m(0), _I(Eigen::Matrix3d::Zero()), _v({0,0,0}), _w({0,0,0})
{}

RigidObject::RigidObject(const Simulation* sim, const std::string& name, const Eigen::Vector3d& position, const Eigen::Vector4d& orientation)
    : Object(sim, name), _p(position), _q(orientation), _m(0), _I(Eigen::Matrix3d::Zero()), _v({0,0,0}), _w({0,0,0})
{}

RigidObject::RigidObject(const Simulation* sim, const std::string& name, const Eigen::Vector3d& position, const Eigen::Vector4d& orientation, const double mass, const Eigen::Matrix3d& inertia_mat)
    : Object(sim, name), _p(position), _q(orientation), _m(mass), _I(inertia_mat), _v({0,0,0}), _w({0,0,0})
{}

std::string RigidObject::toString(const int indent) const
{
    const std::string indent_str(indent, '\t');
    std::stringstream ss;
    ss << indent_str << "=====" << type() << "=====" << std::endl;
    ss << indent_str << "Position: (" << _p[0] << ", " << _p[1] << ", " << _p[2] << ")" << std::endl;
    ss << indent_str << "Orientation: (" << _q[0] << ", " << _q[1] << ", " << _q[2] << ", " << _q[3] << std::endl;
    ss << indent_str << "Translational Velocity: (" << _v[0] << ", " << _v[1] << ", " << _v[2] << ")" << std::endl;
    ss << indent_str << "Angular Velocity: (" << _w[0] << ", " << _w[1] << ", " << _w[2] << ")" << std::endl;
    ss << indent_str << "Mass: " << _m << std::endl;
    ss << indent_str << "Inertia Matrix: " << _I << std::endl;

    return ss.str();
}

void RigidObject::update()
{
    // do nothing for now
    // TODO: implement rigid body dynamics here
}

Eigen::Vector4d RigidObject::_eulXYZ2Quat(const double x, const double y, const double z) const
{
    const double q1 = std::sin(0.5*x)*std::cos(0.5*y)*std::cos(0.5*z) - std::cos(0.5*x)*std::sin(0.5*y)*std::sin(0.5*z);
    const double q2 = std::cos(0.5*x)*std::sin(0.5*y)*std::cos(0.5*z) + std::sin(0.5*x)*std::cos(0.5*y)*std::sin(0.5*z);
    const double q3 = std::cos(0.5*x)*std::cos(0.5*y)*std::sin(0.5*z) - std::sin(0.5*x)*std::sin(0.5*y)*std::cos(0.5*z);
    const double w = std::cos(0.5*x)*std::cos(0.5*y)*std::cos(0.5*z) + std::sin(0.5*x)*std::sin(0.5*y)*std::sin(0.5*z);

    return Eigen::Vector4d({q1, q2, q3, w});
}

} // namespace Simulation