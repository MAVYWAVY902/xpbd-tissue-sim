#include "simobject/RigidObject.hpp"

namespace Simulation
{

RigidObject::RigidObject(const std::string& name)
    : Object(name), _p({0,0,0}), _q({0,0,0,1}), _m(0), _I(Eigen::Matrix3d::Zero()), _v({0,0,0}), _w({0,0,0})
{}

RigidObject::RigidObject(const std::string& name, const Eigen::Vector3d& position, const Eigen::Vector4d& orientation)
    : Object(name), _p(position), _q(orientation), _m(0), _I(Eigen::Matrix3d::Zero()), _v({0,0,0}), _w({0,0,0})
{}

RigidObject::RigidObject(const std::string& name, const Eigen::Vector3d& position, const Eigen::Vector4d& orientation, const double mass, const Eigen::Matrix3d& inertia_mat)
    : Object(name), _p(position), _q(orientation), _m(mass), _I(inertia_mat), _v({0,0,0}), _w({0,0,0})
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

} // namespace Simulation