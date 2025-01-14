#include "simobject/RigidMeshObject.hpp"
#include "utils/MeshUtils.hpp"
#include "utils/GeometryUtils.hpp"

namespace Sim
{

RigidMeshObject::RigidMeshObject(const Simulation* sim, const RigidMeshObjectConfig* config)
    : RigidObject(sim, config), MeshObject(config, config)
{
    _density = config->density();
}

// RigidMeshObject::RigidMeshObject(const Simulation* sim, const std::string& name, const std::string& filename, const double density)
//     : RigidObject(sim, name), _density(density)
// {
//     _mesh = MeshUtils::loadTetMeshFromGmshFile(filename);
// }

std::string RigidMeshObject::toString(const int indent) const
{
    std::string indent_str(indent, '\t');
    std::stringstream ss;
    ss << indent_str << "=====" << type() << "=====" << std::endl;
    ss << indent_str << "Mesh vertices: " << _mesh->numVertices() << std::endl;
    ss << indent_str << "Mesh faces: " << _mesh->numFaces() << std::endl;
    ss << RigidObject::toString(indent + 1);

    return ss.str(); 
}

Geometry::AABB RigidMeshObject::boundingBox() const
{
    return _mesh->boundingBox();
}

void RigidMeshObject::setup()
{
    _loadAndConfigureMesh();

    // compute mass and inertia properties of mesh
    // TODO: Does it matter that the mesh has already been rotated?
    std::tie(_m, std::ignore, _I) = _mesh->massProperties(_density);
    _I_inv = _I.inverse();
}

void RigidMeshObject::update()
{
    // TODO: move the mesh according to rigid body dynamics
    RigidObject::update();

    // move the mesh accordingly
    const Eigen::Vector4d dq = GeometryUtils::quatMult(GeometryUtils::inverseQuat(_q_prev), _q);
    const Eigen::Matrix3d rot_mat = GeometryUtils::quatToMat(dq);
    const Eigen::Vector3d dx = _p - _p_prev;

    _mesh->moveTogether(dx);
    _mesh->rotateAbout(_p, rot_mat);

}

} // namespace Simulation