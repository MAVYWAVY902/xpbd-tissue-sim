#include "simobject/RigidMeshObject.hpp"
#include "utils/MeshUtils.hpp"

namespace Sim
{

RigidMeshObject::RigidMeshObject(const Simulation* sim, const RigidMeshObjectConfig* config)
    : RigidObject(sim, config), MeshObject(config, config)
{
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
    // TODO
    // calculate the mass based on the density and the volume of the mesh
    // also calculate the moment of inertia    
}

void RigidMeshObject::update()
{
    // TODO: move the mesh according to rigid body dynamics
    RigidObject::update();
}

} // namespace Simulation