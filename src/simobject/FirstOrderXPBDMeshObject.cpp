#include "simobject/FirstOrderXPBDMeshObject.hpp"
#include "solver/ConstraintProjectorDecorator.hpp"
#include "simulation/Simulation.hpp"

namespace Sim
{

FirstOrderXPBDMeshObject::FirstOrderXPBDMeshObject(const Simulation* sim, const FirstOrderXPBDMeshObjectConfig* config)
    : XPBDMeshObject(sim, config)
{
    std::cout << "FirstOrderXPBDMeshObject constructor! " << std::endl;
    _damping_multiplier = config->dampingMultiplier().value();

    std::cout << "Damping Multiplier: " << _damping_multiplier << std::endl;
}

void FirstOrderXPBDMeshObject::_calculatePerVertexQuantities()
{
    XPBDMeshObject::_calculatePerVertexQuantities();

    _inv_B.resize(_mesh->numVertices());
    for (int i = 0; i < _mesh->numVertices(); i++)
    {
        _inv_B[i] = 1.0 / (_vertex_volumes[i] * _damping_multiplier);
    }
}

void FirstOrderXPBDMeshObject::setup()
{
    // TODO: lots of duplicated code here...

    _loadAndConfigureMesh();

    // initialize the previous vertices matrix once we've loaded the mesh
    _previous_vertices = _mesh->vertices();
    _vertex_velocities = Geometry::Mesh::VerticesMat::Zero(3, _mesh->numVertices());

    _calculatePerVertexQuantities();
    _createSolver(_solver_type, _num_solver_iters, _residual_policy);       // create the Solver object first and then add ConstraintProjectors to it
    _createConstraints(_constraint_type, _constraints_with_residual, _constraints_with_damping, true);     // create constraints and add ConstraintProjectors to the solver object
}

std::string FirstOrderXPBDMeshObject::toString(const int indent) const
{
    // TODO: better toString
    return XPBDMeshObject::toString(indent);
}

void FirstOrderXPBDMeshObject::_movePositionsInertially()
{
    for (int i = 0; i < _mesh->numVertices(); i++)
    {
        _mesh->displaceVertex(i, Vec3r(0, 0, -_sim->gAccel() * _vertex_masses[i] * _sim->dt() * _inv_B[i]));
    }   
}

} //namespace Sim