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
}

void FirstOrderXPBDMeshObject::_calculatePerVertexQuantities()
{
    XPBDMeshObject::_calculatePerVertexQuantities();

    _inv_B.resize(_mesh->numVertices());
    for (unsigned i = 0; i < _mesh->numVertices(); i++)
    {
        _inv_B[i] = 1.0 / (_vertex_volumes[i] * _damping_multiplier);
    }
}

// void FirstOrderXPBDMeshObject::setup()
// {
//     _calculatePerVertexDamping();
//     _createSolver(_solver_type, _num_solver_iters, _residual_policy);
//     _createConstraints(_constraint_type, _constraints_with_residual, _constraints_with_damping, true);
// }

std::string FirstOrderXPBDMeshObject::toString(const int indent) const
{
    // TODO: better toString
    return XPBDMeshObject::toString(indent);
}

void FirstOrderXPBDMeshObject::_movePositionsInertially()
{
    for (int i = 0; i < _mesh->numVertices(); i++)
    {
        _mesh->displaceVertex(i, 0, 0, -_sim->gAccel() * _vertex_masses[i] * _sim->dt() * _inv_B[i]);
    }   
}

} //namespace Sim