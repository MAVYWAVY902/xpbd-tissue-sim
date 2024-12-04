#include "simobject/FirstOrderXPBDMeshObject.hpp"
#include "solver/ConstraintProjectorDecorator.hpp"
#include "simulation/Simulation.hpp"

FirstOrderXPBDMeshObject::FirstOrderXPBDMeshObject(const FirstOrderXPBDMeshObjectConfig* config)
    : XPBDMeshObject(config)
{
    std::cout << "FirstOrderXPBDMeshObject constructor! " << std::endl;
    _damping_multiplier = config->dampingMultiplier().value();
}

void FirstOrderXPBDMeshObject::_calculatePerVertexDamping()
{
    _B = Eigen::VectorXd::Zero(numVertices());
    for (unsigned i = 0; i < numVertices(); i++)
    {
        _B(i) = _v_volume(i) * _damping_multiplier;
    }

    _inv_B = 1.0/_B.array();
}

void FirstOrderXPBDMeshObject::setup()
{
    _calculatePerVertexDamping();
    _createSolver(_solver_type, _num_solver_iters, _residual_policy);
    _createConstraints(_constraint_type, _constraints_with_residual, _constraints_with_damping, true);
}

std::string FirstOrderXPBDMeshObject::toString() const
{
    // TODO: better toString
    return XPBDMeshObject::toString();
}

void FirstOrderXPBDMeshObject::_movePositionsInertially()
{
    for (int i = 0; i < _vertices.rows(); i++)
    {
        _vertices(i,2) += -_sim->gAccel() * _m(i) * _dt / _B(i);
    }   
}