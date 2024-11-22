#include "simobject/FirstOrderXPBDMeshObject.hpp"
#include "solver/ConstraintDecorator.hpp"
#include "simulation/Simulation.hpp"

FirstOrderXPBDMeshObject::FirstOrderXPBDMeshObject(const FirstOrderXPBDMeshObjectConfig* config)
    : XPBDMeshObject(config)
{
    _damping_multiplier = config->dampingMultiplier().value();

    _calculatePerVertexDamping();
}

void FirstOrderXPBDMeshObject::_calculatePerVertexDamping()
{
    _B = Eigen::VectorXd::Zero(numVertices());
    for (unsigned i = 0; i < numVertices(); i++)
    {
        _B(i) = _v_volume(i) * _damping_multiplier;
    }

    std::cout << _B << std::endl;
}

std::string FirstOrderXPBDMeshObject::toString() const
{
    // TODO: better toString
    return XPBDMeshObject::toString();
}

void FirstOrderXPBDMeshObject::_createConstraints(XPBDConstraintType constraint_type, bool with_residual, bool with_damping)
{
    XPBDMeshObject::_createConstraints(constraint_type, with_residual, with_damping);

    for (unsigned i = 0; i < _constraints.size(); i++)
    {
        // convert all constraints to first order constraints
        std::unique_ptr<Solver::Constraint> first_order_constraint = std::make_unique<Solver::FirstOrder>(std::move(_constraints.at(i)));
    }
}

void FirstOrderXPBDMeshObject::_movePositionsInertially()
{
    for (int i = 0; i < _vertices.rows(); i++)
    {
        _vertices(i,2) += -_sim->gAccel() * _m(i) * _dt / _B(i);
    }   
}