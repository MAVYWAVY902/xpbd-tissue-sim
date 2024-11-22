#include "simobject/XPBDMeshObject.hpp"
#include "simulation/Simulation.hpp"
#include "solver/XPBDGaussSeidelSolver.hpp"
#include "solver/HydrostaticConstraint.hpp"
#include "solver/DeviatoricConstraint.hpp"
#include "solver/ConstraintDecorator.hpp"

XPBDMeshObject::XPBDMeshObject(const XPBDMeshObjectConfig* config)
    : ElasticMeshObject(config)
{

    _solver_type = config->solverType().value();
    _damping_gamma = config->dampingGamma().value();

    XPBDConstraintType constraint_type = config->constraintType().value();
    bool with_residual = config->withResidual().value();
    bool with_damping = config->withDamping().value();
    _createConstraints(constraint_type, with_residual, with_damping);

    unsigned num_iters = config->numSolverIters().value();
    XPBDResidualPolicy residual_policy = config->residualPolicy().value();

    if (_solver_type == XPBDSolverType::GAUSS_SEIDEL)
    {
        _solver = std::make_unique<Solver::XPBDGaussSeidelSolver>(this, num_iters, residual_policy);
    }
    else if (_solver_type == XPBDSolverType::JACOBI)
    {
        std::cout << "\nJacobi solver not implemented yet!" << std::endl;
        assert(0);
    }
    
}

void XPBDMeshObject::_createConstraints(XPBDConstraintType constraint_type, bool with_residual, bool with_damping)
{
    // create a constraint for each element
    for (unsigned i = 0; i < numElements(); i++)
    {
        const unsigned v0 = _elements(i,0);
        const unsigned v1 = _elements(i,1);
        const unsigned v2 = _elements(i,2);
        const unsigned v3 = _elements(i,3);
        if (constraint_type == XPBDConstraintType::STABLE_NEOHOOKEAN)
        {
            std::unique_ptr<Solver::Constraint> hyd_constraint, dev_constraint;
            hyd_constraint = std::make_unique<Solver::HydrostaticConstraint>(_dt, this, v0, v1, v2, v3);
            dev_constraint = std::make_unique<Solver::DeviatoricConstraint>(_dt, this, v0, v1, v2, v3);

            if (with_residual)
            {
                hyd_constraint = std::make_unique<Solver::WithPrimaryResidual>(std::move(hyd_constraint));
                dev_constraint = std::make_unique<Solver::WithPrimaryResidual>(std::move(dev_constraint));
            }

            if (with_damping)
            {
                hyd_constraint = std::make_unique<Solver::WithDamping>(std::move(hyd_constraint), _damping_gamma);
                dev_constraint = std::make_unique<Solver::WithDamping>(std::move(dev_constraint), _damping_gamma);
            }
            
            _constraints.push_back(std::move(hyd_constraint));
            _constraints.push_back(std::move(dev_constraint));
        }
        else if (constraint_type == XPBDConstraintType::STABLE_NEOHOOKEAN_COMBINED)
        {
            std::cout << "\nCombined Neohookean constraints not implemented yet!" << std::endl;
            assert(0);
        }
    }

}

std::string XPBDMeshObject::toString() const
{
    // TODO: complete toString
    return ElasticMeshObject::toString();
}

void XPBDMeshObject::update()
{
    _movePositionsInertially();
    _projectConstraints();
    _updateVelocities();
}

void XPBDMeshObject::_movePositionsInertially()
{
    // move vertices according to their velocity
    _vertices += _dt*_v;
    // external forces (right now just gravity, which acts in -z direction)
    for (int i = 0; i < _vertices.rows(); i++)
    {
        _vertices(i,2) += -_sim->gAccel() * _dt * _dt;
    }
}

void XPBDMeshObject::_projectConstraints()
{
    _solver->solve();

    // TODO: replace with real collision detection
    for (unsigned i = 0; i < numVertices(); i++)
    {
        if (_vertices(i,2) <= 0)
        {
            _vertices(i,2) = 0;
        }
    }
}

void XPBDMeshObject::_updateVelocities()
{
    // velocities are simply (cur_pos - last_pos) / deltaT
    _v = (_vertices - _x_prev) / _dt;
    // set _x_prev to be ready for the next substep
    _x_prev = _vertices;
}