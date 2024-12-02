#include "simobject/XPBDMeshObject.hpp"
#include "simulation/Simulation.hpp"
#include "solver/XPBDGaussSeidelSolver.hpp"
#include "solver/HydrostaticConstraint.hpp"
#include "solver/DeviatoricConstraint.hpp"
#include "solver/ConstraintProjectorDecorator.hpp"

XPBDMeshObject::XPBDMeshObject(const XPBDMeshObjectConfig* config)
    : ElasticMeshObject(config)
{
    std::cout << "XPBDMeshObject constructor!" << std::endl;

    _solver_type = config->solverType().value();
    _damping_gamma = config->dampingGamma().value();


    _constraints_with_residual = config->withResidual().value();
    _constraints_with_damping = config->withDamping().value();
    _constraint_type = config->constraintType().value();
    

    _num_solver_iters = config->numSolverIters().value();
    _residual_policy = config->residualPolicy().value();
    
}

XPBDMeshObject::~XPBDMeshObject()
{

}

void XPBDMeshObject::setup()
{
    _createSolver(_solver_type, _num_solver_iters, _residual_policy);
    _createConstraints(_constraint_type, _constraints_with_residual, _constraints_with_damping);
    // _createSolver(_solver_type, _num_solver_iters, _residual_policy);
    
    std::cout << "done with setup!" << std::endl;
}

void XPBDMeshObject::_createConstraints(XPBDConstraintType constraint_type, bool with_residual, bool with_damping)
{
    // create constraint(s) for each element
    for (unsigned i = 0; i < numElements(); i++)
    {
        const unsigned v0 = _elements(i,0);
        const unsigned v1 = _elements(i,1);
        const unsigned v2 = _elements(i,2);
        const unsigned v3 = _elements(i,3);
        if (constraint_type == XPBDConstraintType::STABLE_NEOHOOKEAN)
        {
            std::unique_ptr<Solver::Constraint> hyd_constraint, dev_constraint;
            hyd_constraint = std::make_unique<Solver::HydrostaticConstraint>(this, v0, v1, v2, v3);
            dev_constraint = std::make_unique<Solver::DeviatoricConstraint>(this, v0, v1, v2, v3);

            if (with_residual)
            {
                assert(0);
                // hyd_constraint = std::make_unique<Solver::WithPrimaryResidual>(std::move(hyd_constraint));
                // dev_constraint = std::make_unique<Solver::WithPrimaryResidual>(std::move(dev_constraint));
            }

            if (with_damping)
            {
                assert(0);
                // hyd_constraint = std::make_unique<Solver::WithDamping>(std::move(hyd_constraint), _damping_gamma);
                // dev_constraint = std::make_unique<Solver::WithDamping>(std::move(dev_constraint), _damping_gamma);
            }
            
            std::vector<Solver::Constraint*> hyd_vec; hyd_vec.push_back(hyd_constraint.get());
            std::unique_ptr<Solver::ConstraintProjector> hyd_projector = std::make_unique<Solver::ConstraintProjector>(hyd_vec, _dt);
            std::vector<Solver::Constraint*> dev_vec; dev_vec.push_back(dev_constraint.get());
            std::unique_ptr<Solver::ConstraintProjector> dev_projector = std::make_unique<Solver::ConstraintProjector>(dev_vec, _dt);

            _constraints.push_back(std::move(dev_constraint));
            _constraints.push_back(std::move(hyd_constraint));

            // _constraint_projectors.push_back(std::move(dev_projector));
            // _constraint_projectors.push_back(std::move(hyd_projector));
            _solver->addConstraintProjector(std::move(dev_projector));
            _solver->addConstraintProjector(std::move(hyd_projector));
            
        }
        else if (constraint_type == XPBDConstraintType::STABLE_NEOHOOKEAN_COMBINED)
        {
            std::unique_ptr<Solver::Constraint> hyd_constraint, dev_constraint;
            hyd_constraint = std::make_unique<Solver::HydrostaticConstraint>(this, v0, v1, v2, v3);
            dev_constraint = std::make_unique<Solver::DeviatoricConstraint>(this, v0, v1, v2, v3);

            if (with_residual)
            {
                assert(0);
                // hyd_constraint = std::make_unique<Solver::WithPrimaryResidual>(std::move(hyd_constraint));
                // dev_constraint = std::make_unique<Solver::WithPrimaryResidual>(std::move(dev_constraint));
            }

            if (with_damping)
            {
                assert(0);
                // hyd_constraint = std::make_unique<Solver::WithDamping>(std::move(hyd_constraint), _damping_gamma);
                // dev_constraint = std::make_unique<Solver::WithDamping>(std::move(dev_constraint), _damping_gamma);
            }
            
            std::vector<Solver::Constraint*> vec; vec.push_back(hyd_constraint.get()); vec.push_back(dev_constraint.get());
            std::unique_ptr<Solver::ConstraintProjector> projector = std::make_unique<Solver::ConstraintProjector>(vec, _dt);

            _constraints.push_back(std::move(dev_constraint));
            _constraints.push_back(std::move(hyd_constraint));

            // _constraint_projectors.push_back(std::move(projector));
            _solver->addConstraintProjector(std::move(projector));
        }
    }
}

void XPBDMeshObject::_createSolver(XPBDSolverType solver_type, unsigned num_solver_iters, XPBDResidualPolicy residual_policy)
{
    if (solver_type == XPBDSolverType::GAUSS_SEIDEL)
    {
        _solver = std::make_unique<Solver::XPBDGaussSeidelSolver>(this, num_solver_iters, residual_policy);
    }
    else if (solver_type == XPBDSolverType::JACOBI)
    {
        std::cout << "\nJacobi solver not implemented yet!" << std::endl;
        assert(0);
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