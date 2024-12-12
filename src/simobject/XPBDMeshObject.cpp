#include "simobject/XPBDMeshObject.hpp"
#include "simulation/Simulation.hpp"
#include "solver/XPBDGaussSeidelSolver.hpp"
#include "solver/CollisionConstraint.hpp"
#include "solver/HydrostaticConstraint.hpp"
#include "solver/DeviatoricConstraint.hpp"
#include "solver/ConstraintProjectorDecorator.hpp"
#include "solver/CombinedNeohookeanConstraintProjector.hpp"

XPBDMeshObject::XPBDMeshObject(const XPBDMeshObjectConfig* config)
    : ElasticMeshObject(config)
{
    /* extract values from the Config object */

    // solver options
    _solver_type = config->solverType().value();
    _num_solver_iters = config->numSolverIters().value();
    _residual_policy = config->residualPolicy().value();
    
    // constraint specifications
    _constraints_with_residual = config->withResidual().value();
    _constraints_with_damping = config->withDamping().value();
    _constraint_type = config->constraintType().value();

    _damping_gamma = config->dampingGamma().value();
}

XPBDMeshObject::~XPBDMeshObject()
{

}

void XPBDMeshObject::setup()
{
    _createSolver(_solver_type, _num_solver_iters, _residual_policy);       // create the Solver object first and then add ConstraintProjectors to it
    _createConstraints(_constraint_type, _constraints_with_residual, _constraints_with_damping, false);     // create constraints and add ConstraintProjectors to the solver object
}

unsigned XPBDMeshObject::numConstraintsForPosition(const unsigned index) const
{
    if (_constraint_type == XPBDConstraintType::STABLE_NEOHOOKEAN)
    {
        return 2*_v_attached_elements[index];   // if sequential constraints are used, there are 2 constraints per element ==> # of constraint updates = 2 * # of elements attached to that vertex
    }
    else if (_constraint_type == XPBDConstraintType::STABLE_NEOHOOKEAN_COMBINED)
    {
        return _v_attached_elements[index];     // if combined constraints are used, there are 2 constraints per element but they are solved together ==> # of constraint updates = # of elements attached to that vertex
    }
}

void XPBDMeshObject::addCollisionConstraint(XPBDMeshObject* vertex_obj, unsigned vertex_ind, XPBDMeshObject* face_obj, unsigned face_ind)
{
    std::unique_ptr<Solver::CollisionConstraint> collision_constraint = std::make_unique<Solver::CollisionConstraint>(vertex_obj, vertex_ind,
                                                                                                                    face_obj, face_obj->faces()(face_ind,0),
                                                                                                                    face_obj->faces()(face_ind,1), face_obj->faces()(face_ind,2));
    std::vector<Solver::Constraint*> collision_vec; collision_vec.push_back(collision_constraint.get());
    std::unique_ptr<Solver::ConstraintProjector> collision_projector = std::make_unique<Solver::ConstraintProjector>(collision_vec, _dt);

    unsigned index = _solver->addConstraintProjector(std::move(collision_projector));
    XPBDCollisionConstraint xpbd_collision_constraint;
    xpbd_collision_constraint.constraint = std::move(collision_constraint);
    xpbd_collision_constraint.projector_index = index;
    xpbd_collision_constraint.num_steps_unused = 0;

    _collision_constraints.push_back(std::move(xpbd_collision_constraint));
}

void XPBDMeshObject::clearCollisionConstraints()
{
    for (const auto& c : _collision_constraints)
    {
        _solver->removeConstraintProjector(c.projector_index);
    }
    _collision_constraints.clear();
}

void XPBDMeshObject::removeOldCollisionConstraints(const unsigned threshold)
{
    for (int i = _collision_constraints.size() - 1; i >= 0; i--)
    {
        if (_collision_constraints[i].num_steps_unused >= threshold)
        {
            // std::cout << "OLD COLLISION CONSTRAINT REMOVED!" << std::endl;
            _solver->removeConstraintProjector(_collision_constraints[i].projector_index);
            _collision_constraints.erase(_collision_constraints.begin() + i);
        }
    }
}

void XPBDMeshObject::_createConstraints(XPBDConstraintType constraint_type, bool with_residual, bool with_damping, bool first_order)
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

            std::vector<Solver::Constraint*> hyd_vec; hyd_vec.push_back(hyd_constraint.get());
            std::unique_ptr<Solver::ConstraintProjector> hyd_projector = std::make_unique<Solver::ConstraintProjector>(hyd_vec, _dt);
            std::vector<Solver::Constraint*> dev_vec; dev_vec.push_back(dev_constraint.get());
            std::unique_ptr<Solver::ConstraintProjector> dev_projector = std::make_unique<Solver::ConstraintProjector>(dev_vec, _dt);

            _elastic_constraints.push_back(std::move(dev_constraint));
            _elastic_constraints.push_back(std::move(hyd_constraint));

            _solver->addConstraintProjector(_decorateConstraintProjector(std::move(dev_projector), with_residual, with_damping, first_order));
            _solver->addConstraintProjector(_decorateConstraintProjector(std::move(hyd_projector), with_residual, with_damping, first_order));
            
        }
        else if (constraint_type == XPBDConstraintType::STABLE_NEOHOOKEAN_COMBINED)
        {
            std::unique_ptr<Solver::Constraint> hyd_constraint, dev_constraint;
            hyd_constraint = std::make_unique<Solver::HydrostaticConstraint>(this, v0, v1, v2, v3);
            dev_constraint = std::make_unique<Solver::DeviatoricConstraint>(this, v0, v1, v2, v3);
            
            std::vector<Solver::Constraint*> vec; vec.push_back(dev_constraint.get()); vec.push_back(hyd_constraint.get());
            std::unique_ptr<Solver::CombinedNeohookeanConstraintProjector> projector = std::make_unique<Solver::CombinedNeohookeanConstraintProjector>(vec, _dt);

            _elastic_constraints.push_back(std::move(dev_constraint));
            _elastic_constraints.push_back(std::move(hyd_constraint));
            
            _solver->addConstraintProjector( _decorateConstraintProjector(std::move(projector), with_residual, with_damping, first_order));
        }
    }
}

std::unique_ptr<Solver::ConstraintProjector> XPBDMeshObject::_decorateConstraintProjector(std::unique_ptr<Solver::ConstraintProjector> projector, bool with_residual, bool with_damping, bool first_order)
{
    if (with_damping)
    {
        projector = std::make_unique<Solver::WithDamping>(std::move(projector), _damping_gamma);        // wrap the ConstraintProjector with the WithDamping decorator
    }

    if (first_order)
    {
        projector = std::make_unique<Solver::FirstOrder>(std::move(projector));                         // wrap the ConstraintProjector with the FirstOrder decorator
    }

    // IMPORTANT: the WithDistributedPrimaryResidual should be applied last so that XPBDSolver can downcast and use the setPrimaryResidual() method.
    // if, for example, the WithDisributedPrimaryResidual is wrapped with a FirstOrder decorator, a downcast to WithDistributedPrimaryResidual will fail,
    //  and there won't be any way to point the WithDistributedPrimaryResidual decorator to the primary residual vector of the XPBDSolver, resulting in a segfault.
    // this is probably indicative of bad design and needs some rethinking, but it's okay for now
    if (with_residual)
    {
        projector = std::make_unique<Solver::WithDistributedPrimaryResidual>(std::move(projector));     // wrap the ConstraintProjector with the WithDistributedPRimaryResidual decorator
    }

    return projector;
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

    // update collision constraints unused
    for (auto& c : _collision_constraints)
    {
        if (_solver->constraintProjectors()[c.projector_index]->lambda()[0] != 0)
        {
            c.num_steps_unused = 0;
        }
        else
        {
            c.num_steps_unused++;
        }
    }

    // TODO: replace with real collision detection
    for (unsigned i = 0; i < numVertices(); i++)
    {
        if (vertexFixed(i))
        {
            _vertices.row(i) = _x_prev.row(i);
        }
        
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