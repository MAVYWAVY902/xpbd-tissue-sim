#include "simobject/FirstOrderXPBDMeshObject.hpp"
#include "simulation/Simulation.hpp"

namespace Sim
{

template<typename SolverType, typename... ConstraintTypes>
FirstOrderXPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::FirstOrderXPBDMeshObject(const Simulation* sim, const FirstOrderXPBDMeshObjectConfig* config)
    : XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>(sim, config)
{
    std::cout << "FirstOrderXPBDMeshObject constructor! " << std::endl;
    _damping_multiplier = config->dampingMultiplier().value();

    std::cout << "Damping Multiplier: " << _damping_multiplier << std::endl;
}

template<typename SolverType, typename... ConstraintTypes>
void FirstOrderXPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::_calculatePerVertexQuantities()
{
    XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::_calculatePerVertexQuantities();

    _inv_B.resize(this->_mesh->numVertices());
    for (int i = 0; i < this->_mesh->numVertices(); i++)
    {
        _inv_B[i] = 1.0 / (this->_vertex_volumes[i] * _damping_multiplier);
    }
}

template<typename SolverType, typename... ConstraintTypes>
void FirstOrderXPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::setup()
{
    // TODO: lots of duplicated code here...

    this->_loadAndConfigureMesh();

    // initialize the previous vertices matrix once we've loaded the mesh
    this->_previous_vertices = this->_mesh->vertices();
    this->_vertex_velocities = Geometry::Mesh::VerticesMat::Zero(3, this->_mesh->numVertices());

    _calculatePerVertexQuantities();
    // _createSolver(_solver_type, _num_solver_iters, _residual_policy);       // create the Solver object first and then add ConstraintProjectors to it
    this->_createConstraints(this->_constraint_type, this->_constraints_with_residual, this->_constraints_with_damping, true);     // create constraints and add ConstraintProjectors to the solver object
}

template<typename SolverType, typename... ConstraintTypes>
std::string FirstOrderXPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::toString(const int indent) const
{
    // TODO: better toString
    return XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::toString(indent);
}

template<typename SolverType, typename... ConstraintTypes>
void FirstOrderXPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::_movePositionsInertially()
{
    for (int i = 0; i < this->_mesh->numVertices(); i++)
    {
        this->_mesh->displaceVertex(i, Vec3r(0, 0, -this->_sim->gAccel() * this->_vertex_masses[i] * this->_sim->dt() * _inv_B[i]));
    }   
}

// CTAD
// template<typename SolverType, typename ...ConstraintTypes> FirstOrderXPBDMeshObject(TypeList<ConstraintTypes...>, const Simulation*, const FirstOrderXPBDMeshObjectConfig* config)
//     -> FirstOrderXPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>;

using SolverTypesStableNeohookean = XPBDMeshObjectSolverTypes<XPBDMeshObjectConstraintConfigurations::StableNeohookean::projector_type_list>;
using SolverTypesStableNeohookeanCombined = XPBDMeshObjectSolverTypes<XPBDMeshObjectConstraintConfigurations::StableNeohookeanCombined::projector_type_list>;
using StableNeohookeanConstraints = XPBDMeshObjectConstraintConfigurations::StableNeohookean::constraint_type_list;
using StableNeohookeanCombinedConstraints = XPBDMeshObjectConstraintConfigurations::StableNeohookeanCombined::constraint_type_list;

// Stable Neohookean constraint config
template class FirstOrderXPBDMeshObject<SolverTypesStableNeohookean::GaussSeidel, StableNeohookeanConstraints>;
template class FirstOrderXPBDMeshObject<SolverTypesStableNeohookean::Jacobi, StableNeohookeanConstraints>;
template class FirstOrderXPBDMeshObject<SolverTypesStableNeohookean::ParallelJacobi, StableNeohookeanConstraints>;

// Stable Neohookean Combined constraint config
template class FirstOrderXPBDMeshObject<SolverTypesStableNeohookeanCombined::GaussSeidel, StableNeohookeanCombinedConstraints>;
template class FirstOrderXPBDMeshObject<SolverTypesStableNeohookeanCombined::Jacobi, StableNeohookeanCombinedConstraints>;
template class FirstOrderXPBDMeshObject<SolverTypesStableNeohookeanCombined::ParallelJacobi, StableNeohookeanCombinedConstraints>;


} //namespace Sim