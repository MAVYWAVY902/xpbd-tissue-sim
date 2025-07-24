#include "utils/MeshUtils.hpp"
#include "config/simobject/XPBDMeshObjectConfig.hpp"
#include "simobject/XPBDObjectFactory.hpp"
#include "simobject/XPBDMeshObject.hpp"

#include "config/simulation/SimulationConfig.hpp"
#include "simulation/Simulation.hpp"

#include <fstream>

int main()
{
    gmsh::initialize();

    Real x_size = 1.0;
    Real y_size = 1.0;
    Real z_size = 1.0;
    int num_subdivisions = 2;

    const std::string filename = std::to_string(x_size) + "x" + std::to_string(y_size) + "x" + std::to_string(z_size) + std::to_string(2) + ".msh";
    MeshUtils::createBeamMsh(filename, y_size, x_size, z_size, num_subdivisions);

    Config::XPBDMeshObjectConfig config(
        "test", Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), false, false,
        filename, std::nullopt, std::nullopt,
        false, true, true, Vec4r(1,1,1,1),
        1000, 1e6, 0.3, 0.5, 0.2,
        10, XPBDObjectSolverTypeEnum::GAUSS_SEIDEL,
        XPBDMeshObjectConstraintConfigurationEnum::STABLE_NEOHOOKEAN_COMBINED,
        XPBDSolverResidualPolicyEnum::NEVER,
        Config::ObjectRenderConfig()
    );


    Config::SimulationConfig sim_config;
    Sim::Simulation sim(&sim_config);
    std::unique_ptr<Sim::XPBDMeshObject_Base> xpbd_mesh_obj = config.createObject(&sim);
    xpbd_mesh_obj->setup();
    MatXr stiffness_mat = xpbd_mesh_obj->stiffnessMatrix();

    std::ofstream stiffness_ss("stiffness.txt");
    stiffness_ss << stiffness_mat;
    stiffness_ss.close();

    std::ofstream vertices_ss("vertices.txt");
    vertices_ss << xpbd_mesh_obj->mesh()->vertices().transpose();
    vertices_ss.close();

    std::ofstream elements_ss("elements.txt");
    elements_ss << xpbd_mesh_obj->tetMesh()->elements().transpose();
    elements_ss.close();
}