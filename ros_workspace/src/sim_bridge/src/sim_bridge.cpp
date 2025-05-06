#include <cstdio>

#include "sim_bridge/SimBridge.hpp"

#include "config/VirtuosoSimulationConfig.hpp"
#include "simulation/VirtuosoSimulation.hpp"

void createAndRunSim(Sim::Simulation* sim)
{
    // std::cout << "Sim dt: " << sim->dt() << std::endl;
    sim->run();
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    const std::string config_filename = "../config/demos/virtuoso_trachea/virtuoso_trachea.yaml";
    VirtuosoSimulationConfig config(YAML::LoadFile(config_filename));
    Sim::VirtuosoSimulation sim(&config);

    std::thread sim_thread(createAndRunSim, &sim);

    rclcpp::spin(std::make_shared<SimBridge>());

    sim_thread.join();

    rclcpp::shutdown();
    return 0;
}
