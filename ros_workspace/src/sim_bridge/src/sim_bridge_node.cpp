#include <cstdio>

#include "sim_bridge/SimBridge.hpp"

#include "config/VirtuosoTissueGraspingSimulationConfig.hpp"
#include "simulation/VirtuosoTissueGraspingSimulation.hpp"

#include <mutex>
#include <condition_variable>

// Global synchronization objects
std::mutex mtx;
std::condition_variable cv;
bool setup_complete = false;

void runSim(Sim::Simulation* sim)
{
    // std::cout << "Sim dt: " << sim->dt() << std::endl;
    sim->setup();

    // notify the main thread that the simulation has completed setup
    {
        std::lock_guard<std::mutex> l(mtx);
        setup_complete = true;
    }
    cv.notify_one();

    sim->run();
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);


    std::string config_filename;
    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];
        if (arg == "--config-filename" && i+1 < argc)
        {
            config_filename = argv[++i];
        }
    }
    std::cout << "Loading config from config filename: " << config_filename << std::endl;

    // const std::string config_filename = "../config/demos/virtuoso_trachea/virtuoso_trachea.yaml";
    VirtuosoTissueGraspingSimulationConfig config(YAML::LoadFile(config_filename));
    Sim::VirtuosoTissueGraspingSimulation sim(&config);

    std::thread sim_thread(runSim, &sim);

    // wait for the simulation to be set up
    {
        std::unique_lock<std::mutex> l(mtx);
        cv.wait(l, [] { return setup_complete; });
    }

    rclcpp::spin(std::make_shared<SimBridge>(&sim));

    sim_thread.join();

    rclcpp::shutdown();
    return 0;
}
