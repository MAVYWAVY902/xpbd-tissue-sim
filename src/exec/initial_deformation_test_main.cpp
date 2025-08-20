#include "simulation/InitialDeformationSimulation.hpp"
#include "config/simulation/InitialDeformationSimulationConfig.hpp"

int main(int argc, char **argv) 
{
    if (argc > 1)
    {
        std::string config_filename(argv[1]);
    Config::InitialDeformationSimulationConfig config(YAML::LoadFile(config_filename));
        Sim::InitialDeformationSimulation sim(&config);
        return sim.run();
    }
    else
    {
        std::cerr << "No config file specified!" << std::endl;
    }
    
}