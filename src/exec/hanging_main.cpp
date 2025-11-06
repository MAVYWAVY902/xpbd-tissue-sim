#include "simulation/HangingSimulation.hpp"

int main(int argc, char **argv) 
{
    if (argc > 1)
    {
        std::string config_filename(argv[1]);
        Config::HangingSimulationConfig config(YAML::LoadFile(config_filename));
        Sim::HangingSimulation sim(&config);
        return sim.run();
    }
    else
    {
        std::cerr << "No config file specified!" << std::endl;
        std::cerr << "Usage: " << argv[0] << " <config_file.yaml>" << std::endl;
    }
    
    return -1;
}