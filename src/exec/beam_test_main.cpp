#include "simulation/BeamSimulation.hpp"


int main(int argc, char **argv) 
{
    if (argc > 1)
    {
        std::string config_filename(argv[1]);
        BeamSimulationConfig config(YAML::LoadFile(config_filename));
        Sim::BeamSimulation sim(&config);
        return sim.run();
    }
    else
    {
        std::cerr << "No config file specified!" << std::endl;
    }
    
}