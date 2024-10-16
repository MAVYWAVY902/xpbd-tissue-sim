#include "InitialDeformationSimulation.hpp"


int main(int argc, char **argv) 
{
    if (argc > 1)
    {
        std::string config_filename(argv[1]);
        InitialDeformationSimulation sim(config_filename);
        return sim.run();
    }
    else
    {
        std::cerr << "No config file specified!" << std::endl;
    }
    
}