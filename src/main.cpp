#include "Simulation.hpp"


int main(int argc, char **argv) 
{
    // easy3d::initialize();
    Simulation sim("Test", "/home/smtobin/Documents/UTK/C++/XPBD/config/config.yaml");
    return sim.run();
    // TextRendering viewer("test");
    // viewer.run();
}