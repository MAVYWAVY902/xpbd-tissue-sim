#include "simulation/VirtuosoSimulation.hpp"

namespace Sim
{

VirtuosoSimulation::VirtuosoSimulation(const std::string& config_filename)
    : Simulation(config_filename)
{
}

VirtuosoSimulation::VirtuosoSimulation()
    : Simulation()
{
}

void VirtuosoSimulation::setup()
{
    Simulation::setup();
    
    // find the VirtuosoArm object
    for (auto& obj : _objects)
    {
        if (VirtuosoArm* arm = dynamic_cast<VirtuosoArm*>(obj.get()))
        {
            _virtuoso_arm = arm;
            break;
        }
    }
}

void VirtuosoSimulation::notifyKeyPressed(int key, int action, int modifiers)
{


    if (action > 0)
    {
        if (key == 81) // Q = CCW inner tube rotation
        {
            const double cur_rot = _virtuoso_arm->innerTubeRotation();
            _virtuoso_arm->setInnerTubeRotation(cur_rot + IT_ROT_RATE*dt());

        }
        else if (key == 87) // W = CCW outer tube rotation
        {
            const double cur_rot = _virtuoso_arm->outerTubeRotation();
            _virtuoso_arm->setOuterTubeRotation(cur_rot + OT_ROT_RATE*dt());
        }
        else if (key == 69) // E = inner tube extension
        {
            const double cur_trans = _virtuoso_arm->innerTubeTranslation();
            _virtuoso_arm->setInnerTubeTranslation(cur_trans + IT_TRANS_RATE*dt());
        }
        else if (key == 82) // R = outer tube extension
        {
            const double cur_trans = _virtuoso_arm->outerTubeTranslation();
            _virtuoso_arm->setOuterTubeTranslation(cur_trans + OT_TRANS_RATE*dt());
        }
        else if (key == 65) // A = CW inner tube rotation
        {
            const double cur_rot = _virtuoso_arm->innerTubeRotation();
            _virtuoso_arm->setInnerTubeRotation(cur_rot - IT_ROT_RATE*dt()); 
        }
        else if (key == 83) // S = CW outer tube rotation
        {
            const double cur_rot = _virtuoso_arm->outerTubeRotation();
            _virtuoso_arm->setOuterTubeRotation(cur_rot - OT_ROT_RATE*dt());
        }
        else if (key == 68) // D = inner tube retraction
        {
            const double cur_trans = _virtuoso_arm->innerTubeTranslation();
            _virtuoso_arm->setInnerTubeTranslation(cur_trans - IT_TRANS_RATE*dt());
        }
        else if (key == 70) // F = outer tube retraction
        {
            const double cur_trans = _virtuoso_arm->outerTubeTranslation();
            _virtuoso_arm->setOuterTubeTranslation(cur_trans - OT_TRANS_RATE*dt());
        }
    }

}

} // namespace Sim
