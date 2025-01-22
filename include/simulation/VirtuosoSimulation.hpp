#ifndef __VIRTUOSO_SIMULATION_HPP
#define __VIRTUOSO_SIMULATION_HPP

#include "simulation/Simulation.hpp"
#include "simobject/VirtuosoArm.hpp"

namespace Sim
{

class VirtuosoSimulation : public Simulation
{
    public:
    VirtuosoSimulation(const std::string& config_filename);

    VirtuosoSimulation();

    virtual std::string type() const override { return "ResidualSimulation"; }

    virtual void setup() override;

    /** Notifies the simulation that a key has been pressed in the viewer.
     * @param key : the key that was pressed
     * @param action : the action performed on the keyboard
     * @param modifiers : the modifiers (i.e. Shift, Ctrl, Alt)
     */
    virtual void notifyKeyPressed(int key, int action, int modifiers) override;

    protected:
    // TODO: make these settable simulation parameters
    constexpr static double IT_ROT_RATE = 10; // rad/s
    constexpr static double IT_TRANS_RATE = 0.02; // m/s
    constexpr static double OT_ROT_RATE = 10; // rad/s
    constexpr static double OT_TRANS_RATE = 0.02; // m/s 
    VirtuosoArm* _virtuoso_arm;
};

} // namespace Sim

#endif // __VIRTUOSO_SIMULATION_HPP