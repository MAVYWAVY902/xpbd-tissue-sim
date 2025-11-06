#pragma once

#include "simulation/Simulation.hpp"
#include "config/simulation/SimulationConfig.hpp"
#include "yaml-cpp/yaml.h"

namespace Config
{

/**
 * @brief Configuration class for HangingSimulation
 */
class HangingSimulationConfig : public SimulationConfig
{
public:
    HangingSimulationConfig(const YAML::Node& config_node) : SimulationConfig(config_node) 
    {
        if (config_node["fix-max-z"])
            _fix_max_z = config_node["fix-max-z"].as<bool>();
    }

    bool fixMaxZ() const { return _fix_max_z; }

private:
    bool _fix_max_z = true;         ///< default: fix top vertices (ceiling attachment)
};

} // namespace Config

namespace Sim
{

/**
 * @brief A simulation for hanging objects from an invisible ceiling
 * 
 * This simulation fixes the top vertices of deformable objects to simulate
 * hanging from a ceiling. Useful for testing how objects stretch under gravity
 * with different constraint configurations (e.g., with/without nerve constraints).
 */
class HangingSimulation : public Simulation
{
public:
    HangingSimulation(const Config::HangingSimulationConfig* config);

    void setup() override;

private:
    bool _fix_max_z;  ///< whether to fix top vertices
};

} // namespace Sim