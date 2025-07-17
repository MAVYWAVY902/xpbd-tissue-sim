#ifndef __VIRTUOSO_TISSUE_GRASPING_TISSUE_SIMULATION_CONFIG_HPP
#define __VIRTUOSO_TISSUE_GRASPING_TISSUE_SIMULATION_CONFIG_HPP

#include "config/simulation/VirtuosoSimulationConfig.hpp"

namespace Config
{

class VirtuosoTissueGraspingSimulationConfig : public VirtuosoSimulationConfig
{

    public:
    explicit VirtuosoTissueGraspingSimulationConfig(const YAML::Node& node)
        : VirtuosoSimulationConfig(node)
    {
        _extractParameter("fixed-faces-filename", node, _fixed_faces_filename);
        _extractParameter("tumor-faces-filename", node, _tumor_faces_filename);
        _extractParameter("goal-filename", node, _goal_filename);
        _extractParameter("goals-folder", node, _goals_folder);

        _extractParameter("device-name1", node, _device_name1);
        _extractParameter("device-name2", node, _device_name2);
    }

    std::optional<std::string> fixedFacesFilename() const { return _fixed_faces_filename.value; }
    std::optional<std::string> tumorFacesFilename() const { return _tumor_faces_filename.value; }
    std::optional<std::string> goalFilename() const { return _goal_filename.value; }
    std::optional<std::string> goalsFolder() const { return _goals_folder.value; }

    std::optional<std::string> deviceName1() const { return _device_name1.value; }
    std::optional<std::string> deviceName2() const { return _device_name2.value; }

    protected:
    ConfigParameter<std::optional<std::string>> _fixed_faces_filename;
    ConfigParameter<std::optional<std::string>> _tumor_faces_filename;
    ConfigParameter<std::optional<std::string>> _goal_filename;
    ConfigParameter<std::optional<std::string>> _goals_folder;

    ConfigParameter<std::optional<std::string>> _device_name1;
    ConfigParameter<std::optional<std::string>> _device_name2;
};

} // namespace Config

#endif // __VIRTUOSO_TISSUE_GRASPING_TISSUE_SIMULATION_CONFIG_HPP