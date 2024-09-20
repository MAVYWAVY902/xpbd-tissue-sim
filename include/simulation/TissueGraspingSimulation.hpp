#ifndef __TISSUE_GRASPING_SIMULATION_HPP
#define __TISSUE_GRASPING_SIMULATION_HPP

#include "OutputSimulation.hpp"
#include <Eigen/Dense>

class TissueGraspingSimulation : public OutputSimulation
{
    public:

    explicit TissueGraspingSimulation(const std::string& config_filename);

    virtual std::string type() const override { return "TissueGraspingSimulation"; }

    virtual void setup() override;

    virtual void printInfo() const override;

    virtual void notifyMouseButtonPressed(int button, int action, int modifiers) override;

    protected:

    void _toggleTissueGrasping();

    bool _grasping;
    std::vector<unsigned> _grasped_vertices;
    ElasticMeshObject* _tissue_block;
};

#endif // __TISSUE_GRASPING_SIMULATION_HPP