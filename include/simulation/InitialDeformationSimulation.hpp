#ifndef __INITIAL_DEFORMATION_SIMULATION_HPP
#define __INITIAL_DEFORMATION_SIMULATION_HPP

#include "OutputSimulation.hpp"
#include "InitialDeformationSimulationConfig.hpp"
#include <Eigen/Dense>

class InitialDeformationSimulation : public OutputSimulation
{
    public:

    explicit InitialDeformationSimulation(const std::string& config_filename);

    std::string deformationType() const;
    virtual std::string toString() const override;
    virtual std::string type() const override { return "InitialDeformationSimulation"; }

    virtual void setup() override;

    virtual void printInfo() const override;

    protected:
    std::vector<MeshObject::VerticesMat> initial_vertices;
    DeformationType _deformation_type;
    double _deformation_factor;
};

#endif // __INITIAL_DEFORMATION_SIMULATION_HPP