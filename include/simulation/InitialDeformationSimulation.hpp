#ifndef __INITIAL_DEFORMATION_SIMULATION_HPP
#define __INITIAL_DEFORMATION_SIMULATION_HPP

#include "simulation/OutputSimulation.hpp"
#include "config/InitialDeformationSimulationConfig.hpp"
#include "common/types.hpp"

namespace Sim
{

namespace Sim
{

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
    Real _deformation_factor;
};

} // namespace Sim

#endif // __INITIAL_DEFORMATION_SIMULATION_HPP