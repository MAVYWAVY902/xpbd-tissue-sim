#ifndef __FIRST_ORDER_XPBD_MESH_OBJECT_HPP
#define __FIRST_ORDER_XPBD_MESH_OBJECT_HPP

#include "simobject/XPBDMeshObject.hpp"
#include "config/FirstOrderXPBDMeshObjectConfig.hpp"

namespace Sim
{

class FirstOrderXPBDMeshObject : public XPBDMeshObject
{
    public:
    explicit FirstOrderXPBDMeshObject(const Simulation* sim, const FirstOrderXPBDMeshObjectConfig* config);

    virtual std::string toString(const int indent) const override;
    virtual std::string type() const override { return "FirstOrderXPBDMeshObject"; }

    virtual void setup() override;

    double vertexDamping(const unsigned index) const { return 1.0/_inv_B[index]; }

    double vertexInvDamping(const unsigned index) const { return _inv_B[index]; }

    protected:
    /** Moves the vertices in the absence of constraints.
     * i.e. according to their current velocities and the forces applied to them
     */
    virtual void _movePositionsInertially() override;

    private:
    virtual void _calculatePerVertexQuantities() override;

    protected:
    double _damping_multiplier;

    std::vector<double> _inv_B;

};

} // namespace Sim

#endif // __FIRST_ORDER_XPBD_MESH_OBJECT_HPP