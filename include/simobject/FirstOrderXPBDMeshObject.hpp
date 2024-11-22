#ifndef __FIRST_ORDER_XPBD_MESH_OBJECT_HPP
#define __FIRST_ORDER_XPBD_MESH_OBJECT_HPP

#include "simobject/XPBDMeshObject.hpp"
#include "config/FirstOrderXPBDMeshObjectConfig.hpp"

class FirstOrderXPBDMeshObject : public XPBDMeshObject
{
    public:
    explicit FirstOrderXPBDMeshObject(const FirstOrderXPBDMeshObjectConfig* config);

    virtual std::string toString() const override;
    virtual std::string type() const override { return "FirstOrderXPBDMeshObject"; }

    virtual void setup() override;

    double vertexDamping(const unsigned index) { return _B(index); }

    double vertexInvDamping(const unsigned index) { return _inv_B(index); }

    protected:
    /** Moves the vertices in the absence of constraints.
     * i.e. according to their current velocities and the forces applied to them
     */
    virtual void _movePositionsInertially() override;

    private:
    virtual void _calculatePerVertexDamping();

    void _convertConstraintsToFirstOrder();

    protected:
    double _damping_multiplier;

    Eigen::VectorXd _B;

    Eigen::VectorXd _inv_B;

};

#endif // __FIRST_ORDER_XPBD_MESH_OBJECT_HPP