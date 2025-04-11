#ifndef __RIGID_MESH_OBJECT_HPP
#define __RIGID_MESH_OBJECT_HPP

#include "simobject/RigidObject.hpp"
#include "simobject/MeshObject.hpp"
#include "geometry/Mesh.hpp"
#include "config/RigidMeshObjectConfig.hpp"

namespace Sim
{

class RigidMeshObject : public RigidObject, public MeshObject
{
    public:
    RigidMeshObject(const Simulation* sim, const RigidMeshObjectConfig* config);

    virtual std::string type() const override { return "RigidMeshObject"; }

    virtual std::string toString(const int indent) const override;

    virtual Geometry::AABB boundingBox() const override;

    virtual void setup() override;

    virtual void update() override;

    virtual void setPosition(const Vec3r& position) override;

    virtual void setOrientation(const Vec4r& orientation) override;

 #ifdef HAVE_CUDA
    virtual void createGPUResource() override { assert(0); /* not implemented */ }
 #endif

    protected:
    Real _density;
    std::unique_ptr<Geometry::Mesh> _initial_mesh;
};

} // namespace Simulation

#endif // __RIGID_MESH_OBJECT