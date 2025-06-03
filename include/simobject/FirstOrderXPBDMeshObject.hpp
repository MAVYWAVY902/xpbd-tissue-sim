#ifndef __FIRST_ORDER_XPBD_MESH_OBJECT_HPP
#define __FIRST_ORDER_XPBD_MESH_OBJECT_HPP

#include "simobject/XPBDMeshObject.hpp"
#include "config/simobject/FirstOrderXPBDMeshObjectConfig.hpp"

namespace Sim
{
template<typename SolverType, typename ConstraintTypeList> class FirstOrderXPBDMeshObject;

template<typename SolverType, typename... ConstraintTypes>
class FirstOrderXPBDMeshObject<SolverType, TypeList<ConstraintTypes...>> : public XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>
{
    public:
    using ConfigType = Config::FirstOrderXPBDMeshObjectConfig;

    public:
    explicit FirstOrderXPBDMeshObject(const Simulation* sim, const ConfigType* config);

    virtual std::string toString(const int indent) const override;
    virtual std::string type() const override { return "FirstOrderXPBDMeshObject"; }

    virtual void setup() override;

   //  Real vertexDamping(const unsigned index) const { return 1.0/_inv_B[index]; }

   //  Real vertexInvDamping(const unsigned index) const { return _inv_B[index]; }

   void addStaticCollisionConstraint(const Geometry::SDF* sdf, const Vec3r& p, const Vec3r& n,
      int face_ind, const Real u, const Real v, const Real w);
   
   void addVertexStaticCollisionConstraint(const Geometry::SDF* sdf, const Vec3r& p, const Vec3r& n, int vert_ind);

   void addRigidDeformableCollisionConstraint(const Geometry::SDF* sdf, Sim::RigidObject* rigid_obj, const Vec3r& rigid_body_point, const Vec3r& collision_normal,
      int face_ind, const Real u, const Real v, const Real w);

   void clearCollisionConstraints();

 #ifdef HAVE_CUDA
   //  virtual void createGPUResource() override { assert(0); /* not implemented */ }
 #endif

    protected:
    /** Creates constraints according to the constraint type.
     */
    void _createElasticConstraints();
                                 
    protected:
    /** Moves the vertices in the absence of constraints.
     * i.e. according to their current velocities and the forces applied to them
     */
    virtual void _movePositionsInertially() override;

    private:
    virtual void _calculatePerVertexQuantities() override;

    protected:
    Real _damping_multiplier;

    std::vector<Real> _inv_B;

    std::vector<bool> _vertices_in_collision;

};

} // namespace Sim

#endif // __FIRST_ORDER_XPBD_MESH_OBJECT_HPP