#ifndef __XPBD_MESH_OBJECT_BASE_HPP
#define __XPBD_MESH_OBJECT_BASE_HPP

#include "simobject/Object.hpp"
#include "simobject/MeshObject.hpp"
#include "simobject/ElasticMaterial.hpp"
#include "simobject/RigidObject.hpp"
#include "geometry/SDF.hpp"
#include "common/XPBDEnumTypes.hpp"

// TODO: resolve circular dependenciees! Too many bandaids everywhere
class XPBDMeshObjectConfig;

namespace Sim
{
    // TODO: XPBDMeshObject_Base used as a band-aid for now...try to remove most (if not all) external references to it
    class XPBDMeshObject_Base : public Object, public TetMeshObject
    {
        public:
        explicit XPBDMeshObject_Base(const Simulation* sim, const XPBDMeshObjectConfig* config);

        virtual ~XPBDMeshObject_Base() {}

        virtual const ElasticMaterial& material() const = 0;

        virtual Real* vertexPreviousPositionPointer(const int index) const = 0;
        virtual void fixVertex(int index) = 0;
        virtual bool vertexFixed(int index) const = 0;
        virtual Real vertexMass(int index) const = 0;
        virtual Real vertexInvMass(int index) const = 0;
        virtual int vertexAttachedElements(int index) const = 0;
        virtual Vec3r vertexVelocity(int index) const = 0;
        virtual Vec3r vertexPreviousPosition(int index) const = 0;
        virtual int numConstraintsForPosition(const int index) const = 0;
        virtual void addStaticCollisionConstraint(const Geometry::SDF* sdf, const Vec3r& p, const Vec3r& n,
            int face_ind, const Real u, const Real v, const Real w) = 0;
        virtual void addRigidDeformableCollisionConstraint(const Geometry::SDF* sdf, Sim::RigidObject* rigid_obj, const Vec3r& rigid_body_point, const Vec3r& collision_normal,
            int face_ind, const Real u, const Real v, const Real w) = 0;
        virtual void clearCollisionConstraints() = 0;
        virtual void removeOldCollisionConstraints(const int threshold) = 0;
    };
}

#endif // __XBPD_MESH_OBJECT_BASE_HPP