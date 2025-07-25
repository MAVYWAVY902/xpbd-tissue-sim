#ifndef __XPBD_MESH_OBJECT_BASE_WRAPPER_HPP
#define __XPBD_MESH_OBJECT_BASE_WRAPPER_HPP

#include "simobject/XPBDMeshObjectBase.hpp"

#include "solver/xpbd_projector/ConstraintProjectorReference.hpp"

#include <variant>

namespace Sim
{

class XPBDMeshObject_BasePtrWrapper
{
public:
    using SDFType = XPBDMeshObject_Base::SDFType;

    template<bool IsFirstOrder>
    XPBDMeshObject_BasePtrWrapper(XPBDMeshObject_Base_<IsFirstOrder>* obj) : _variant(obj) {}

    XPBDMeshObject_BasePtrWrapper() : _variant( (XPBDMeshObject_Base_<true>*)nullptr ) {}

    template<bool IsFirstOrder>
    XPBDMeshObject_Base_<IsFirstOrder>* getAs()
    {
        if (std::holds_alternative<XPBDMeshObject_Base_<IsFirstOrder>>(_variant))
        {
            return std::get<XPBDMeshObject_Base_<IsFirstOrder>>(_variant);
        }
        else
        {
            return nullptr;
        }
    }

    template<bool IsFirstOrder>
    const XPBDMeshObject_Base_<IsFirstOrder>* getAs() const
    {
        if (std::holds_alternative<XPBDMeshObject_Base_<IsFirstOrder>>(_variant))
        {
            return std::get<XPBDMeshObject_Base_<IsFirstOrder>>(_variant);
        }
        else
        {
            return nullptr;
        }
    }

    explicit operator bool() const
    {
        return std::visit([](const auto& obj) { return obj != nullptr; }, _variant);
    }

    /** TODO: should some of these methods use perfect forwarding? */
    const ElasticMaterial& material() const
    {
        return std::visit([](const auto& obj) -> const ElasticMaterial& { return obj->material(); }, _variant);
    }

    const SDFType* SDF() const
    {
        return std::visit([](const auto& obj) -> const SDFType* { return obj->SDF(); }, _variant );
    }

    void fixVertex(int index)
    {
        return std::visit([&](auto& obj) { return obj->fixVertex(index); }, _variant);
    }

    bool vertexFixed(int index) const
    {
        return std::visit([&](const auto& obj) { return obj->vertexFixed(index); }, _variant);
    }

    Real vertexMass(int index) const
    {
        return std::visit([&](const auto& obj) { return obj->vertexMass(index); }, _variant);
    }

    int vertexAttachedElements(int index) const
    {
        return std::visit([&](const auto& obj) { return obj->vertexAttachedElements(index); }, _variant);
    }

    Vec3r vertexVelocity(int index) const
    {
        return std::visit([&](const auto& obj) { return obj->vertexVelocity(index); }, _variant);
    }

    Vec3r vertexPreviousPosition(int index) const
    {
        return std::visit([&](const auto& obj) { return obj->vertexPreviousPosition(index); }, _variant);
    }

    Real vertexConstraintInertia(int index) const
    {
        return std::visit([&](const auto& obj) { return obj->vertexConstraintInertia(index); }, _variant);
    }

    Solver::ConstraintProjectorReferenceWrapper<Solver::StaticDeformableCollisionConstraint>
    addStaticCollisionConstraint(const Geometry::SDF* sdf, const Vec3r& surface_point, const Vec3r& collision_normal,
        int face_ind, const Real u, const Real v, const Real w)
    {
        return std::visit([&](auto& obj)
        {
            return Solver::ConstraintProjectorReferenceWrapper<Solver::StaticDeformableCollisionConstraint>(
                obj->addStaticCollisionConstraint(sdf, surface_point, collision_normal, face_ind, u, v, w)
            );
        }, _variant);
    }

    Solver::ConstraintProjectorReferenceWrapper<Solver::RigidDeformableCollisionConstraint>
    addRigidDeformableCollisionConstraint(const Geometry::SDF* sdf, Sim::RigidObject* rigid_obj, const Vec3r& rigid_body_point, const Vec3r& collision_normal,
        int face_ind, const Real u, const Real v, const Real w)
    {
        return std::visit([&](auto& obj)
        {
            return Solver::ConstraintProjectorReferenceWrapper<Solver::RigidDeformableCollisionConstraint>(
                obj->addRigidDeformableCollisionConstraint(sdf, rigid_obj, rigid_body_point, collision_normal, face_ind, u, v, w)
            );
        }, _variant);
    }

    void clearCollisionConstraints()
    {
        return std::visit([](auto& obj) { obj->clearCollisionConstraints(); }, _variant);
    }

    Solver::ConstraintProjectorReferenceWrapper<Solver::AttachmentConstraint>
    addAttachmentConstraint(int v_ind, const Vec3r* attach_pos_ptr, const Vec3r& attachment_offset)
    {
        return std::visit([&](auto& obj)
        {
            return Solver::ConstraintProjectorReferenceWrapper<Solver::AttachmentConstraint>(
                obj->addAttachmentConstraint(v_ind, attach_pos_ptr, attachment_offset)
            );
        }, _variant);
        
    }

    void clearAttachmentConstraints()
    {
        return std::visit([](auto& obj) { obj->clearAttachmentConstraints(); }, _variant);
    }

    Vec3r elasticForceAtVertex(int index) const
    {
        return std::visit([&](const auto& obj) { return obj->elasticForceAtVertex(index); }, _variant);
    }

    MatXr stiffnessMatrix() const
    {
        return std::visit([](const auto& obj) { return obj->stiffnessMatrix(); }, _variant);
    }

    const Geometry::Mesh* mesh() const
    {
        return std::visit([](const auto& obj) { return obj->mesh(); }, _variant);
    }

    const Geometry::TetMesh* tetMesh() const
    {
        return std::visit([](const auto& obj) { return obj->tetMesh(); }, _variant);
    }

private:
    std::variant<XPBDMeshObject_Base_<true>*, XPBDMeshObject_Base_<false>*> _variant;
};

} // namespace Sim

#endif // __XPBD_MESH_OBJECT_BASE_WRAPPER_HPP