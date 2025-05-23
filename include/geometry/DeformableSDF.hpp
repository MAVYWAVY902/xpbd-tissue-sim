#ifndef __DEFORMABLE_SDF_HPP
#define __DEFORMABLE_SDF_HPP

#include "geometry/SDF.hpp"
#include "geometry/EmbreeScene.hpp"

#include "simobject/MeshObject.hpp"

namespace Geometry
{

class DeformableSDF : public SDF
{
    public:
    DeformableSDF(const Sim::TetMeshObject* obj, const EmbreeScene* embree_scene);

    virtual Real evaluate(const Vec3r& x) const override;
    virtual Vec3r gradient(const Vec3r& x) const override;

    private:
    const Sim::TetMeshObject* _obj;

    /** For making BVH point queries. We assume that the object has been added to the Embree scene. */
    const EmbreeScene* _embree_scene;
}

} // namespace Geometry

#endif // __DEFORMABLE_SDF_HPP