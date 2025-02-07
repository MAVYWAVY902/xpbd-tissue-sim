#ifndef __MESH_SDF_HPP
#define __MESH_SDF_HPP

// define MESH2SDF_DOUBLE_PRECISION so the Mesh2SDF library compiles with Real precision
#ifndef HAVE_CUDA
#define MESH2SDF_DOUBLE_PRECISION
#endif

#include <Mesh2SDF/MeshSDF.hpp>

#include "geometry/SDF.hpp"
#include "simobject/RigidMeshObject.hpp"
#include "config/RigidMeshObjectConfig.hpp"

namespace Geometry
{
class MeshSDF : public SDF
{
    public:
    MeshSDF(const Sim::RigidMeshObject* mesh_obj, const RigidMeshObjectConfig* config);

    virtual Real evaluate(const Vec3r& x) const override;

    virtual Vec3r gradient(const Vec3r& x) const override;

 #ifdef HAVE_CUDA
    virtual void createGPUResource() override;
 #endif

    protected:
    const Sim::RigidMeshObject* _mesh_obj;
    mesh2sdf::MeshSDF _sdf;
    bool _from_file;

};

} // namespace Geometry

#endif // __MESH_SDF_HPP