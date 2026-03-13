#include "geometry/MeshSDF.hpp"

#include "simobject/RigidMeshObject.hpp"

#ifdef HAVE_CUDA
#include "gpu/resource/MeshSDFGPUResource.hpp"
#endif

namespace Geometry
{

MeshSDF::MeshSDF(const Sim::RigidMeshObject* mesh_obj, const Config::RigidMeshObjectConfig* config)
    : SDF(), _mesh_obj(mesh_obj), _use_original_coords(config && config->useOriginalCoords()), _sdf_offset(Vec3r::Zero())
{
    if (config && config->sdfFilename().has_value())
    {
        _from_file = true;
        // load SDF from file
        _sdf = mesh2sdf::MeshSDF(config->sdfFilename().value());
        // TODO: Need to store/load _sdf_offset for pre-generated SDFs
    }
    else
    {
        // ROOT FIX: SDF must ALWAYS be generated in body frame (centered at origin, unrotated)
        // This is independent of use-original-coords setting
        // The globalToBody() transformation handles the coordinate conversion
        
        Geometry::Mesh mesh_copy(*(mesh_obj->mesh()));
        
        // Transform mesh to body frame: center at origin, remove rotation
        // This is the same for both use-original-coords modes because:
        // - In standard mode: mesh is already at _p with rotation _q
        // - In use-original-coords mode: mesh is at its mass center (which equals _p) with rotation _q
        mesh_copy.moveTogether(-mesh_obj->position());
        const Mat3r rot_mat = GeometryUtils::quatToMat(GeometryUtils::inverseQuat(mesh_obj->orientation()));
        mesh_copy.rotateAbout(Vec3r::Zero(), rot_mat);
        
        // Compute SDF in body frame (always centered at origin)
        _sdf = mesh2sdf::MeshSDF(mesh_copy.vertices(), mesh_copy.faces(), 128, 5, true);

        // No offset needed - globalToBody() handles all coordinate conversion
        _sdf_offset = Vec3r::Zero();
        
        // Print SDF bounding box for debugging
        const mesh2sdf::BoundingBox sdf_bbox = _sdf.gridBoundingBox();
        std::cout << "[MeshSDF] " << mesh_obj->name() << ": SDF generated in body frame\n";
        std::cout << "  Body position (_p): (" << mesh_obj->position().transpose() << ")\n";
        std::cout << "  SDF bbox: (" << sdf_bbox.first[0] << ", " << sdf_bbox.first[1] << ", " 
                  << sdf_bbox.first[2] << ") to (" << sdf_bbox.second[0] << ", " 
                  << sdf_bbox.second[1] << ", " << sdf_bbox.second[2] << ")\n";
    }
}

inline Real MeshSDF::evaluate(const Vec3r& x) const
{
    // ROOT FIX: ALWAYS use globalToBody() transformation
    // This works correctly for both modes because:
    // - Standard mode: _p is set by config, mesh moved to _p
    // - use-original-coords mode: _p equals mesh mass center, mesh stays at mass center
    // Either way: globalToBody(x) = rotate(x - _p) correctly transforms to body frame
    
    const Vec3r x_body = _mesh_obj->globalToBody(x);
    
    // SDF may not be centered about the origin (if loaded from file)
    if (_from_file)
    {
        // TODO: fix alignment between SDF coordinates and body coordinates
        const mesh2sdf::BoundingBox sdf_mesh_bbox = _sdf.meshBoundingBox();
        const Vec3r sdf_mesh_size = sdf_mesh_bbox.second - sdf_mesh_bbox.first;
        const Vec3r obj_mesh_size = _mesh_obj->mesh()->unrotatedSize();
        const Vec3r sdf_mesh_cm = _sdf.meshMassCenter();
        const Vec3r scaling_factors_xyz = sdf_mesh_size.array() / obj_mesh_size.array();
        const Vec3r x_sdf = sdf_mesh_cm.array() + x_body.array() * scaling_factors_xyz.array();
        const Real dist = _sdf.evaluate(x_sdf);
        const Vec3r grad = _sdf.gradient(x_sdf);
        const Vec3r scaled_dist_vec =  grad.array() * dist / scaling_factors_xyz.array();
        return scaled_dist_vec.norm() * ( (dist < 0) ? -1 : 1);
    }
    
    // SDF is in body frame, x_body is in body frame -> direct query
    return _sdf.evaluate(x_body);
}

inline Vec3r MeshSDF::gradient(const Vec3r& x) const
{
    // ROOT FIX: ALWAYS use globalToBody() transformation (same as evaluate())
    const Vec3r x_body = _mesh_obj->globalToBody(x);
    
    Vec3r grad;
    
    // SDF may not be centered about the origin (if loaded from file)
    if (_from_file)
    {
        // TODO: fix alignment between SDF coordinates and body coordinates
        const mesh2sdf::BoundingBox sdf_mesh_bbox = _sdf.meshBoundingBox();
        const Vec3r sdf_mesh_size = sdf_mesh_bbox.second - sdf_mesh_bbox.first;
        const Vec3r obj_mesh_size = _mesh_obj->mesh()->unrotatedSize();
        const Vec3r sdf_mesh_cm = _sdf.meshMassCenter();
        const Vec3r scaling_factors_xyz = sdf_mesh_size.array() / obj_mesh_size.array();
        const Vec3r x_sdf = sdf_mesh_cm.array() + x_body.array() * scaling_factors_xyz.array();
        grad = _sdf.gradient(x_sdf);
    }
    else
    {
        // SDF is in body frame, query in body frame
        grad = _sdf.gradient(x_body);
    }
    
    // Transform gradient from body frame back to world frame
    return GeometryUtils::rotateVectorByQuat(grad, _mesh_obj->orientation());
}

 #ifdef HAVE_CUDA
inline void MeshSDF::createGPUResource()
{
    _gpu_resource = std::make_unique<Sim::MeshSDFGPUResource>(this);
    _gpu_resource->allocate();
}
 #endif

} // namespace Geometry