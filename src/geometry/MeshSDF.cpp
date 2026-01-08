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
        // calculate the SDF at the mesh's un-transformed state
        // if the corresponding RigidMeshObject to this SDF has an initial rotation, the mesh is already rotated, which will throw off the SDF
        Geometry::Mesh mesh_copy(*(mesh_obj->mesh()));
        
        // Check if use-original-coords is enabled
        const bool use_original_coords = config && config->useOriginalCoords();
        
        if (!use_original_coords) {
            // Standard behavior: center the mesh at origin for SDF generation
            // untranslate the copy of the mesh
            mesh_copy.moveTogether(-mesh_obj->position());
            // unrotate the copy of the mesh
            const Mat3r rot_mat = GeometryUtils::quatToMat(GeometryUtils::inverseQuat(mesh_obj->orientation()));
            mesh_copy.rotateAbout(Vec3r::Zero(), rot_mat);
            _sdf_offset = Vec3r::Zero();  // SDF centered at body origin
        } else {
            // FIX for use-original-coords: Center SDF at mesh mass center to avoid coordinate mismatch
            // This ensures SDF queries work correctly even when mesh stays in original coordinates
            Vec3r mass_center = mesh_copy.massCenter();
            _sdf_offset = mass_center;  // Store offset for later coordinate conversion
            mesh_copy.moveTogether(-mass_center);  // Center mesh at origin for SDF generation
            
            std::cout << "[MeshSDF] use-original-coords: Centering SDF at mesh mass center\n";
            std::cout << "[MeshSDF]   Original mass center: (" << mass_center.transpose() << ")\n";
            std::cout << "[MeshSDF]   SDF will be centered at origin with offset stored\n";
        }
        
        // compute the SDF (always centered at origin now)
        _sdf = mesh2sdf::MeshSDF(mesh_copy.vertices(), mesh_copy.faces(), 128, 5, true);
        
        // Print SDF bounding box after generation for debugging
        const mesh2sdf::BoundingBox sdf_bbox = _sdf.gridBoundingBox();
        std::cout << "[MeshSDF]   SDF grid bbox (centered): (" 
                  << sdf_bbox.first[0] << ", " << sdf_bbox.first[1] << ", " << sdf_bbox.first[2] << ") to ("
                  << sdf_bbox.second[0] << ", " << sdf_bbox.second[1] << ", " << sdf_bbox.second[2] << ")\n";
    }
}

inline Real MeshSDF::evaluate(const Vec3r& x) const
{
    // When use-original-coords is true, apply offset correction
    // x is in world coordinates, SDF is centered at origin with offset stored
    if (_use_original_coords) {
        // Transform world point by subtracting the stored offset
        // This converts from world coordinates to SDF-centered coordinates
        return _sdf.evaluate(x - _sdf_offset);
    }
    
    // Standard behavior: transform x into body coordinates
    const Vec3r x_body = _mesh_obj->globalToBody(x);
    // SDF may not be centered about the origin
    if (_from_file)
    {
        // TODO: fix alignment between SDF coordinates and body coordinates
        const mesh2sdf::BoundingBox sdf_mesh_bbox = _sdf.meshBoundingBox();
        const Vec3r sdf_mesh_size = sdf_mesh_bbox.second - sdf_mesh_bbox.first;
        const Vec3r obj_mesh_size = _mesh_obj->mesh()->unrotatedSize();
        const Vec3r sdf_mesh_cm = _sdf.meshMassCenter();
        const Vec3r scaling_factors_xyz = sdf_mesh_size.array() / obj_mesh_size.array();
        const Vec3r x_sdf = sdf_mesh_cm.array() + x_body.array() * scaling_factors_xyz.array();
        // std::cout << "x_body: " << x_body[0] << ", " << x_body[1] << ", " << x_body[2] << std::endl;
        // std::cout << "x_sdf: " << x_sdf[0] << ", " << x_sdf[1] << ", " << x_sdf[2] << std::endl;
        const Real dist = _sdf.evaluate(x_sdf);
        // std::cout << "dist: " << dist << std::endl;
        const Vec3r grad = _sdf.gradient(x_sdf);
        const Vec3r scaled_dist_vec =  grad.array() * dist / scaling_factors_xyz.array();
        return scaled_dist_vec.norm() * ( (dist < 0) ? -1 : 1);
    }
    return _sdf.evaluate(x_body);
}

inline Vec3r MeshSDF::gradient(const Vec3r& x) const
{
    // When use-original-coords is true, apply offset correction
    // x is in world coordinates, SDF is centered at origin with offset stored
    if (_use_original_coords) {
        // Transform world point by subtracting the stored offset
        // Gradient direction is independent of translation, so no additional transform needed
        return _sdf.gradient(x - _sdf_offset);
    }
    
    // Standard behavior: transform x into body coordinates
    const Vec3r x_body = _mesh_obj->globalToBody(x);
    Vec3r grad;
    // SDF may not be centered about the origin
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
        grad = _sdf.gradient(x_body);
    }
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